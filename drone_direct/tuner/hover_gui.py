"""
drone_direct/tuner/hover_gui.py
================================
Real-time slider UI for hand-tuning drone hover.
Sends binary CRTP packets to the ESP-Drone over UDP.

Keyboard — Mode 2 RC layout (standard transmitter):
  Left stick  (WASD)              Right stick  (Arrow keys)
  W / S  = Throttle up / down     Up / Down    = Pitch forward / back
  A / D  = Yaw left / right       Left / Right = Roll left / right

  Attitude keys return to zero on release (joystick feel).
  W/S accumulate into the throttle slider.

Camera stabilization (optional, on by default):
  Requires phone connected via USB with USB Debugging enabled.
  ArUco marker ID 7 on/above the drone provides position feedback.
  If unavailable the app runs normally — just no automatic hold.

Run:
  cd drone_direct/tuner
  python hover_gui.py
"""

import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from typing import Optional

sys.path.insert(0, os.path.dirname(__file__))
import config as cfg

from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QDoubleSpinBox, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QPushButton, QSizePolicy, QSlider, QVBoxLayout,
    QWidget,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont


# ── Protocol ──────────────────────────────────────────────────────────────────
# False = binary CRTP (ESP-Drone, port 2390)  ← default
# True  = CSV text    (keyboard_control.ino,  port 1234)
USE_CSV_PROTOCOL = False


def _cksum(data: bytes) -> int:
    return sum(data) & 0xFF


def make_rpyt(roll: float, pitch: float, yaw: float, thrust: int) -> bytes:
    if USE_CSV_PROTOCOL:
        x = max(-1.0, min(1.0, roll  / 20.0))
        y = max(-1.0, min(1.0, pitch / 20.0))
        z = max(0.0,  min(1.0, thrust / 65535.0))
        return f"{x:.4f},{y:.4f},{z:.4f},0\n".encode()
    t       = max(0, min(65535, int(thrust)))
    payload = struct.pack('<BfffH', 0x30, float(roll), float(pitch), float(yaw), t)
    return payload + bytes([_cksum(payload)])


# ── PID controller ────────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp; self.ki = ki; self.kd = kd; self.limit = limit
        self.integral = self.prev_error = 0.0
        self.last_t   = time.time()

    def reset(self):
        self.integral = self.prev_error = 0.0
        self.last_t   = time.time()

    def compute(self, error):
        now = time.time()
        dt  = max(now - self.last_t, 0.001)
        self.integral   = max(-self.limit, min(self.limit,
                              self.integral + error * dt))
        d               = (error - self.prev_error) / dt
        out             = self.kp * error + self.ki * self.integral + self.kd * d
        self.prev_error = error
        self.last_t     = now
        return max(-self.limit, min(self.limit, out))


# ── Kalman filter for altitude ────────────────────────────────────────────────
class KalmanHeight:
    """
    1-D Kalman filter with state = [altitude_m, velocity_m_s].
    Fuses noisy Z measurements from solvePnP into a smooth height estimate
    and suppresses camera jitter / momentary detection glitches.
    """
    def __init__(self, q_pos: float = 0.005, q_vel: float = 0.01,
                 r_meas: float = 0.08):
        self.x   = 0.0; self.v = 0.0          # state
        self.P   = [[1.0, 0.0], [0.0, 1.0]]   # covariance
        self.qp  = q_pos; self.qv = q_vel; self.r = r_meas

    def step(self, z: float, dt: float) -> float:
        dt = max(dt, 0.001)
        # ── Predict ──────────────────────────────────────────────────────────
        self.x  += self.v * dt
        p00 = self.P[0][0] + dt*(self.P[1][0] + self.P[0][1]) + dt*dt*self.P[1][1] + self.qp
        p01 = self.P[0][1] + dt * self.P[1][1]
        p10 = self.P[1][0] + dt * self.P[1][1]
        p11 = self.P[1][1] + self.qv
        self.P  = [[p00, p01], [p10, p11]]
        # ── Update ───────────────────────────────────────────────────────────
        S   = self.P[0][0] + self.r
        k0  = self.P[0][0] / S;  k1 = self.P[1][0] / S
        inn = z - self.x
        self.x += k0 * inn;  self.v += k1 * inn
        self.P  = [[(1 - k0)*self.P[0][0], (1 - k0)*self.P[0][1]],
                   [self.P[1][0] - k1*self.P[0][0], self.P[1][1] - k1*self.P[0][1]]]
        return self.x


# ── Camera stabiliser ─────────────────────────────────────────────────────────
_SCRCPY_EXE = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'scrcpy',
    'scrcpy-win64-v3.0.2', 'scrcpy.exe'))


class CameraStabilizer:
    """
    Optional ArUco + PID lateral position hold via phone camera (scrcpy).
    Runs in a daemon thread. Falls back silently if camera is unavailable.
    Outputs (roll, pitch, yaw) corrections added to the base setpoints.
    """
    S_STARTING    = "starting"
    S_NO_LOCK     = "searching"
    S_ACTIVE      = "active"
    S_UNAVAILABLE = "unavailable"

    MARKER_ID   = 7
    MARKER_SIZE = 0.04   # metres — inner black square side (4 cm)
    EMA_ALPHA   = 0.25
    DEAD_XY     = 0.02   # m  — ignore lateral jitter below this
    DEAD_Z      = 0.03   # m  — height dead-zone (3 cm)
    DEAD_YAW    = 5.0    # °
    MIN_STREAK  = 4      # consecutive frames before trusting detection
    HOLD_SECS   = 0.20   # keep last position this long after losing marker

    def __init__(self):
        self._lock         = threading.Lock()
        self._enabled      = True
        self._corr         = (0.0, 0.0, 0.0)   # roll, pitch, yaw corrections
        self._height_enabled = False
        self._target_height  = 1.0              # metres
        self._height_corr    = 0.0              # thrust delta from height PID
        self._current_height: Optional[float] = None
        self.status        = self.S_STARTING
        self._baseline_yaw: Optional[float] = None
        self._proc: Optional[subprocess.Popen] = None
        self._stop         = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._stop.set()
        if self._proc:
            try:
                self._proc.terminate()
            except Exception:
                pass

    def set_enabled(self, v: bool):
        with self._lock:
            self._enabled = v
            if not v:
                self._corr       = (0.0, 0.0, 0.0)
                self._height_corr = 0.0

    def get_corrections(self) -> tuple:
        """Returns (roll, pitch, yaw) corrections — zeros when disabled."""
        with self._lock:
            return self._corr if self._enabled else (0.0, 0.0, 0.0)

    def set_height_enabled(self, v: bool):
        with self._lock:
            self._height_enabled = v
            if not v:
                self._height_corr = 0.0

    def set_target_height(self, h: float):
        with self._lock:
            self._target_height = max(0.10, h)

    def get_height_correction(self) -> float:
        """Thrust delta from height PID — zero when height hold is disabled."""
        with self._lock:
            return self._height_corr if (self._enabled and self._height_enabled) else 0.0

    def get_current_height(self) -> Optional[float]:
        with self._lock:
            return self._current_height

    def _run(self):
        # Optional imports — fails gracefully if not installed
        try:
            import cv2
            import numpy as np
            import win32gui, win32ui, win32con
        except ImportError:
            self.status = self.S_UNAVAILABLE
            return

        if not os.path.exists(_SCRCPY_EXE):
            self.status = self.S_UNAVAILABLE
            return

        WIN = 'HOVER_GUI_CAM'
        try:
            self._proc = subprocess.Popen(
                [_SCRCPY_EXE, '--video-source=camera', '--camera-facing=back',
                 '--no-audio', '--max-fps=30', f'--window-title={WIN}'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            self.status = self.S_UNAVAILABLE
            return

        # Wait up to 10 s for the scrcpy window
        hwnd = None
        for _ in range(20):
            if self._stop.is_set():
                return
            time.sleep(0.5)
            box = []
            def _cb(h, _):
                if win32gui.IsWindowVisible(h) and WIN in win32gui.GetWindowText(h):
                    box.append(h)
            win32gui.EnumWindows(_cb, None)
            if box:
                hwnd = box[0]
                break

        if not hwnd:
            self._proc.terminate()
            self.status = self.S_UNAVAILABLE
            return

        # ArUco detector
        aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        aruco_params = cv2.aruco.DetectorParameters()
        aruco_params.adaptiveThreshWinSizeMin  = 3
        aruco_params.adaptiveThreshWinSizeMax  = 53
        aruco_params.adaptiveThreshWinSizeStep = 4
        aruco_params.minMarkerPerimeterRate    = 0.03
        aruco_params.cornerRefinementMethod    = cv2.aruco.CORNER_REFINE_SUBPIX
        try:
            detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        except AttributeError:
            detector = None   # older OpenCV

        clahe       = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        cam_matrix  = None
        dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        pid_roll   = PID(35.0, 1.0,   8.0,  20.0)
        pid_pitch  = PID(35.0, 1.0,   8.0,  20.0)
        pid_yaw    = PID(15.0, 0.0,   1.5,  80.0)
        # Height-hold PID: error in metres → thrust correction units
        # kp=5000 → 1 m off ≈ ±5000 thrust;  kd damps vertical oscillation
        pid_thrust = PID(5000.0, 200.0, 1500.0, 12000.0)
        kf_height  = KalmanHeight(q_pos=0.005, q_vel=0.01, r_meas=0.08)
        last_kf_t  = time.time()

        smooth_pos    = None
        last_det_t    = 0.0
        detect_streak = 0
        self.status   = self.S_NO_LOCK

        while not self._stop.is_set():
            # Grab frame from scrcpy window
            try:
                l, t, r, b = win32gui.GetWindowRect(hwnd)
                w, h = r - l, b - t
                if w <= 0 or h <= 0:
                    time.sleep(0.02)
                    continue
                hDC = win32gui.GetWindowDC(hwnd)
                mDC = win32ui.CreateDCFromHandle(hDC)
                sDC = mDC.CreateCompatibleDC()
                bmp = win32ui.CreateBitmap()
                bmp.CreateCompatibleBitmap(mDC, w, h)
                sDC.SelectObject(bmp)
                sDC.BitBlt((0, 0), (w, h), mDC, (0, 0), win32con.SRCCOPY)
                raw = bmp.GetBitmapBits(True)
                win32gui.DeleteObject(bmp.GetHandle())
                sDC.DeleteDC(); mDC.DeleteDC()
                win32gui.ReleaseDC(hwnd, hDC)
                img   = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 4)
                frame = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGRA2BGR),
                                   (640, 480))
            except Exception:
                time.sleep(0.02)
                continue

            fh, fw = frame.shape[:2]
            if cam_matrix is None:
                cam_matrix = np.array([[fw, 0, fw / 2],
                                       [0, fw, fh / 2],
                                       [0,  0,       1]], dtype=np.float32)

            gray = clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            try:
                if detector:
                    corners, ids, _ = detector.detectMarkers(gray)
                else:
                    corners, ids, _ = cv2.aruco.detectMarkers(
                        gray, aruco_dict, parameters=aruco_params)
            except Exception:
                ids = None

            detected = None
            if ids is not None:
                valid = [i for i, mid in enumerate(ids.flatten())
                         if mid == self.MARKER_ID]
                if valid:
                    c    = corners[valid[0]][0]
                    diag = math.hypot(c[2][0] - c[0][0], c[2][1] - c[0][1])
                    if diag >= 20:
                        half    = self.MARKER_SIZE / 2.0
                        obj_pts = np.array(
                            [[-half, half, 0], [half, half, 0],
                             [half, -half, 0], [-half, -half, 0]],
                            dtype=np.float32)
                        _, _, tvec = cv2.solvePnP(
                            obj_pts, c, cam_matrix, dist_coeffs,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE)
                        tv  = tvec.flatten()
                        pix = (float(np.mean(c[:, 0])),
                               float(np.mean(c[:, 1])))
                        sx  = (pix[0] - fw / 2) / fw * tv[2]
                        sy  = (pix[1] - fh / 2) / fw * tv[2]
                        yaw_raw = math.degrees(
                            math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0]))
                        raw = (sx, sy, float(tv[2]), yaw_raw)

                        if smooth_pos is None:
                            smooth_pos = raw
                        else:
                            diff = (yaw_raw - smooth_pos[3] + 180) % 360 - 180
                            if abs(diff) > 45:
                                diff = 0
                            raw = (raw[0], raw[1], raw[2], smooth_pos[3] + diff)
                            smooth_pos = tuple(
                                self.EMA_ALPHA * r + (1 - self.EMA_ALPHA) * s
                                for r, s in zip(raw, smooth_pos))

                        last_det_t    = time.time()
                        detect_streak = min(detect_streak + 1, self.MIN_STREAK)
                        if detect_streak >= self.MIN_STREAK:
                            detected = smooth_pos

            if ids is None:
                if smooth_pos and (time.time() - last_det_t) < self.HOLD_SECS:
                    detected = (smooth_pos
                                if detect_streak >= self.MIN_STREAK else None)
                else:
                    detect_streak = 0
                    smooth_pos    = None

            if detected:
                x_m, y_m, z_m, yaw_raw = detected
                if self._baseline_yaw is None:
                    self._baseline_yaw = yaw_raw
                    pid_roll.reset(); pid_pitch.reset()
                    pid_yaw.reset();  pid_thrust.reset()

                # ── Lateral + yaw corrections ─────────────────────────────
                baseline: float = self._baseline_yaw  # type: ignore[assignment]
                dx   = x_m if abs(x_m) > self.DEAD_XY else 0.0
                dy   = y_m if abs(y_m) > self.DEAD_XY else 0.0
                tilt = float((yaw_raw - baseline + 180) % 360 - 180)
                tilt = float(tilt if abs(tilt) > self.DEAD_YAW else 0.0)

                # ── Height: Kalman-filter Z then run PID ──────────────────
                now_kf      = time.time()
                h_filtered  = kf_height.step(z_m, now_kf - last_kf_t)
                last_kf_t   = now_kf
                with self._lock:
                    target_h = self._target_height
                    h_en     = self._height_enabled
                h_err        = target_h - h_filtered
                h_corr       = pid_thrust.compute(h_err) if h_en else 0.0
                if h_en and abs(h_err) < self.DEAD_Z:
                    h_corr = 0.0

                with self._lock:
                    self._corr           = (pid_roll.compute(-dx),
                                            pid_pitch.compute(dy),
                                            pid_yaw.compute(-tilt))
                    self._height_corr    = h_corr
                    self._current_height = h_filtered
                self.status = self.S_ACTIVE
            else:
                with self._lock:
                    self._corr           = (0.0, 0.0, 0.0)
                    self._height_corr    = 0.0
                    self._current_height = None
                if self.status == self.S_ACTIVE:
                    self.status        = self.S_NO_LOCK
                    self._baseline_yaw = None

        self._proc.terminate()


# ── UDP sender thread ─────────────────────────────────────────────────────────
class DroneLink(threading.Thread):
    def __init__(self, cam_stab: CameraStabilizer = None):
        super().__init__(daemon=True)
        self.sock       = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock      = threading.Lock()
        self._armed     = False
        self._emergency = False
        self._thrust    = cfg.HOVER_THRUST
        self._roll      = cfg.TRIM_ROLL
        self._pitch     = cfg.TRIM_PITCH
        self._yaw       = cfg.TRIM_YAW
        self._running   = True
        # Transient key offsets (return to 0 on key release)
        self._key_r     = 0.0
        self._key_p     = 0.0
        self._key_y     = 0.0
        # Optional gyro smooth filter (higher value = more smoothing)
        self._stabilize = False
        self._cam_stab  = cam_stab

    def _send(self, roll, pitch, yaw, thrust):
        try:
            self.sock.sendto(make_rpyt(roll, pitch, yaw, thrust),
                             (cfg.DRONE_IP, cfg.DRONE_PORT))
        except OSError:
            pass

    def set_params(self, thrust, roll, pitch, yaw):
        with self._lock:
            self._thrust = thrust
            self._roll   = roll
            self._pitch  = pitch
            self._yaw    = yaw

    def set_key_offsets(self, roll: float, pitch: float, yaw: float):
        """Transient attitude offsets from held keys — snap to 0 on release."""
        with self._lock:
            self._key_r = roll
            self._key_p = pitch
            self._key_y = yaw

    def set_stabilize(self, enabled: bool):
        """Gyro smooth filter: higher SMOOTH suppresses vibration/jitter."""
        with self._lock:
            self._stabilize = enabled

    def arm(self):
        threading.Thread(target=self._ramp_arm, daemon=True).start()

    def disarm(self):
        threading.Thread(target=self._ramp_disarm, daemon=True).start()

    def emergency(self):
        with self._lock:
            self._emergency = True
            self._armed     = False

    def clear_emergency(self):
        with self._lock:
            self._emergency = False

    @property
    def is_armed(self):
        with self._lock:
            return self._armed

    @property
    def is_emergency(self):
        with self._lock:
            return self._emergency

    def _ramp_arm(self):
        self._send(0, 0, 0, 0)
        time.sleep(cfg.UNLOCK_DURATION)
        steps = max(10, int(cfg.THRUST_RAMP_TIME * cfg.SEND_HZ))
        with self._lock:
            target       = max(cfg.THRUST_MIN, self._thrust)  # honour slider
            self._thrust = 0
            self._armed  = True
        for i in range(steps):
            with self._lock:
                self._thrust = int(target * (i + 1) / steps)
            time.sleep(cfg.THRUST_RAMP_TIME / steps)
        with self._lock:
            self._thrust = target

    def _ramp_disarm(self):
        with self._lock:
            from_t = self._thrust
        delay = cfg.DESCENT_SECS / cfg.DESCENT_STEPS
        for i in range(cfg.DESCENT_STEPS):
            with self._lock:
                self._thrust = max(
                    0, int(from_t * (1 - (i + 1) / cfg.DESCENT_STEPS)))
            time.sleep(delay)
        with self._lock:
            self._thrust = 0
            self._armed  = False

    def run(self):
        interval = 1.0 / cfg.SEND_HZ
        cur_r, cur_p, cur_y, cur_t = 0.0, 0.0, 0.0, 0.0
        while self._running:
            cam_r = cam_p = cam_y = 0.0
            h_corr = 0.0
            if self._cam_stab:
                cam_r, cam_p, cam_y = self._cam_stab.get_corrections()
                h_corr = self._cam_stab.get_height_correction()

            with self._lock:
                armed  = self._armed
                emerg  = self._emergency
                base_t = float(self._thrust)
                # Permanent trim  +  key offset  +  camera correction
                tgt_r  = self._roll  + self._key_r + cam_r
                tgt_p  = self._pitch + self._key_p + cam_p
                tgt_y  = self._yaw   + self._key_y + cam_y
                smooth = 0.25 if self._stabilize else 0.08

            # Height PID adds to base thrust; clamp to safe range
            tgt_t = float(max(cfg.THRUST_MIN,
                              min(cfg.THRUST_MAX, base_t + h_corr)))

            cur_r += (tgt_r - cur_r) * smooth
            cur_p += (tgt_p - cur_p) * smooth
            cur_y += (tgt_y - cur_y) * smooth
            cur_t += (tgt_t - cur_t) * 0.15   # smooth thrust changes

            if armed and not emerg:
                self._send(cur_r, cur_p, cur_y, int(cur_t))
            else:
                self._send(0, 0, 0, 0)
            time.sleep(interval)

    def stop(self):
        self._running = False
        try:
            self._send(0, 0, 0, 0)
            self.sock.close()
        except Exception:
            pass


# ── Vertical slider with label + spinbox ─────────────────────────────────────
class ParamSlider(QWidget):
    def __init__(self, title, min_v, max_v, default, scale=1.0,
                 unit='', decimals=0, color='#00bcd4', desc='', step=None):
        super().__init__()
        self.scale    = scale
        self.decimals = decimals
        self.unit     = unit

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)

        lbl = QLabel(title.upper())
        lbl.setFont(QFont('Segoe UI', 9, QFont.Bold))
        lbl.setStyleSheet('color:#666; letter-spacing:1px;')
        lbl.setAlignment(Qt.AlignCenter)

        desc_lbl = QLabel(desc)
        desc_lbl.setFont(QFont('Segoe UI', 8))
        desc_lbl.setStyleSheet('color:#3a3a5a; font-style:italic;')
        desc_lbl.setAlignment(Qt.AlignCenter)
        desc_lbl.setWordWrap(True)

        self.val_lbl = QLabel()
        self.val_lbl.setFont(QFont('Segoe UI', 26, QFont.Bold))
        self.val_lbl.setAlignment(Qt.AlignCenter)
        self.val_lbl.setStyleSheet(f'color:{color};')

        self.slider = QSlider(Qt.Vertical)
        self.slider.setRange(min_v, max_v)
        self.slider.setValue(default)
        self.slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        if step is not None:
            self.slider.setSingleStep(step)
            self.slider.setPageStep(step * 10)
            self.slider.setTickInterval(step * 10)
            self.slider.setTickPosition(QSlider.TicksRight)
        self.slider.setStyleSheet(f"""
            QSlider::groove:vertical {{
                background:#1e1e2e; width:10px; border-radius:5px;
            }}
            QSlider::handle:vertical {{
                background:{color}; height:24px; width:24px;
                margin:0 -7px; border-radius:12px;
            }}
            QSlider::sub-page:vertical {{
                background:{color}; border-radius:5px;
            }}
        """)
        self.slider.valueChanged.connect(self._on_slider)

        spin_step = (step * scale) if step is not None else (1000.0 if decimals == 0 else scale)
        self.spinbox = QDoubleSpinBox()
        self.spinbox.setRange(min_v * scale, max_v * scale)
        self.spinbox.setSingleStep(spin_step)
        self.spinbox.setDecimals(decimals)
        self.spinbox.setValue(default * scale)
        self.spinbox.setStyleSheet(
            'background:#1e1e2e; color:#ddd; border:1px solid #333; '
            'border-radius:4px; padding:2px;')
        self.spinbox.setFont(QFont('Segoe UI', 9))
        self.spinbox.valueChanged.connect(self._on_spinbox)

        lay.addWidget(lbl)
        lay.addWidget(desc_lbl)
        lay.addWidget(self.val_lbl)
        lay.addWidget(self.slider, stretch=1)
        lay.addWidget(self.spinbox)
        self._update_label(default)

    def _update_label(self, raw):
        v   = raw * self.scale
        txt = f"{int(v)}" if self.decimals == 0 else f"{v:+.{self.decimals}f}"
        self.val_lbl.setText(txt + self.unit)

    def _on_slider(self, raw):
        self._update_label(raw)
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(raw * self.scale)
        self.spinbox.blockSignals(False)

    def _on_spinbox(self, val):
        raw = int(round(val / self.scale))
        self.slider.blockSignals(True)
        self.slider.setValue(raw)
        self.slider.blockSignals(False)
        self._update_label(raw)

    def value(self):
        return self.slider.value() * self.scale

    def set_value(self, real):
        self.slider.setValue(int(real / self.scale))  # _on_slider syncs spinbox


# ── Main window ───────────────────────────────────────────────────────────────
class TunerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.cam_stab = CameraStabilizer()
        self.cam_stab.start()

        self.drone = DroneLink(cam_stab=self.cam_stab)
        self.drone.start()

        self.setWindowTitle("Hover Tuner — drone_direct")
        self.setMinimumSize(860, 580)
        self._build_ui()

        self._poll = QTimer()
        self._poll.timeout.connect(self._refresh)
        self._poll.start(120)

        self._cfg_mtime = self._get_mtime()
        self._hot = QTimer()
        self._hot.timeout.connect(self._hot_reload)
        self._hot.start(500)

        self._pressed   = set()
        self._key_timer = QTimer()
        self._key_timer.timeout.connect(self._apply_keys)
        self._key_timer.start(20)   # 50 Hz key polling

    def _build_ui(self):
        root = QWidget()
        root.setStyleSheet("""
            QWidget { background:#111120; color:#ddd;
                      font-family:'Segoe UI',sans-serif; }
            QGroupBox { border:1px solid #2a2a3e; border-radius:10px;
                        margin-top:12px; padding:8px;
                        font-size:10px; font-weight:bold;
                        color:#444; letter-spacing:1px; }
            QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 4px; }
        """)
        self.setCentralWidget(root)
        v = QVBoxLayout(root)
        v.setContentsMargins(16, 12, 16, 12)
        v.setSpacing(10)

        h = QLabel("Hover Tuner")
        h.setFont(QFont('Segoe UI', 18, QFont.Bold))
        h.setStyleSheet('color:#fff;')
        sub = QLabel(
            f"{cfg.DRONE_IP}:{cfg.DRONE_PORT}  ·  "
            "ARM  →  tune Throttle until drone lifts  →  fix hardware drift "
            "with Trim sliders  →  Save")
        sub.setStyleSheet('color:#444; font-size:10px;')
        v.addWidget(h)
        v.addWidget(sub)

        # ── Sliders — Mode 2 RC layout ────────────────────────────────────
        sticks = QHBoxLayout()
        sticks.setSpacing(12)

        # Left stick: Throttle (vertical axis) + Yaw trim (horizontal axis)
        lg = QGroupBox("LEFT STICK  —  W/S = throttle   A/D = yaw")
        ll = QHBoxLayout(lg)
        ll.setSpacing(16)
        self.sl_thrust = ParamSlider(
            "Throttle", cfg.THRUST_MIN, cfg.THRUST_MAX, cfg.HOVER_THRUST,
            color='#ff6b6b', step=100,
            desc="Base motor power. W = up, S = down while flying."
        )
        self.sl_yaw = ParamSlider(
            "Yaw trim", -30000, 30000, int(cfg.TRIM_YAW * 1000),
            scale=0.001, unit='°/s', decimals=4, color='#845ef7',
            desc="Permanent spin correction. A/D keys = yaw while flying."
        )
        for s in (self.sl_thrust, self.sl_yaw):
            s.slider.valueChanged.connect(self._push)
            s.spinbox.valueChanged.connect(self._push)
            ll.addWidget(s)

        # Right stick: Pitch (vertical axis) + Roll (horizontal axis)
        rg = QGroupBox("RIGHT STICK  —  Up/Down = pitch   Left/Right = roll")
        rl = QHBoxLayout(rg)
        rl.setSpacing(16)
        self.sl_pitch = ParamSlider(
            "Pitch trim", -20000, 20000, int(cfg.TRIM_PITCH * 1000),
            scale=0.001, unit='°', decimals=4, color='#6bcb77',
            desc="Permanent fwd/back correction. Arrow Up/Down = pitch while flying."
        )
        self.sl_roll = ParamSlider(
            "Roll trim", -20000, 20000, int(cfg.TRIM_ROLL * 1000),
            scale=0.001, unit='°', decimals=4, color='#ffd93d',
            desc="Permanent left/right correction. Arrow Left/Right = roll while flying."
        )
        for s in (self.sl_pitch, self.sl_roll):
            s.slider.valueChanged.connect(self._push)
            s.spinbox.valueChanged.connect(self._push)
            rl.addWidget(s)

        sticks.addWidget(lg, stretch=1)
        sticks.addWidget(rg, stretch=1)
        v.addLayout(sticks, stretch=1)

        # ── Controls ──────────────────────────────────────────────────────
        bg = QGroupBox("Controls")
        bl = QHBoxLayout(bg)
        bl.setSpacing(10)

        self.btn_arm  = self._btn("ARM",         '#22aa55', self._arm_toggle, h=50)
        self.btn_emrg = self._btn("EMERGENCY",   '#dd3333', self._emergency,  h=50)
        self.btn_rst  = self._btn("Reset Trims", '#2a2a3e', self._reset,      h=50)
        self.btn_save = self._btn("Save config", '#1a3a5c', self._save,       h=50)

        self.chk_gyro = QCheckBox("Gyro smooth")
        self.chk_gyro.setStyleSheet('color:#ddd; font-size:11px; padding:6px;')
        self.chk_gyro.setToolTip(
            "Increases attitude smoothing filter to suppress vibration\n"
            "not caused by your inputs. Reduces oscillation at slight lag cost.")
        self.chk_gyro.stateChanged.connect(
            lambda s: self.drone.set_stabilize(bool(s)))

        # Camera stabilise — on by default, updates label in _refresh()
        self.chk_cam = QCheckBox("Camera stabilize")
        self.chk_cam.setChecked(True)
        self.chk_cam.setStyleSheet('color:#888; font-size:11px; padding:6px;')
        self.chk_cam.setToolTip(
            "Uses phone camera + ArUco marker (ID 7) for lateral position hold.\n"
            "Requires phone connected via USB with USB Debugging enabled.\n"
            "Falls back gracefully to key-only control if unavailable.")
        self.chk_cam.stateChanged.connect(
            lambda s: self.cam_stab.set_enabled(bool(s)))

        for w in (self.btn_arm, self.btn_emrg, self.btn_rst,
                  self.btn_save, self.chk_gyro, self.chk_cam):
            bl.addWidget(w)
        v.addWidget(bg)

        # ── Height Hold ───────────────────────────────────────────────────
        hg = QGroupBox("HEIGHT HOLD  —  ArUco Z from camera (requires camera lock)")
        hl = QHBoxLayout(hg)
        hl.setSpacing(12)

        self.chk_height = QCheckBox("Enable")
        self.chk_height.setStyleSheet('color:#4fc3f7; font-size:11px; padding:6px;')
        self.chk_height.setToolTip(
            "PID controller keeps drone at the target height using the\n"
            "ArUco marker Z distance. Kalman filter suppresses camera jitter.\n"
            "Requires camera lock (marker visible).")
        self.chk_height.stateChanged.connect(
            lambda s: self.cam_stab.set_height_enabled(bool(s)))

        # Target height 0.20 m – 3.00 m, 1 cm resolution
        self.sl_height = ParamSlider(
            "Target Height", 20, 300, 100,
            scale=0.01, unit=' m', decimals=4, color='#4fc3f7',
            desc="Hold the drone at this height above the camera."
        )
        self.sl_height.slider.valueChanged.connect(
            lambda raw: self.cam_stab.set_target_height(raw * 0.01))
        self.sl_height.spinbox.valueChanged.connect(
            lambda val: self.cam_stab.set_target_height(val))

        self.lbl_height = QLabel("Height: --")
        self.lbl_height.setFont(QFont('Segoe UI', 11, QFont.Bold))
        self.lbl_height.setStyleSheet('color:#4fc3f7; padding:4px; min-width:130px;')

        hl.addWidget(self.chk_height)
        hl.addWidget(self.sl_height, stretch=1)
        hl.addWidget(self.lbl_height)
        v.addWidget(hg)

        self.status = QLabel("DISARMED")
        self.status.setFont(QFont('Segoe UI', 12, QFont.Bold))
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setFixedHeight(34)
        self.status.setStyleSheet(
            'background:#1a1a2e; border-radius:8px; color:#ffd93d; padding:4px;')
        v.addWidget(self.status)

    def _btn(self, text, color, slot, h=44):
        b = QPushButton(text)
        b.setFixedHeight(h)
        b.setFont(QFont('Segoe UI', 11, QFont.Bold))
        b.setStyleSheet(f'background:{color}; color:#fff; border-radius:8px;')
        b.clicked.connect(slot)
        return b

    def _push(self):
        self.drone.set_params(int(self.sl_thrust.value()),
                               self.sl_roll.value(),
                               self.sl_pitch.value(),
                               self.sl_yaw.value())

    def _arm_toggle(self):
        if self.drone.is_emergency:
            return
        self.drone.disarm() if self.drone.is_armed else self.drone.arm()

    def _emergency(self):
        if self.drone.is_emergency:
            self.drone.clear_emergency()
            self.btn_emrg.setText("EMERGENCY")
            self.btn_emrg.setStyleSheet(
                'background:#dd3333; color:#fff; border-radius:8px;')
        else:
            self.drone.emergency()
            self.btn_emrg.setText("Clear Emergency")
            self.btn_emrg.setStyleSheet(
                'background:#555; color:#aaa; border-radius:8px;')

    def _reset(self):
        self.sl_roll.set_value(0.0)
        self.sl_pitch.set_value(0.0)
        self.sl_yaw.set_value(0.0)
        self._push()

    # ── Keyboard — Mode 2 RC ──────────────────────────────────────────────
    def keyPressEvent(self, event):
        if not event.isAutoRepeat():
            self._pressed.add(event.key())

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            self._pressed.discard(event.key())

    def _apply_keys(self):
        # Don't steal keys while user is typing in a spinbox
        if isinstance(QApplication.focusWidget(), QDoubleSpinBox):
            self.drone.set_key_offsets(0.0, 0.0, 0.0)
            return
        if not self._pressed:
            self.drone.set_key_offsets(0.0, 0.0, 0.0)
            return

        # Mode 2 RC:
        #   Left stick  (WASD)  —  W/S = throttle,    A/D = yaw
        #   Right stick (arrows)—  Up/Down = pitch,  Left/Right = roll
        STEP_ATT    = 5.0    # degrees per 20 ms tick
        STEP_YAW    = 15.0   # deg/s per tick
        STEP_THRUST = 100    # thrust units per tick

        r = p = y = 0.0
        dt = 0

        for k in self._pressed:
            if k == Qt.Key_Left:  r -= STEP_ATT
            if k == Qt.Key_Right: r += STEP_ATT
            if k == Qt.Key_Up:    p -= STEP_ATT   # forward = nose down
            if k == Qt.Key_Down:  p += STEP_ATT
            if k == Qt.Key_A:     y -= STEP_YAW
            if k == Qt.Key_D:     y += STEP_YAW
            if k == Qt.Key_W:     dt += STEP_THRUST
            if k == Qt.Key_S:     dt -= STEP_THRUST

        if dt:
            new_t = max(cfg.THRUST_MIN,
                        min(cfg.THRUST_MAX, int(self.sl_thrust.value()) + dt))
            self.sl_thrust.set_value(new_t)
            self._push()

        self.drone.set_key_offsets(r, p, y)

    # ── Save / reload ─────────────────────────────────────────────────────
    def _save(self):
        import re
        path = os.path.join(os.path.dirname(__file__), 'config.py')
        with open(path, encoding='utf-8') as f:
            text = f.read()
        vals = {
            'HOVER_THRUST': int(self.sl_thrust.value()),
            'TRIM_ROLL':    round(self.sl_roll.value(),  4),
            'TRIM_PITCH':   round(self.sl_pitch.value(), 4),
            'TRIM_YAW':     round(self.sl_yaw.value(),   4),
        }
        for key, val in vals.items():
            text = re.sub(rf'^({re.escape(key)}\s*=\s*)\S+',
                          rf'\g<1>{val}', text, flags=re.MULTILINE)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        self.status.setText(
            f"Saved — Thrust:{vals['HOVER_THRUST']}  "
            f"R:{vals['TRIM_ROLL']:+.4f}  "
            f"P:{vals['TRIM_PITCH']:+.4f}  "
            f"Y:{vals['TRIM_YAW']:+.4f}")

    def _get_mtime(self):
        try:
            return os.path.getmtime(
                os.path.join(os.path.dirname(__file__), 'config.py'))
        except Exception:
            return 0

    def _hot_reload(self):
        mt = self._get_mtime()
        if mt <= self._cfg_mtime:
            return
        self._cfg_mtime = mt
        try:
            import importlib, config as _c
            importlib.reload(_c)
            self.sl_thrust.set_value(_c.HOVER_THRUST)
            self.sl_roll.set_value(_c.TRIM_ROLL)
            self.sl_pitch.set_value(_c.TRIM_PITCH)
            self.sl_yaw.set_value(_c.TRIM_YAW)
            self._push()
            self.status.setText("config.py reloaded")
        except Exception as e:
            self.status.setText(f"Reload error: {e}")

    def _refresh(self):
        armed = self.drone.is_armed
        emerg = self.drone.is_emergency
        thr   = int(self.sl_thrust.value())

        # Live height readout
        h = self.cam_stab.get_current_height()
        self.lbl_height.setText(
            f"Height: {h:.4f} m" if h is not None else "Height: --")

        # Update camera stabilise checkbox label/colour with live status
        cs = self.cam_stab.status
        cam_label = {
            CameraStabilizer.S_STARTING:    'Camera stabilize (starting...)',
            CameraStabilizer.S_NO_LOCK:     'Camera stabilize (searching for marker)',
            CameraStabilizer.S_ACTIVE:      'Camera stabilize (locked)',
            CameraStabilizer.S_UNAVAILABLE: 'Camera stabilize (unavailable)',
        }.get(cs, 'Camera stabilize')
        cam_color = {
            CameraStabilizer.S_STARTING:    '#888',
            CameraStabilizer.S_NO_LOCK:     '#ffd93d',
            CameraStabilizer.S_ACTIVE:      '#6bcb77',
            CameraStabilizer.S_UNAVAILABLE: '#555',
        }.get(cs, '#888')
        self.chk_cam.setText(cam_label)
        self.chk_cam.setStyleSheet(
            f'color:{cam_color}; font-size:11px; padding:6px;')

        if emerg:
            self.status.setText("EMERGENCY — motors cut")
            self.status.setStyleSheet(
                'background:#3a1111; border-radius:8px; color:#ff6b6b; padding:4px;')
            self.btn_arm.setStyleSheet(
                'background:#2a2a2a; color:#444; border-radius:8px;')
        elif armed:
            self.status.setText(f"ARMED   Throttle: {thr}")
            self.status.setStyleSheet(
                'background:#0d2b1a; border-radius:8px; color:#6bcb77; padding:4px;')
            self.btn_arm.setText("DISARM")
            self.btn_arm.setStyleSheet(
                'background:#cc3333; color:white; border-radius:8px;')
        else:
            self.status.setText("DISARMED  —  press ARM to begin")
            self.status.setStyleSheet(
                'background:#1a1a2e; border-radius:8px; color:#ffd93d; padding:4px;')
            self.btn_arm.setText("ARM")
            self.btn_arm.setStyleSheet(
                'background:#22aa55; color:white; border-radius:8px;')

    def closeEvent(self, event):
        if self.drone.is_armed:
            self.drone.disarm()
            time.sleep(cfg.DESCENT_SECS + 0.3)
        self.drone.stop()
        self.cam_stab.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = TunerWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
