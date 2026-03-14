"""
drone_direct/balance.py
========================
Full autonomous hover — runs entirely on the PC.

Architecture:
  Phone camera ──(scrcpy --video-source=camera)──> USB/ADB ──> PC (PyAV decode)
  PC           ──(ArUco detection + PIDs)────────> UDP ───────> Drone WiFi

Setup:
  1. Connect phone via USB cable
  2. Enable USB Debugging on the phone (Settings → Developer Options)
  3. Connect PC WiFi to drone hotspot
  4. Run:  python balance.py
     (scrcpy starts automatically in the background)

Controls  (click the camera preview window first):
  SPACE   — Arm / Disarm
  ESC     — Emergency stop (toggle)
  ↑ / ↓   — Nudge base thrust ±500
  H       — Toggle hover-only mode (altitude hold, no lateral PID)
  Ctrl+C  — Quit

Config is read from  drone_direct/tuner/config.py  so your tuned thrust
and trim values carry over automatically.
"""

import sys
import os
import subprocess
import socket
import struct
import threading
import time
import math

import cv2
import numpy as np
import win32gui
import win32ui
import win32con
import keyboard
# ── Load tuner config ─────────────────────────────────────────────────────────
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'tuner'))
import config as cfg

# ── Network ───────────────────────────────────────────────────────────────────
DRONE_IP   = cfg.DRONE_IP
DRONE_PORT = cfg.DRONE_PORT

# ── Scrcpy camera source ──────────────────────────────────────────────────────
_SCRCPY_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'scrcpy',
                 'scrcpy-win64-v3.0.2'))
SCRCPY_EXE     = os.path.join(_SCRCPY_DIR, 'scrcpy.exe')
CAM_WIDTH      = 640    # resize frames to this size for faster processing
CAM_HEIGHT     = 480
CAM_FACING     = 'back' # 'back' or 'front'


# ── ArUco marker ──────────────────────────────────────────────────────────────
MARKER_SIZE_M   = 0.08        # physical marker side length in metres
DRONE_MARKER_ID = 7           # which ArUco ID is on the drone

# ── PID gains ─────────────────────────────────────────────────────────────────
# These are the starting values — tune in the GUI or edit here.
# Roll/Pitch: output = degrees of tilt angle command
# Yaw:        output = degrees/s rotation command
# Elevation:  output = thrust delta added to base_thrust

PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD  = 35.0,  1.0,   8.0
PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD = 35.0,  1.0,   8.0
PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD  = 15.0,  0.0,   1.5
PID_ELEV_KP,  PID_ELEV_KI,  PID_ELEV_KD = 12000, 300.0, 3500.0

# PID output limits
ROLL_LIMIT  = 20.0
PITCH_LIMIT = 20.0
YAW_LIMIT   = 80.0
ELEV_LIMIT  = 17000.0

# Dead-zones — ignore tiny errors to avoid jitter
DEAD_XY   = 0.02    # metres
DEAD_YAW  = 5.0     # degrees
DEAD_ELEV = 0.03    # relative error (fraction of target Z)

# ── Takeoff / climb parameters ────────────────────────────────────────────────
BLIND_CLIMB_START = 35000   # thrust before first ArUco lock
BLIND_CLIMB_MAX   = 62000
BLIND_CLIMB_RATE  = 400     # per frame (roughly 30 fps)
TAKEOFF_RAMP_TO   = 55000   # thrust at first ArUco lock
TAKEOFF_SECS      = 1.5

# Adaptive hover — slowly corrects base_thrust drift
ADAPT_ALPHA  = 0.005
ADAPT_THRESH = 1500
ADAPT_MAX    = 3000

# ── Detection smoothing ───────────────────────────────────────────────────────
EMA_ALPHA  = 0.25   # smoothing factor (0=ignore new, 1=no smoothing)
HOLD_SECS  = 0.20   # keep last position this long after losing marker
MIN_STREAK = 4      # frames of consecutive detect before trusting position


# ─────────────────────────────────────────────────────────────────────────────
# PID controller
# ─────────────────────────────────────────────────────────────────────────────
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


# ─────────────────────────────────────────────────────────────────────────────
# CRTP packet
# ─────────────────────────────────────────────────────────────────────────────
def _cksum(b: bytes) -> int:
    return sum(b) & 0xFF

def make_rpyt(roll, pitch, yaw, thrust) -> bytes:
    header  = (3 << 4) | 0
    t       = max(0, min(65535, int(thrust)))
    payload = struct.pack('<BfffH', header,
                          float(roll), float(pitch), float(yaw), t)
    return payload + bytes([_cksum(payload)])


# ─────────────────────────────────────────────────────────────────────────────
# Shared state
# ─────────────────────────────────────────────────────────────────────────────
_lock  = threading.Lock()
_state = {
    'armed':        False,
    'emergency':    False,
    'hover_only':   False,
    'thrust_delta': 0,      # cumulative nudge from keyboard
    'quit':         False,
}
_flight = {
    'base_thrust':  BLIND_CLIMB_START,
    'ramp_target':  None,
    'baseline':     False,
    'z_target':     -1.0,
    'baseline_yaw': None,
}


# ─────────────────────────────────────────────────────────────────────────────
# UDP socket
# ─────────────────────────────────────────────────────────────────────────────
_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_rpyt(roll, pitch, yaw, thrust):
    try:
        _sock.sendto(make_rpyt(roll, pitch, yaw, thrust), (DRONE_IP, DRONE_PORT))
    except Exception:
        pass


# ─────────────────────────────────────────────────────────────────────────────
# Safe descent (called in its own thread on disarm)
# ─────────────────────────────────────────────────────────────────────────────
def safe_descend(from_thrust):
    steps = cfg.DESCENT_STEPS
    delay = cfg.DESCENT_SECS / steps
    print(f"\n[DISARM] Safe descent from {from_thrust} …")
    for i in range(steps):
        t = (i + 1) / steps
        send_rpyt(0, 0, 0, max(0, int(from_thrust * (1.0 - t))))
        time.sleep(delay)
    send_rpyt(0, 0, 0, 0)
    print("[DISARM] Landed.")


# ─────────────────────────────────────────────────────────────────────────────
# Camera + ArUco + PID loop  (main control thread)
# ─────────────────────────────────────────────────────────────────────────────
SCRCPY_WIN_TITLE = 'BALANCE_CAM'

def _open_scrcpy_camera():
    cmd = [
        SCRCPY_EXE,
        '--video-source=camera',
        f'--camera-facing={CAM_FACING}',
        '--no-audio',
        '--max-fps=30',
        f'--window-title={SCRCPY_WIN_TITLE}',
    ]
    print(f"[CAM] Starting scrcpy camera ({CAM_FACING}) …")
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
    hwnd = None
    for _ in range(30):
        time.sleep(0.5)
        def _cb(h, _):
            if win32gui.IsWindowVisible(h) and SCRCPY_WIN_TITLE in win32gui.GetWindowText(h):
                hwnd_box.append(h)
        hwnd_box = []
        win32gui.EnumWindows(_cb, None)
        if hwnd_box:
            hwnd = hwnd_box[0]
            break
    if not hwnd:
        proc.terminate()
        raise RuntimeError("scrcpy window did not appear — check USB & tap Allow on phone")
    print(f"[CAM] Window found (hwnd={hwnd})")
    return proc, hwnd


def _grab_frame(hwnd):
    try:
        l, t, r, b = win32gui.GetWindowRect(hwnd)
        w, h = r - l, b - t
        if w <= 0 or h <= 0:
            return None
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
        img = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 4)
        return cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGRA2BGR),
                          (CAM_WIDTH, CAM_HEIGHT))
    except Exception:
        return None


def control_loop():
    global _adapt_avg
    _adapt_avg = 0.0

    # PID controllers
    pid_roll  = PID(PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  ROLL_LIMIT)
    pid_pitch = PID(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PITCH_LIMIT)
    pid_yaw   = PID(PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   YAW_LIMIT)
    pid_elev  = PID(PID_ELEV_KP,  PID_ELEV_KI,  PID_ELEV_KD,  ELEV_LIMIT)

    def reset_pids():
        for p in (pid_roll, pid_pitch, pid_yaw, pid_elev):
            p.reset()

    # ArUco setup
    aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_params.adaptiveThreshWinSizeMin  = 3
    aruco_params.adaptiveThreshWinSizeMax  = 53
    aruco_params.adaptiveThreshWinSizeStep = 4
    aruco_params.minMarkerPerimeterRate    = 0.03
    aruco_params.cornerRefinementMethod    = cv2.aruco.CORNER_REFINE_SUBPIX
    aruco_params.errorCorrectionRate       = 0.6
    _clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    try:
        _detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        _detector = None

    cam_matrix   = None
    dist_coeffs  = np.zeros((4, 1), dtype=np.float32)

    # Detection smoothing state
    smooth_pos    = None
    last_det_t    = 0.0
    detect_streak = 0

    # ── Start scrcpy camera ───────────────────────────────────────────────
    try:
        scrcpy_proc, cam_hwnd = _open_scrcpy_camera()
        print("[CAM] Control loop starting …\n")
    except Exception as e:
        print(f"[CAM] ERROR: {e}")
        return

    prev_armed = False

    while True:
        with _lock:
            if _state['quit']:
                break

        frame = _grab_frame(cam_hwnd)
        if frame is None:
            time.sleep(0.02)
            continue

        fh, fw = frame.shape[:2]

        # Build camera matrix from frame size (approximate)
        if cam_matrix is None:
            cam_matrix = np.array([[fw, 0, fw/2],
                                   [0, fw, fh/2],
                                   [0,  0,    1]], dtype=np.float32)

        # ── ArUco detection ────────────────────────────────────────────────
        gray = _clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        try:
            if _detector:
                corners, ids, _ = _detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, aruco_dict, parameters=aruco_params)
        except Exception:
            ids = None

        detected = None
        if ids is not None:
            valid = [i for i, mid in enumerate(ids.flatten())
                     if mid == DRONE_MARKER_ID]
            if valid:
                c    = corners[valid[0]][0]
                diag = math.hypot(c[2][0]-c[0][0], c[2][1]-c[0][1])
                if diag >= 20:
                    half    = MARKER_SIZE_M / 2.0
                    obj_pts = np.array([[-half, half, 0], [half, half, 0],
                                        [half, -half, 0], [-half, -half, 0]],
                                       dtype=np.float32)
                    _, _, tvec = cv2.solvePnP(
                        obj_pts, c, cam_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    tv  = tvec.flatten()
                    pix = (float(np.mean(c[:, 0])), float(np.mean(c[:, 1])))
                    sx  = (pix[0] - fw/2) / fw * tv[2]
                    sy  = (pix[1] - fh/2) / fw * tv[2]
                    yaw_raw = math.degrees(
                        math.atan2(c[1][1]-c[0][1], c[1][0]-c[0][0]))
                    raw = (sx, sy, float(tv[2]), yaw_raw)

                    if smooth_pos is None:
                        smooth_pos = raw
                    else:
                        diff = (yaw_raw - smooth_pos[3] + 180) % 360 - 180
                        if abs(diff) > 45:
                            diff = 0
                        raw = (raw[0], raw[1], raw[2], smooth_pos[3] + diff)
                        smooth_pos = tuple(
                            EMA_ALPHA * r + (1 - EMA_ALPHA) * s
                            for r, s in zip(raw, smooth_pos))

                    last_det_t    = time.time()
                    detect_streak = min(detect_streak + 1, MIN_STREAK)
                    if detect_streak >= MIN_STREAK:
                        detected = smooth_pos

                    # Draw on frame
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    cx, cy = int(pix[0]), int(pix[1])
                    cv2.drawMarker(frame, (cx, cy), (0, 255, 0),
                                   cv2.MARKER_CROSS, 20, 2)
                    cv2.putText(frame,
                                f"X:{sx:.2f}m  Y:{sy:.2f}m  Z:{tv[2]:.2f}m",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0, 255, 0), 2)
                else:
                    ids = None

        if ids is None:
            if smooth_pos and (time.time() - last_det_t) < HOLD_SECS:
                detected = smooth_pos if detect_streak >= MIN_STREAK else None
            else:
                detect_streak = 0
                smooth_pos    = None

        # ── Read state ────────────────────────────────────────────────────
        with _lock:
            armed      = _state['armed']
            emergency  = _state['emergency']
            hover_only = _state['hover_only']
            tdelta     = _state['thrust_delta']

        # ── Emergency ─────────────────────────────────────────────────────
        if emergency:
            send_rpyt(0, 0, 0, 0)
            reset_pids()
            with _lock:
                _flight['baseline']    = False
                _flight['z_target']    = -1.0
                _flight['base_thrust'] = BLIND_CLIMB_START
                _flight['ramp_target'] = None
                _flight['baseline_yaw'] = None
            _adapt_avg    = 0.0
            smooth_pos    = None
            detect_streak = 0
            _overlay(frame, "EMERGENCY", (0, 0, 255))
            cv2.imshow("Hover Balance — PC Direct", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                with _lock: _state['emergency'] = not _state['emergency']
            prev_armed = False
            continue

        # ── Disarmed ──────────────────────────────────────────────────────
        if not armed:
            if prev_armed:
                # Just disarmed — safe descent
                bt = _flight['base_thrust']
                with _lock:
                    _flight.update(baseline=False, z_target=-1.0,
                                   base_thrust=BLIND_CLIMB_START,
                                   ramp_target=None, baseline_yaw=None)
                reset_pids()
                threading.Thread(target=safe_descend, args=(bt,),
                                 daemon=True).start()
            send_rpyt(0, 0, 0, 0)
            _overlay(frame, "DISARMED — press SPACE", (0, 200, 255))
            cv2.imshow("Hover Balance — PC Direct", frame)
            cv2.waitKey(1)
            prev_armed = False
            continue

        prev_armed = True

        # ── Armed — read flight state ──────────────────────────────────────
        with _lock:
            base_thrust  = _flight['base_thrust']
            ramp_target  = _flight['ramp_target']
            baseline     = _flight['baseline']
            z_target     = _flight['z_target']
            baseline_yaw = _flight['baseline_yaw']

        if detected is not None:
            x_m, y_m, z_m, yaw_raw = detected

            if not baseline:
                z_target     = z_m
                baseline_yaw = yaw_raw
                ramp_target  = TAKEOFF_RAMP_TO
                pid_elev.reset()
                _adapt_avg = 0.0
                print(f"[LOCK] ArUco locked — Z={z_m:.3f}m. PIDs engaged.")
                with _lock:
                    _flight.update(baseline=True, z_target=z_target,
                                   baseline_yaw=baseline_yaw,
                                   ramp_target=ramp_target)

            # Takeoff ramp
            if ramp_target is not None:
                step = (ramp_target - base_thrust) * (0.033 / TAKEOFF_SECS)
                if abs(ramp_target - base_thrust) <= abs(step) + 1:
                    base_thrust = ramp_target
                    ramp_target = None
                    print(f"[RAMP] Done — hovering at {base_thrust}")
                else:
                    base_thrust = int(base_thrust + step)
                with _lock:
                    _flight['ramp_target'] = ramp_target

            # Keyboard nudge
            base_thrust = max(10000, min(65000, base_thrust + tdelta))
            with _lock:
                _state['thrust_delta'] = 0

            # Lateral error
            dx   = x_m if abs(x_m) > DEAD_XY  else 0.0
            dy   = y_m if abs(y_m) > DEAD_XY  else 0.0
            tilt = (yaw_raw - baseline_yaw) if baseline_yaw else 0.0
            tilt = (tilt + 180) % 360 - 180
            tilt = tilt if abs(tilt) > DEAD_YAW else 0.0

            if hover_only:
                cmd_roll = cmd_pitch = cmd_yaw = 0.0
            else:
                cmd_roll  = pid_roll.compute(-dx)
                cmd_pitch = pid_pitch.compute(dy)
                cmd_yaw   = pid_yaw.compute(-tilt)

            # Elevation PID
            if ramp_target is not None or z_target <= 0:
                cmd_thrust_adj = 0.0
            else:
                elev_err = (z_m / z_target) - 1.0
                eff_elev = elev_err if abs(elev_err) > DEAD_ELEV else 0.0
                cmd_thrust_adj = pid_elev.compute(eff_elev)
                # Adaptive hover — slowly correct base_thrust drift
                if abs(elev_err) < 0.15:
                    _adapt_avg = ADAPT_ALPHA * cmd_thrust_adj + \
                                 (1 - ADAPT_ALPHA) * _adapt_avg
                    if abs(_adapt_avg) > ADAPT_THRESH:
                        nudge = max(-ADAPT_MAX, min(ADAPT_MAX, int(_adapt_avg)))
                        base_thrust = max(10000, min(65000, base_thrust + nudge))
                        pid_elev.integral -= nudge
                        _adapt_avg *= 0.3

            thrust = int(max(0, min(65000, base_thrust + cmd_thrust_adj)))
            send_rpyt(cmd_roll  + cfg.TRIM_ROLL,
                      cmd_pitch + cfg.TRIM_PITCH,
                      cmd_yaw   + cfg.TRIM_YAW,
                      thrust)

            with _lock:
                _flight['base_thrust'] = base_thrust

            # HUD
            cv2.putText(frame,
                        f"R:{cmd_roll:.1f}  P:{cmd_pitch:.1f}  "
                        f"Y:{cmd_yaw:.1f}  T:{thrust}",
                        (10, fh-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 0), 2)
            mode_str = "HOVER-ONLY" if hover_only else "TRACKING"
            _overlay(frame, f"ARMED  {mode_str}  Z={z_m:.2f}m", (0, 255, 100))

        else:
            # No marker — blind climb or hold
            if not baseline:
                if base_thrust < BLIND_CLIMB_MAX:
                    base_thrust = min(BLIND_CLIMB_MAX,
                                      base_thrust + BLIND_CLIMB_RATE)
                with _lock:
                    _flight['base_thrust'] = base_thrust
            send_rpyt(0, 0, 0, int(base_thrust))
            _overlay(frame, f"ARMED  searching marker…  T:{base_thrust}", (0, 165, 255))

        cv2.imshow("Hover Balance — PC Direct", frame)
        key = cv2.waitKey(1) & 0xFF
        _handle_cv_key(key)

    cv2.destroyAllWindows()
    try:
        scrcpy_proc.terminate()
    except Exception:
        pass


def _overlay(frame, text, color):
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, h-40), (w, h), (20, 20, 20), -1)
    cv2.putText(frame, text, (10, h-12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)


def _handle_cv_key(key):
    """Handle key presses from the OpenCV preview window."""
    if key == ord(' '):
        with _lock:
            _state['armed'] = not _state['armed']
    elif key == 27:   # ESC
        with _lock:
            prev = _state['emergency']
            _state['emergency'] = not prev
            if not prev:
                _state['armed'] = False
    elif key == 82:   # ↑ arrow
        with _lock:
            _state['thrust_delta'] += 500
    elif key == 84:   # ↓ arrow
        with _lock:
            _state['thrust_delta'] -= 500
    elif key in (ord('h'), ord('H')):
        with _lock:
            _state['hover_only'] = not _state['hover_only']
        print(f"[H] Hover-only: {_state['hover_only']}")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("=" * 60)
    print("  Hover Balance — Direct PC → Drone")
    print("=" * 60)
    print(f"  Camera : scrcpy ({CAM_FACING} camera)  →  {SCRCPY_EXE}")
    print(f"  Drone  : {DRONE_IP}:{DRONE_PORT}  (UDP direct)")
    print()
    print("  BEFORE RUNNING:")
    print("  1. Connect phone via USB cable")
    print("  2. Enable USB Debugging on phone (Developer Options)")
    print("  3. Connect PC WiFi to drone hotspot")
    print()
    print("  Controls (click the camera preview window first):")
    print("    SPACE — Arm / Disarm")
    print("    ESC   — Emergency stop (toggle)")
    print("    ↑ / ↓ — Thrust nudge ±500")
    print("    H     — Toggle hover-only (no lateral PID)")
    print("    Ctrl+C — Quit")
    print()
    print("  ArUco marker ID:", DRONE_MARKER_ID,
          "  Size:", MARKER_SIZE_M, "m")
    print()
    input("  Press ENTER to begin …")
    print()

    def _kb_space():
        with _lock:
            _state['armed'] = not _state['armed']
        print(f"[KEY] {'ARMED' if _state['armed'] else 'DISARMED'}")

    def _kb_esc():
        with _lock:
            prev = _state['emergency']
            _state['emergency'] = not prev
            if not prev:
                _state['armed'] = False
        print("[KEY] EMERGENCY TOGGLE")

    def _kb_up():
        with _lock:
            _state['thrust_delta'] += 500

    def _kb_down():
        with _lock:
            _state['thrust_delta'] -= 500

    def _kb_h():
        with _lock:
            _state['hover_only'] = not _state['hover_only']
        print(f"[KEY] Hover-only: {_state['hover_only']}")

    keyboard.add_hotkey('space', _kb_space)
    keyboard.add_hotkey('esc',   _kb_esc)
    keyboard.add_hotkey('up',    _kb_up)
    keyboard.add_hotkey('down',  _kb_down)
    keyboard.add_hotkey('h',     _kb_h)

    try:
        control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[EXIT] Cutting motors …")
        with _lock:
            _state['quit'] = True
        send_rpyt(0, 0, 0, 0)
        time.sleep(0.3)
        _sock.close()
        print("[EXIT] Done.")


if __name__ == '__main__':
    main()
