"""
drone_direct/fly_ground_cam.py
==============================
Hand-held to autonomous flight test using a GROUND CAMERA (pointing UP).

Workflow:
  1. Phone is flat on ground, pointing UP at the ceiling.
  2. Hold drone over camera. Props should be completely stopped (DISARMED).
  3. Press SPACE to ARM. Props will spin at IDLE_THRUST (safe to hold).
  4. Raise the drone in your hands.
  5. Once Z (altitude) crosses 0.4m, ACTIVE FLIGHT mode begins.
     - PIDs engage to lock the X/Y position.
     - Target Z is locked to whatever height it was when you crossed 0.4m.
  6. Let go of the drone (or hold lightly to feel the correction).
  7. Press SPACE to drop it, or ESC for instant kill.
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

# ── Scrcpy ────────────────────────────────────────────────────────────────────
_SCRCPY_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'scrcpy',
                 'scrcpy-win64-v3.0.2'))
SCRCPY_EXE     = os.path.join(_SCRCPY_DIR, 'scrcpy.exe')
CAM_WIDTH      = 640
CAM_HEIGHT     = 480
CAM_FACING     = 'back'

# ── ArUco ─────────────────────────────────────────────────────────────────────
MARKER_SIZE_M   = 0.04        # 4 cm
# We will lock onto the FIRST marker we see if None, or force an ID here.
DRONE_MARKER_ID = None        

# ── Flight Params — initial values pulled from tuner/config.py ───────────────
# Edit config.py to change these; fly_ground_cam picks them up automatically.
HOVER_SEARCH       = cfg.HOVER_THRUST    # Starting hover reference (config: HOVER_THRUST)
TAKEOFF_RAMP_TIME  = cfg.THRUST_RAMP_TIME  # Seconds to ramp 0 → hover thrust
TAKEOFF_MAX_THRUST = cfg.HOVER_THRUST    # Ramp to hover thrust — matches hover_gui behaviour

# Launch-catch: on ACTIVE entry, hold sub-hover thrust to bleed upward momentum
# before handing off to PID. Prevents the drone rocketing to the ceiling.
LAUNCH_THRUST = int(cfg.HOVER_THRUST * 0.88)  # ~88% of hover — sub-hover to brake climb
LAUNCH_SECS   = 2.0                           # Catch phase duration (seconds)

# Adaptive Hover Thrust Learning
# The real hover thrust is unknown. We start at cfg.HOVER_THRUST and slowly ramp
# up/down by watching whether the drone is consistently above or below target.
# ADAPT_ALPHA  : learning rate (0 = never changes, 1 = tracks instantly)
# ADAPT_THRESH : min average correction to trigger a base update
# ADAPT_MAX    : max step per update (prevents wild jumps)
ADAPT_ALPHA  = 0.003          # very conservative
ADAPT_THRESH = 1000           # units of thrust
ADAPT_MAX    = 2000           # units of thrust per nudge
HOVER_MIN    = cfg.THRUST_MIN
HOVER_MAX    = cfg.THRUST_MAX

HANDOVER_Z  = 0.40            # Altitude (m) where PIDs take over from Idle
CEILING_Z   = 1.20            # Hard ceiling (m) — thrust reduced if exceeded

# ── Safety boundary ───────────────────────────────────────────────────────────
# If the marker pixel position moves outside this fraction of the frame from
# center, motors are KILLED immediately (drone drifted too far away).
BOUNDARY_FRAC = 1          # 60 % of frame width/height from center

# ── PID gains (Ground Cam Perspective) ────────────────────────────────────────
PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD  = 35.0,  1.0,   8.0
PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD = 35.0,  1.0,   8.0
PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD   = 15.0,  0.0,   1.5
PID_ELEV_KP,  PID_ELEV_KI,  PID_ELEV_KD  = 12000, 300.0, 3500.0

ROLL_LIMIT  = 20.0
PITCH_LIMIT = 20.0
YAW_LIMIT   = 80.0
ELEV_LIMIT  = 17000.0

DEAD_XY   = 0.02
DEAD_YAW  = 5.0
DEAD_ELEV = 0.03

EMA_ALPHA  = 0.25
HOLD_SECS  = 1.5              # Max time to fly blind — 1.5s to survive vibration dropout on launch
MIN_STREAK  = 10              # Frames needed for ACCURATE pose used by ACTIVE PIDs
IDLE_STREAK = 2               # Frames needed to confirm drone is there before spinning IDLE props

# ESC unlock duration — pulled from config.py (UNLOCK_DURATION)
ESC_UNLOCK_SECS = cfg.UNLOCK_DURATION

# ── Ceiling brake zone ────────────────────────────────────────────────────────────
# From CEILING_WARN_Z to CEILING_Z, thrust is blended proportionally from
# hover down to near-zero. This gives a smooth slowdown rather than a hard cut.
CEILING_WARN_Z = 0.85         # Brake warning threshold (70% of 1.2m)

# ── Auto-landing (once camera locks) ──────────────────────────────────────────
# After the first confident camera lock, target_z is frozen at the detected
# height then steadily reduced at DESCEND_RATE m/s until LANDING_Z is reached,
# at which point motors are cut via safe_descend().
DESCEND_RATE = 0.06           # m/s — gentle descent speed
LANDING_Z    = 0.15           # m  — cut motors when this low


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
# UDP
# ─────────────────────────────────────────────────────────────────────────────
def _cksum(b: bytes) -> int:
    return sum(b) & 0xFF

def make_rpyt(roll, pitch, yaw, thrust) -> bytes:
    header  = (3 << 4) | 0
    t       = max(0, min(65535, int(thrust)))
    payload = struct.pack('<BfffH', header,
                          float(roll), float(pitch), float(yaw), t)
    return payload + bytes([_cksum(payload)])

_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_rpyt(roll, pitch, yaw, thrust):
    try:
        _sock.sendto(make_rpyt(roll, pitch, yaw, thrust), (DRONE_IP, DRONE_PORT))
    except Exception:
        pass


# ─────────────────────────────────────────────────────────────────────────────
# State
# ─────────────────────────────────────────────────────────────────────────────
_lock  = threading.Lock()
_state = {
    'armed':        False,
    'emergency':    False,
    'quit':         False,
    'tdelta':       0.0,
}

# ── Drone connectivity status (updated by background thread) ────────────────────
_drone_online = False

def _ping_loop():
    global _drone_online
    while True:
        try:
            result = subprocess.run(
                ['ping', '-n', '1', '-w', '300', DRONE_IP],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            _drone_online = (result.returncode == 0)
        except Exception:
            _drone_online = False
        time.sleep(2.0)

threading.Thread(target=_ping_loop, daemon=True).start()

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
# Camera Utils
# ─────────────────────────────────────────────────────────────────────────────
SCRCPY_WIN_TITLE = 'GROUND_CAM_FLIGHT'

def _open_scrcpy_camera():
    cmd = [
        SCRCPY_EXE, '--video-source=camera', f'--camera-facing={CAM_FACING}',
        '--no-audio', '--max-fps=30', f'--window-title={SCRCPY_WIN_TITLE}',
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    hwnd = None
    for _ in range(30):
        time.sleep(0.5)
        def _cb(h, _):
            if win32gui.IsWindowVisible(h) and SCRCPY_WIN_TITLE in win32gui.GetWindowText(h):
                hwnd_box.append(h)
        hwnd_box = []
        win32gui.EnumWindows(_cb, None)
        if hwnd_box: return proc, hwnd_box[0]
    proc.terminate()
    raise RuntimeError("scrcpy window did not appear")

def _grab_frame(hwnd):
    try:
        l, t, r, b = win32gui.GetWindowRect(hwnd)
        w, h = r - l, b - t
        if w <= 0 or h <= 0: return None
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
        return cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGRA2BGR), (CAM_WIDTH, CAM_HEIGHT))
    except Exception:
        return None

def _overlay(frame, text, color, y_offset=12):
    h, w = frame.shape[:2]
    # Draw bottom bar
    cv2.rectangle(frame, (0, h-40), (w, h), (20, 20, 20), -1)
    cv2.putText(frame, text, (10, h - y_offset),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)

def _draw_hud_status(frame):
    """Top-right corner: small drone connectivity indicator."""
    h, w = frame.shape[:2]
    dot_color = (0, 220, 60) if _drone_online else (0, 0, 220)
    label     = f"Drone: {DRONE_IP}" if _drone_online else "Drone: OFFLINE"
    cv2.circle(frame, (w - 14, 14), 6, dot_color, -1)
    cv2.putText(frame, label, (w - 160, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, dot_color, 1)

def _draw_boundary(frame):
    """Draw a semi-transparent coloured rectangle showing the safe-fly zone."""
    h, w = frame.shape[:2]
    bx = int(w * (1 - BOUNDARY_FRAC) / 2)
    by = int(h * (1 - BOUNDARY_FRAC) / 2)
    ex = w - bx
    ey = h - by
    overlay = frame.copy()
    cv2.rectangle(overlay, (bx, by), (ex, ey), (0, 165, 255), -1)  # orange fill
    cv2.addWeighted(overlay, 0.12, frame, 0.88, 0, frame)           # blend
    cv2.rectangle(frame, (bx, by), (ex, ey), (0, 165, 255), 2)     # solid border
    cv2.putText(frame, f"SAFE ZONE ({int(BOUNDARY_FRAC*100)}%)",
                (bx + 4, by + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
    return bx, by, ex, ey

def control_loop():
    pid_roll  = PID(PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  ROLL_LIMIT)
    pid_pitch = PID(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PITCH_LIMIT)
    pid_yaw   = PID(PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   YAW_LIMIT)
    pid_elev  = PID(PID_ELEV_KP,  PID_ELEV_KI,  PID_ELEV_KD,  ELEV_LIMIT)

    def reset_pids():
        for p in (pid_roll, pid_pitch, pid_yaw, pid_elev): p.reset()

    aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters()
    # ── STRICT settings to eliminate false positives ──
    aruco_params.cornerRefinementMethod      = cv2.aruco.CORNER_REFINE_SUBPIX
    aruco_params.errorCorrectionRate         = 0.4   # 0.6 is default; 0.4 cuts obvious false positives without breaking real markers
    aruco_params.minMarkerPerimeterRate      = 0.04  # must subtend at least 4% of frame perimeter
    aruco_params.maxMarkerPerimeterRate      = 4.0
    aruco_params.minCornerDistanceRate       = 0.05
    aruco_params.minDistanceToBorder         = 5
    _clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    try:
        _detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        _detector = None

    cam_matrix  = None
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    smooth_pos    = None
    last_det_t    = 0.0
    detect_streak = 0
    tracked_id    = DRONE_MARKER_ID

    # Flight Phase state
    flight_phase  = 'DISARMED' # DISARMED -> TAKING_OFF -> ACTIVE
    takeoff_start_t = 0.0
    target_z      = -1.0
    baseline_yaw  = None
    home_x        = 0.0
    home_y        = 0.0
    curr_thrust   = 0
    base_hover    = cfg.HOVER_THRUST   # real hover baseline from config
    adapt_avg     = 0.0
    esc_unlock_until    = 0.0
    launch_catch_until  = 0.0          # ACTIVE launch-catch phase end time

    scrcpy_proc, cam_hwnd = _open_scrcpy_camera()
    prev_armed = False

    while True:
        with _lock:
            if _state['quit']: break
            armed     = _state['armed']
            emergency = _state['emergency']
            tdelta    = float(_state['tdelta'])
            _state['tdelta'] = 0.0

        frame = _grab_frame(cam_hwnd)
        if frame is None:
            time.sleep(0.02)
            continue

        fh, fw = frame.shape[:2]
        if cam_matrix is None:
            cam_matrix = np.array([[fw, 0, fw/2], [0, fw, fh/2], [0, 0, 1]], dtype=np.float32)

        # ── Draw safe-zone boundary and HUD status on every frame ───────────
        bx, by, bex, bey = _draw_boundary(frame)
        _draw_hud_status(frame)

        # ── Detect ──────────────────────────────────────────────────────────
        gray = _clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        try:
            if _detector: corners, ids, _ = _detector.detectMarkers(gray)
            else:         corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        except Exception: ids = None

        detected     = None
        raw_detected = None
        if ids is not None:
            # If searching, pick the LARGEST visible marker (most likely the real drone marker)
            if tracked_id is None and len(ids) > 0:
                best_idx  = 0
                best_diag = 0.0
                for ci, corner in enumerate(corners):
                    d = math.hypot(corner[0][2][0] - corner[0][0][0],
                                   corner[0][2][1] - corner[0][0][1])
                    if d > best_diag:
                        best_diag = d
                        best_idx  = ci
                tracked_id = int(ids[best_idx][0])
                print(f"[DETECT] Locked onto largest marker ID: {tracked_id} (diag={best_diag:.0f}px)")

            valid = [i for i, mid in enumerate(ids.flatten()) if mid == tracked_id]
            if valid:
                c = corners[valid[0]][0]
                diag = math.hypot(c[2][0]-c[0][0], c[2][1]-c[0][1])
                if diag >= 20:
                    half = MARKER_SIZE_M / 2.0
                    obj_pts = np.array([[-half, half, 0], [half, half, 0],
                                        [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
                    _, _, tvec = cv2.solvePnP(obj_pts, c, cam_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    
                    tv = tvec.flatten()
                    pix = (float(np.mean(c[:, 0])), float(np.mean(c[:, 1])))
                    sx = (pix[0] - fw/2) / fw * tv[2]
                    sy = (pix[1] - fh/2) / fw * tv[2]
                    yaw_raw = math.degrees(math.atan2(c[1][1]-c[0][1], c[1][0]-c[0][0]))
                    raw = (sx, sy, float(tv[2]), yaw_raw)

                    if smooth_pos is None:
                        smooth_pos = raw
                    else:
                        diff = (yaw_raw - smooth_pos[3] + 180) % 360 - 180
                        if abs(diff) > 45: diff = 0
                        raw = (raw[0], raw[1], raw[2], smooth_pos[3] + diff)
                        smooth_pos = tuple(EMA_ALPHA * r + (1 - EMA_ALPHA) * s for r, s in zip(raw, smooth_pos))

                    last_det_t    = time.time()
                    detect_streak = min(detect_streak + 1, MIN_STREAK)
                    # IDLE_STREAK frames) to spin props.
                    # ACTIVE mode needs the full detected (MIN_STREAK frames) for accurate PIDs.
                    if detect_streak >= IDLE_STREAK:
                        raw_detected = smooth_pos
                    if detect_streak >= MIN_STREAK:
                        detected = smooth_pos

                    # ── Boundary check (pixel-level, ACTIVE only) ─────────
                    # During TAKING_OFF we expect it to be uncentered. Kill only in ACTIVE.
                    pix_x, pix_y = int(pix[0]), int(pix[1])
                    outside_zone = not (bx < pix_x < bex and by < pix_y < bey)
                    if outside_zone and flight_phase == 'ACTIVE':
                        print(f"[BOUNDARY] Drift outside safe zone during ACTIVE. KILL. pix=({pix_x},{pix_y})")
                        send_rpyt(0, 0, 0, 0)
                        with _lock:
                            _state['armed']     = False
                            _state['emergency'] = False
                        reset_pids()
                        flight_phase  = 'DISARMED'
                        smooth_pos    = None
                        detect_streak = 0
                        curr_thrust      = 0
                        esc_unlock_until = time.time() + ESC_UNLOCK_SECS
                        prev_armed       = False
                        tracked_id       = DRONE_MARKER_ID
                        detected         = None  # skip ACTIVE logic below
                    elif outside_zone:
                        # TAKING_OFF — just warn on-screen
                        cv2.putText(frame, "ALIGN TO SAFE ZONE BEFORE TAKEOFF",
                                    (bx + 4, bey - 8), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.45, (0, 0, 255), 1)

                    # Draw marker
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    cv2.putText(frame, f"Z:{tv[2]:.2f}m", (int(pix[0]), int(pix[1])-20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if ids is None:
            if smooth_pos and (time.time() - last_det_t) < HOLD_SECS:
                if detect_streak >= MIN_STREAK:
                    detected = smooth_pos
                if detect_streak >= IDLE_STREAK:
                    raw_detected = smooth_pos
            else:
                detect_streak = 0
                smooth_pos    = None
                raw_detected  = None

        # ── Control Logic ───────────────────────────────────────────────────
        if emergency:
            # ESC = instant motor kill and full state reset.
            # Must re-arm intentionally after clearing emergency.
            send_rpyt(0, 0, 0, 0)
            reset_pids()
            flight_phase  = 'DISARMED'
            tracked_id    = DRONE_MARKER_ID
            smooth_pos    = None
            detect_streak = 0
            curr_thrust      = 0
            esc_unlock_until = time.time() + ESC_UNLOCK_SECS
            prev_armed       = False
            with _lock:
                _state['armed']     = False  # Force disarm
                _state['emergency'] = False  # Auto-clear so ESC is a one-shot kill
            _overlay(frame, "EMERGENCY KILL — Press SPACE to re-arm", (0, 0, 255))
            cv2.imshow("Ground Cam Flight Test", frame)
            cv2.waitKey(1)
            continue

        if not armed:
            if prev_armed:
                # Just disarmed — trigger safe descent AND start ESC unlock timer
                threading.Thread(target=safe_descend, args=(curr_thrust,), daemon=True).start()
                curr_thrust      = 0
                esc_unlock_until = time.time() + ESC_UNLOCK_SECS
                print(f"[ESC] Unlock pulse started ({ESC_UNLOCK_SECS}s of zeros)")
            send_rpyt(0, 0, 0, 0)
            reset_pids()
            flight_phase = 'DISARMED'
            tracked_id   = DRONE_MARKER_ID
            unlocking = time.time() < esc_unlock_until
            if unlocking:
                remaining = esc_unlock_until - time.time()
                _overlay(frame, f"ESC UNLOCK ({remaining:.1f}s) — wait before re-arming", (180, 100, 0))
            else:
                _overlay(frame, "DISARMED — Hold drone & Press SPACE", (0, 200, 255))
            cv2.imshow("Ground Cam Flight Test", frame)
            cv2.waitKey(1)
            prev_armed = False
            continue

        prev_armed = True

        # ── Tracking lost handling ──────────────────────────────────────────────────────────
        # ACTIVE mode needs the full detected (MIN_STREAK frames) for accurate PIDs.
        if flight_phase == 'ACTIVE' and detected is None:
            print("[LOST] Tracking lost during ACTIVE flight. Safe descend.")
            reset_pids()
            threading.Thread(target=safe_descend, args=(curr_thrust,), daemon=True).start()
            curr_thrust      = 0
            esc_unlock_until = time.time() + ESC_UNLOCK_SECS
            flight_phase     = 'DISARMED'
            with _lock: _state['armed'] = False
            prev_armed   = False
            tracked_id   = DRONE_MARKER_ID
            continue

        # Pick the best available position: accurate if ACTIVE, loose if TAKING_OFF
        pose = detected if detected is not None else raw_detected
        x_m, y_m, z_m, yaw_raw = pose if pose is not None else (0.0, 0.0, 0.0, 0.0)

        # Flight State Machine
        if flight_phase == 'DISARMED':
            # Blind launch — drone may or may not be in frame. Arm and ramp
            # thrust unconditionally (same ramp-up as hover_gui). Camera will
            # catch the drone mid-air and take over automatically.
            flight_phase    = 'TAKING_OFF'
            takeoff_start_t = time.time()
            print("[TAKEOFF] Armed. Blind ramp started — searching for drone with camera...")

        if flight_phase == 'TAKING_OFF':
            if time.time() < esc_unlock_until:
                # MUST send 2.0s of zeros before ESC allows motors to spin
                send_rpyt(0, 0, 0, 0)
                takeoff_start_t = time.time()  # delay ramp start
                remain = esc_unlock_until - time.time()
                _overlay(frame, f"TAKEOFF — Unlocking ESC ({remain:.1f}s)", (0, 165, 255))
            else:
                t_elapsed = time.time() - takeoff_start_t
                ramp_frac = min(1.0, t_elapsed / TAKEOFF_RAMP_TIME)
                curr_thrust = int(ramp_frac * TAKEOFF_MAX_THRUST)
                send_rpyt(cfg.TRIM_ROLL, cfg.TRIM_PITCH, cfg.TRIM_YAW, curr_thrust)
                
                if pose is None:
                    _overlay(frame, f"TAKING OFF (BLIND)  T:{curr_thrust}  Waiting for camera lock…", (0, 100, 255))
                elif detected is None:
                    streak_pct = int(detect_streak / MIN_STREAK * 100)
                    _overlay(frame, f"TAKEOFF  Z≈{z_m:.2f}m  Calibrating ({streak_pct}%)  T:{curr_thrust}", (0, 165, 255))
                else:
                    _overlay(frame, f"TAKEOFF  Z={z_m:.2f}m  Camera locked — handing over…  T:{curr_thrust}", (0, 165, 255))

            # Hand over as soon as we have a confident camera lock — at any height.
            # Freeze target_z at whatever the drone is right now; ACTIVE will
            # descend from there automatically.
            if detected is not None:
                flight_phase = 'ACTIVE'
                target_z     = float(z_m)   # lock height, then descend from here
                baseline_yaw = yaw_raw
                home_x       = 0.0
                home_y       = 0.0
                reset_pids()
                base_hover   = cfg.HOVER_THRUST
                adapt_avg    = 0.0
                curr_thrust  = LAUNCH_THRUST   # sub-hover to brake any upward momentum
                launch_catch_until = time.time() + LAUNCH_SECS
                print(f"[ACTIVE] Camera lock at Z={target_z:.2f}m  base_hover={base_hover}  CATCH {LAUNCH_SECS}s → auto-descend to {LANDING_Z}m")

        elif flight_phase == 'ACTIVE':
            # ── LAUNCH CATCH phase ───────────────────────────────────────────────
            # Immediately after takeoff handover, drone has upward momentum.
            # Hold sub-hover thrust for LAUNCH_SECS to let gravity brake the climb.
            # During this window, we do NOT constantly update target_z to peak anymore.
            # target_z is firmly set to 0.40. PIDs will bring it down.
            if time.time() < launch_catch_until:
                catch_remaining = launch_catch_until - time.time()
                curr_thrust = LAUNCH_THRUST
                # Still output RPY corrections during catch so it centers quickly
                dx = x_m - home_x
                dy = y_m - home_y
                tilt = (yaw_raw - baseline_yaw)
                tilt = (tilt + 180) % 360 - 180
                ex = dx if abs(dx) > DEAD_XY else 0.0
                ey = dy if abs(dy) > DEAD_XY else 0.0
                eyaw = tilt if abs(tilt) > DEAD_YAW else 0.0
                cmd_roll  = pid_roll.compute(-ex)
                cmd_pitch = pid_pitch.compute(ey)
                cmd_yaw   = pid_yaw.compute(-eyaw)
                
                send_rpyt(cmd_roll + cfg.TRIM_ROLL, cmd_pitch + cfg.TRIM_PITCH, cmd_yaw + cfg.TRIM_YAW, curr_thrust)
                _overlay(frame,
                    f"CATCH  Z:{z_m:.2f}m  braking ({catch_remaining:.1f}s)  → will descend to {LANDING_Z}m",
                    (0, 220, 255))
                cv2.imshow("Ground Cam Flight Test", frame)
                cv2.waitKey(1)
                continue

            # ── Proportional Ceiling Brake Zone ─────────────────────────────────
            # TOP PRIORITY: prevent ceiling collision.
            # From CEILING_WARN_Z (0.85m) to CEILING_Z (1.2m) we blend thrust
            # linearly toward zero. This overrides PID output.
            if z_m >= CEILING_WARN_Z:
                # How far into the brake zone? 0.0=entering, 1.0=at hard ceiling
                frac = (z_m - CEILING_WARN_Z) / (CEILING_Z - CEILING_WARN_Z)
                frac = min(1.0, max(0.0, frac))
                # Ceiling thrust = 20k at ceiling, 35k at warning
                ceiling_thrust = int(35000 * (1.0 - frac) + 20000 * frac)
                curr_thrust    = ceiling_thrust
                send_rpyt(0, 0, 0, curr_thrust)
                col = (0, 100, 255) if frac < 0.5 else (0, 0, 255)
                _overlay(frame, f"CEILING BRAKE Z:{z_m:.2f}m  ({int(frac*100)}% to limit)  T:{curr_thrust}", col)
                cv2.imshow("Ground Cam Flight Test", frame)
                cv2.waitKey(1)
                continue

            # Auto-descend at DESCEND_RATE m/s. tdelta (Up/Down keys) can nudge
            # the target height temporarily (useful for manual override).
            target_z = max(LANDING_Z, min(CEILING_Z - 0.05,
                           target_z - DESCEND_RATE / cfg.SEND_HZ + tdelta))

            dx = x_m - home_x
            dy = y_m - home_y
            tilt = (yaw_raw - baseline_yaw)
            tilt = (tilt + 180) % 360 - 180

            ex = dx if abs(dx) > DEAD_XY else 0.0
            ey = dy if abs(dy) > DEAD_XY else 0.0
            eyaw = tilt if abs(tilt) > DEAD_YAW else 0.0

            # Ground Cam Perspective
            # ex is inverted to counteract. ey maps correctly (+Y in OpenCV is DOWN screen)
            cmd_roll  = pid_roll.compute(-ex)
            cmd_pitch = pid_pitch.compute(ey)
            cmd_yaw   = pid_yaw.compute(-eyaw)

            # ── Adaptive hover baseline ───────────────────────────────────────
            # Fast: pid_elev handles immediate Z correction.
            # Slow: adapt_avg learns the true hover thrust over time.
            elev_err = (z_m / target_z) - 1.0  # +ve = too high, -ve = too low
            eff_elev = elev_err if abs(elev_err) > DEAD_ELEV else 0.0
            cmd_thrust_adj = pid_elev.compute(eff_elev)

            # Slowly drift base_hover toward the true hover point
            if abs(elev_err) < 0.15:          # only adapt when close to target
                adapt_avg = ADAPT_ALPHA * cmd_thrust_adj + (1 - ADAPT_ALPHA) * adapt_avg
                if abs(adapt_avg) > ADAPT_THRESH:
                    nudge       = max(-ADAPT_MAX, min(ADAPT_MAX, int(adapt_avg)))
                    base_hover  = max(HOVER_MIN, min(HOVER_MAX, base_hover - nudge))
                    pid_elev.integral -= nudge   # anti-windup: remove absorbed error
                    adapt_avg   *= 0.3           # damp after adapting
                    print(f"[ADAPT] base_hover nudged to {base_hover}")

            # Ground Cam Elevation: +ve error (too high) -> SUBTRACT
            curr_thrust = int(max(0, min(65000, base_hover - cmd_thrust_adj)))

            send_rpyt(cmd_roll  + cfg.TRIM_ROLL,
                      cmd_pitch + cfg.TRIM_PITCH,
                      cmd_yaw   + cfg.TRIM_YAW,
                      curr_thrust)

            # ── Auto-land when we've reached LANDING_Z ──────────────────────
            if target_z <= LANDING_Z and z_m <= LANDING_Z + 0.12:
                print(f"[LAND] Reached landing height Z={z_m:.2f}m — safe descend.")
                threading.Thread(target=safe_descend, args=(curr_thrust,), daemon=True).start()
                with _lock:
                    _state['armed'] = False
                curr_thrust      = 0
                esc_unlock_until = time.time() + ESC_UNLOCK_SECS
                flight_phase     = 'DISARMED'
                prev_armed       = False
                reset_pids()

            _overlay(frame, f"ACTIVE Z:{z_m:.2f}m  Tgt:{target_z:.2f}m  BaseHov:{base_hover}  T:{curr_thrust}  ↓{DESCEND_RATE}m/s", (0, 255, 100))
            cv2.putText(frame, f"R:{cmd_roll:+.1f} P:{cmd_pitch:+.1f} Y:{cmd_yaw:+.1f}", (10, fh-45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("Ground Cam Flight Test", frame)
        key = cv2.waitKey(1) & 0xFF
        # ESC — kill motors but stay running (re-arm with SPACE)
        if key == 27:
            with _lock:
                _state['armed']     = False
                _state['emergency'] = False
            send_rpyt(0, 0, 0, 0)
            reset_pids()
            flight_phase     = 'DISARMED'
            curr_thrust      = 0
            esc_unlock_until = time.time() + ESC_UNLOCK_SECS
            prev_armed       = False
            tracked_id       = DRONE_MARKER_ID
            print("[ESC] Motors killed.")
        # Q — quit the application entirely
        elif key in (ord('q'), ord('Q')):
            send_rpyt(0, 0, 0, 0)
            with _lock: _state['quit'] = True
            print("[Q] Quitting.")
            break
        # R — attempt software ESC reset (2s zero-thrust unlock, like spin_test.py)
        elif key in (ord('r'), ord('R')):
            def _esc_reset():
                print("[R] ESC reset: sending 2s of zeros to unlock ESC...")
                t_end = time.time() + 2.0
                while time.time() < t_end:
                    send_rpyt(0, 0, 0, 0)
                    time.sleep(0.02)
                print("[R] ESC reset done — re-arm with SPACE")
            threading.Thread(target=_esc_reset, daemon=True).start()
            with _lock:
                _state['armed'] = False
            flight_phase = 'DISARMED'
            curr_thrust  = 0
            prev_armed   = False

    cv2.destroyAllWindows()
    try: scrcpy_proc.terminate()
    except Exception: pass


# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("=" * 60)
    print("  Ground Cam Flight Test (Hand-held Release)")
    print("=" * 60)
    print("  1. Phone flat on ground, pointing UP.")
    print("  2. Run this script.")
    print("  3. Hold drone over camera (<0.4m).")
    print("  4. Press SPACE. Props spin slowly (IDLE).")
    print("  5. Lift drone above 0.4m -> PIDs engage (ACTIVE).")
    print("  6. Let go or ride it gently to feel corrections.")
    print("  7. SPACE again to softly drop.")
    print("=" * 60)

    last_space_t = [0.0]
    def _kb_space():
        if time.time() - last_space_t[0] > 1.0:
            with _lock: _state['armed'] = not _state['armed']
            last_space_t[0] = time.time()
    def _kb_esc():
        with _lock: _state['emergency'] = not _state['emergency']
    def _kb_up():
        with _lock: _state['tdelta'] += 0.05
    def _kb_down():
        with _lock: _state['tdelta'] -= 0.05

    keyboard.add_hotkey('space', _kb_space)
    keyboard.add_hotkey('esc',   _kb_esc)
    keyboard.add_hotkey('up',    _kb_up)
    keyboard.add_hotkey('down',  _kb_down)

    try:
        control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n[EXIT] Cutting motors …")
        with _lock: _state['quit'] = True
        send_rpyt(0, 0, 0, 0)
        time.sleep(0.3)
        _sock.close()
        try: keyboard.unhook_all()
        except: pass
        print("[EXIT] Done.")

if __name__ == '__main__':
    main()
