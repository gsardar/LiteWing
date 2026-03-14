"""
Neurotech Phone Balance — runs on Android via Termux
Full autonomous flight controller: ArUco detection + all PIDs + drone comms.

Responsibilities:
  - Reads camera via android-ip-camera MJPEG stream (localhost)
  - Detects ArUco marker → X, Y, Z
  - Runs all PIDs: roll, pitch, yaw, elevation → sends RPYT to drone
  - Receives arm/emergency/hover flags + thrust nudge from laptop
  - Sends X, Y, Z, base_thrust back to laptop for HUD display

Phone setup (Termux):
  pkg install python
  pip install opencv-python-headless numpy   (or: pkg install opencv-python)
  # Start android-ip-camera app → tap Start
  # Stream: https://127.0.0.1:4444/stream
  python phone_balance.py

PC setup:
  adb forward tcp:9391 tcp:9391
  python track_tilt.py

Protocol — Laptop → Phone (4 bytes):
  B  flags        bit0=armed  bit1=emergency  bit2=hover_only
  h  thrust_delta signed int16 nudge from arrow keys
  B  checksum

Protocol — Phone → Laptop (17 bytes):
  f  x_m          lateral offset metres
  f  y_m          forward offset metres
  f  z_m          depth metres  (-1 = not detected)
  I  base_thrust  current thrust
  B  checksum
"""

import socket
import struct
import threading
import time
import math

import cv2
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
LISTEN_HOST   = '127.0.0.1'
LISTEN_PORT   = 9391
DRONE_IP      = '192.168.43.42'
DRONE_PORT    = 2390
CAM_URL       = 'https://127.0.0.1:4444/stream'  # android-ip-camera

MARKER_SIZE_M   = 0.08
DRONE_MARKER_ID = 7

# ── Incoming command (laptop → phone) ─────────────────────────────────────────
CMD_FMT  = '<Bh'
CMD_BODY = struct.calcsize(CMD_FMT)   # 3 bytes
CMD_FULL = CMD_BODY + 1               # 4 bytes

FLAG_ARMED     = 0x01
FLAG_EMERGENCY = 0x02
FLAG_HOVER     = 0x04

# ── Outgoing position report (phone → laptop) ─────────────────────────────────
POS_FMT  = '<fffI'
POS_BODY = struct.calcsize(POS_FMT)   # 16 bytes
POS_FULL = POS_BODY + 1               # 17 bytes

# ── ArUco ─────────────────────────────────────────────────────────────────────
aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_params = cv2.aruco.DetectorParameters()
aruco_params.adaptiveThreshWinSizeMin   = 3
aruco_params.adaptiveThreshWinSizeMax   = 53
aruco_params.adaptiveThreshWinSizeStep  = 4
aruco_params.minMarkerPerimeterRate     = 0.03
aruco_params.cornerRefinementMethod     = cv2.aruco.CORNER_REFINE_SUBPIX
aruco_params.errorCorrectionRate        = 0.6
_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
try:
    _detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
except AttributeError:
    _detector = None

_cam_matrix  = None
_dist_coeffs = np.zeros((4, 1), dtype=np.float32)

# ── PID ───────────────────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self.integral = self.prev_error = 0.0
        self.last_t   = time.time()

    def reset(self):
        self.integral = self.prev_error = 0.0
        self.last_t   = time.time()

    def compute(self, error):
        now = time.time()
        dt  = max(now - self.last_t, 0.001)
        self.integral   = max(-self.limit, min(self.limit, self.integral + error * dt))
        d               = (error - self.prev_error) / dt
        out             = self.kp * error + self.ki * self.integral + self.kd * d
        self.prev_error = error
        self.last_t     = now
        return max(-self.limit, min(self.limit, out))

pid_roll  = PID(kp=35.0,     ki=1.0,   kd=8.0,    limit=20.0)
pid_pitch = PID(kp=35.0,     ki=1.0,   kd=8.0,    limit=20.0)
pid_yaw   = PID(kp=15.0,     ki=0.0,   kd=1.5,    limit=80.0)
pid_elev  = PID(kp=12000.0,  ki=300.0, kd=3500.0, limit=17000.0)

DEAD_ZONE_XY   = 0.02
DEAD_ZONE_YAW  = 5.0
DEAD_ZONE_ELEV = 0.03

BLIND_CLIMB_START = 35000
BLIND_CLIMB_MAX   = 64000
BLIND_CLIMB_RATE  = 500    # per camera frame ~30fps
TAKEOFF_RAMP_TO   = 55000  # target thrust on first ArUco lock
TAKEOFF_SECS      = 1.5

ADAPT_ALPHA  = 0.005
ADAPT_THRESH = 1500
ADAPT_MAX    = 3000

# ── Shared state ──────────────────────────────────────────────────────────────
_lock   = threading.Lock()
_state  = {
    'armed':        False,
    'emergency':    False,
    'hover_only':   False,
    'thrust_delta': 0,      # cumulative nudge from laptop arrow keys
}
_flight = {
    'base_thrust':  BLIND_CLIMB_START,
    'ramp_target':  None,
    'baseline':     False,
    'z_target':     -1.0,
    'baseline_yaw': None,
    'x_m': 0.0, 'y_m': 0.0, 'z_m': -1.0,
}
_adapt_avg  = 0.0
_conn_ref   = [None]
_udp_sock   = [None]

# ── RPYT packet ───────────────────────────────────────────────────────────────
def cksum(data: bytes) -> int:
    return sum(data) & 0xFF

def make_rpyt(roll, pitch, yaw, thrust) -> bytes:
    header  = (3 << 4) | 0
    t       = max(0, min(65535, int(thrust)))
    payload = struct.pack('<BfffH', header, float(roll), float(pitch), float(yaw), t)
    return payload + bytes([cksum(payload)])

def send_rpyt(roll, pitch, yaw, thrust):
    udp = _udp_sock[0]
    if udp:
        try:
            udp.sendto(make_rpyt(roll, pitch, yaw, thrust), (DRONE_IP, DRONE_PORT))
        except Exception:
            pass

# ── Safe descent (called in thread on disarm) ─────────────────────────────────
def safe_descend(from_thrust):
    steps, secs = 40, 2.0
    delay = secs / steps
    print(f"[*] Safe descent from {from_thrust} → 0 ({secs:.1f}s)")
    for i in range(steps):
        t = (i + 1) / steps
        thrust = max(0, int(from_thrust * (1.0 - t)))
        send_rpyt(0, 0, 0, thrust)
        time.sleep(delay)
    send_rpyt(0, 0, 0, 0)
    print("[*] Descent complete.")

# ── Camera + ArUco + PID loop ─────────────────────────────────────────────────
def camera_loop():
    global _adapt_avg, _cam_matrix

    print(f"[CAM] Connecting to {CAM_URL} ...")
    cap = cv2.VideoCapture(CAM_URL, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        print("[CAM] ERROR: Cannot open stream. Is android-ip-camera running?")
        return
    print("[CAM] Stream open.")

    smooth_pos    = None
    EMA_ALPHA     = 0.25
    HOLD_SECS     = 0.20
    last_det_t    = 0.0
    detect_streak = 0
    MIN_STREAK    = 4

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.05)
            continue

        fh, fw = frame.shape[:2]
        if _cam_matrix is None:
            _cam_matrix = np.array([[fw, 0, fw/2],
                                    [0, fw, fh/2],
                                    [0,  0,    1]], dtype=np.float32)

        # ── ArUco detection ────────────────────────────────────────────────
        gray = _clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        try:
            corners, ids, _ = (_detector.detectMarkers(gray) if _detector
                               else cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params))
        except Exception:
            ids = None

        detected = None
        if ids is not None:
            valid = [i for i, mid in enumerate(ids.flatten()) if mid == DRONE_MARKER_ID]
            if valid:
                c    = corners[valid[0]][0]
                diag = math.hypot(c[2][0]-c[0][0], c[2][1]-c[0][1])
                if diag >= 20:
                    half    = MARKER_SIZE_M / 2.0
                    obj_pts = np.array([[-half, half,0],[half, half,0],
                                        [half,-half,0],[-half,-half,0]], dtype=np.float32)
                    _, _, tvec = cv2.solvePnP(obj_pts, c, _cam_matrix, _dist_coeffs,
                                              flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    tv  = tvec.flatten()
                    pix = (float(np.mean(c[:,0])), float(np.mean(c[:,1])))
                    sx  = (pix[0] - fw/2) / fw * tv[2]
                    sy  = (pix[1] - fh/2) / fw * tv[2]
                    yaw_raw = math.degrees(math.atan2(c[1][1]-c[0][1], c[1][0]-c[0][0]))
                    raw = (sx, sy, float(tv[2]), yaw_raw)

                    if smooth_pos is None:
                        smooth_pos = raw
                    else:
                        # Yaw unwrap
                        diff = (yaw_raw - smooth_pos[3] + 180) % 360 - 180
                        if abs(diff) > 45: diff = 0
                        raw = (raw[0], raw[1], raw[2], smooth_pos[3] + diff)
                        smooth_pos = tuple(EMA_ALPHA*r + (1-EMA_ALPHA)*s
                                          for r, s in zip(raw, smooth_pos))
                    last_det_t    = time.time()
                    detect_streak = min(detect_streak + 1, MIN_STREAK)
                    if detect_streak >= MIN_STREAK:
                        detected = smooth_pos
                else:
                    ids = None

        if ids is None:
            if smooth_pos and (time.time() - last_det_t) < HOLD_SECS:
                detected = smooth_pos if detect_streak >= MIN_STREAK else None
            else:
                detect_streak = 0
                smooth_pos    = None

        # ── Read operator state ────────────────────────────────────────────
        with _lock:
            armed      = _state['armed']
            emergency  = _state['emergency']
            hover_only = _state['hover_only']
            tdelta     = _state['thrust_delta']

        # ── Emergency ─────────────────────────────────────────────────────
        if emergency:
            send_rpyt(0, 0, 0, 0)
            pid_roll.reset(); pid_pitch.reset()
            pid_yaw.reset();  pid_elev.reset()
            with _lock:
                _flight['baseline']    = False
                _flight['z_target']    = -1.0
                _flight['base_thrust'] = BLIND_CLIMB_START
                _flight['ramp_target'] = None
                _flight['baseline_yaw'] = None
            _adapt_avg = 0.0
            smooth_pos = None
            detect_streak = 0
            _send_pos(-1.0, -1.0, -1.0, 0)
            continue

        # ── Idle ──────────────────────────────────────────────────────────
        if not armed:
            send_rpyt(0, 0, 0, 0)
            _send_pos(0.0, 0.0, -1.0, 0)
            continue

        # ── Armed: read flight state ───────────────────────────────────────
        base_thrust  = _flight['base_thrust']
        ramp_target  = _flight['ramp_target']
        baseline     = _flight['baseline']
        z_target     = _flight['z_target']
        baseline_yaw = _flight['baseline_yaw']

        if detected is not None:
            x_m, y_m, z_m, yaw_raw = detected

            # First lock → set targets
            if not baseline:
                z_target     = z_m
                baseline_yaw = yaw_raw
                base_thrust  = base_thrust   # keep current climb thrust
                ramp_target  = TAKEOFF_RAMP_TO
                pid_elev.reset()
                _adapt_avg = 0.0
                print(f"[*] ArUco locked — Z={z_m:.3f}m. Engaging PIDs.")
                _flight.update(baseline=True, z_target=z_target,
                               baseline_yaw=baseline_yaw, ramp_target=ramp_target)

            # Takeoff ramp
            if ramp_target is not None:
                step = (ramp_target - base_thrust) * (0.033 / TAKEOFF_SECS)
                if abs(ramp_target - base_thrust) <= abs(step) + 1:
                    base_thrust = ramp_target
                    ramp_target = None
                    print(f"[*] Ramp done — hovering at {base_thrust}")
                else:
                    base_thrust = int(base_thrust + step)
                _flight['ramp_target'] = ramp_target

            # Apply laptop thrust nudge
            base_thrust = max(10000, min(65000, base_thrust + tdelta))
            with _lock:
                _state['thrust_delta'] = 0   # consume nudge

            # Lateral PID
            dx   = x_m if abs(x_m) > DEAD_ZONE_XY  else 0.0
            dy   = y_m if abs(y_m) > DEAD_ZONE_XY  else 0.0
            tilt = (yaw_raw - baseline_yaw) if baseline_yaw is not None else 0.0
            tilt = (tilt + 180) % 360 - 180
            tilt = tilt if abs(tilt) > DEAD_ZONE_YAW else 0.0

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
                eff_elev = elev_err if abs(elev_err) > DEAD_ZONE_ELEV else 0.0
                cmd_thrust_adj = pid_elev.compute(eff_elev)

                # Adaptive hover
                if abs(elev_err) < 0.15:
                    _adapt_avg = ADAPT_ALPHA * cmd_thrust_adj + (1 - ADAPT_ALPHA) * _adapt_avg
                    if abs(_adapt_avg) > ADAPT_THRESH:
                        nudge        = max(-ADAPT_MAX, min(ADAPT_MAX, int(_adapt_avg)))
                        base_thrust  = max(10000, min(65000, base_thrust + nudge))
                        pid_elev.integral -= nudge
                        _adapt_avg  *= 0.3

            thrust = int(max(0, min(65000, base_thrust + cmd_thrust_adj)))
            send_rpyt(cmd_roll, cmd_pitch, cmd_yaw, thrust)

            _flight['base_thrust'] = base_thrust
            _flight['x_m'] = x_m
            _flight['y_m'] = y_m
            _flight['z_m'] = z_m
            _send_pos(x_m, y_m, z_m, base_thrust)

        else:
            # No detection — blind climb or hold
            if not baseline:
                if base_thrust < BLIND_CLIMB_MAX:
                    base_thrust = min(BLIND_CLIMB_MAX, base_thrust + BLIND_CLIMB_RATE)
                _flight['base_thrust'] = base_thrust
            send_rpyt(0, 0, 0, int(base_thrust))
            _send_pos(0.0, 0.0, -1.0, base_thrust)

def _send_pos(x_m, y_m, z_m, base_thrust):
    conn = _conn_ref[0]
    if conn:
        try:
            body = struct.pack(POS_FMT, float(x_m), float(y_m), float(z_m),
                               max(0, min(65535, int(base_thrust))))
            conn.sendall(body + bytes([cksum(body)]))
        except Exception:
            pass

# ── TCP handler — receives flags from laptop ──────────────────────────────────
def handle_client(conn, addr):
    print(f"[+] Laptop connected from {addr}")
    _conn_ref[0] = conn
    buf = b''
    prev_armed = False
    try:
        while True:
            chunk = conn.recv(256)
            if not chunk:
                break
            buf += chunk
            while len(buf) >= CMD_FULL:
                msg  = buf[:CMD_FULL]
                buf  = buf[CMD_FULL:]
                body = msg[:CMD_BODY]
                if cksum(body) != msg[CMD_BODY]:
                    continue
                flags, tdelta = struct.unpack(CMD_FMT, body)
                armed     = bool(flags & FLAG_ARMED)
                emergency = bool(flags & FLAG_EMERGENCY)
                hover     = bool(flags & FLAG_HOVER)

                with _lock:
                    _state['armed']      = armed
                    _state['emergency']  = emergency
                    _state['hover_only'] = hover
                    if tdelta != 0:
                        _state['thrust_delta'] += tdelta

                # Disarm → trigger safe descent
                if prev_armed and not armed and not emergency:
                    bt = _flight['base_thrust']
                    _flight.update(baseline=False, z_target=-1.0,
                                   base_thrust=BLIND_CLIMB_START,
                                   ramp_target=None, baseline_yaw=None)
                    pid_roll.reset(); pid_pitch.reset()
                    pid_yaw.reset();  pid_elev.reset()
                    threading.Thread(target=safe_descend, args=(bt,), daemon=True).start()

                prev_armed = armed
    except Exception as e:
        print(f"[!] {e}")
    finally:
        _conn_ref[0] = None
        conn.close()
        print("[-] Laptop disconnected")

def main():
    print("=" * 50)
    print("  Neurotech Phone Balance  (Full Autonomous)")
    print("=" * 50)
    print(f"  Camera : {CAM_URL}")
    print(f"  Listen : {LISTEN_HOST}:{LISTEN_PORT}")
    print(f"  Drone  : {DRONE_IP}:{DRONE_PORT}")
    print()

    _udp_sock[0] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    threading.Thread(target=camera_loop, daemon=True).start()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((LISTEN_HOST, LISTEN_PORT))
    server.listen(5)
    print("[*] Ready. Waiting for laptop...")

    while True:
        conn, addr = server.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

if __name__ == "__main__":
    main()
