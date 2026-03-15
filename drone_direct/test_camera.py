"""
drone_direct/test_camera.py
============================
ArUco ID-7 Visualizer for GROUND CAMERA Setup.
Camera is flat on the ground pointing UP. Drone flies above it.

Features
  • Locks onto the FIRST detected ArUco marker
  • Calibrates home pose on first stable lock (MIN_STREAK frames)
  • Target Height (Z): Locks initial Z as target, adjustable via UP/DOWN keys
  • Z Logic Inverted: Z is distance *to* drone. If Z > target, Thrust must *decrease*.
  • Continuously computes PID counter-motion (roll / pitch / yaw / thrust)
  • Tilt-direction arrow shows which way drone would correct
  • Drone IMU streamed live via CRTP logging (stabilizer + gyro)
  • All numbers drawn on the camera feed

Run:
  cd g:/Projects/Hackathon/Neurotech
  .venv/Scripts/python.exe drone_direct/test_camera.py

Controls (click the preview window):
  R       — re-calibrate (reset home pose & target Z)
  UP/DOWN — adjust target hover height (target Z) by ±2cm
  Q       — quit
"""

import subprocess, sys, os, time, math, threading, socket, struct
import cv2
import numpy as np
import win32gui, win32ui, win32con


sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'tuner'))
import config as cfg

# ── Scrcpy ────────────────────────────────────────────────────────────────────
SCRCPY_EXE  = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', 'scrcpy',
    'scrcpy-win64-v3.0.2', 'scrcpy.exe'))
WIN_TITLE   = 'ARUCO_CAM_TEST'

# ── ArUco ─────────────────────────────────────────────────────────────────────
# We will lock onto the FIRST marker we see
MARKER_SIZE = 0.08      # metres (8 cm inner black square)
MIN_STREAK  = 4         # consecutive frames before home calibration locks
EMA_ALPHA   = 0.25      # pose smoothing (0 = frozen, 1 = raw)
HOLD_SECS   = 2.0       # keep LOCKED state this long after marker disappears

# ── PID gains (mirrors balance.py — no commands actually sent) ─────────────────
PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD  = 35.0,  1.0,   8.0
PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD = 35.0,  1.0,   8.0
PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD   = 15.0,  0.0,   1.5
PID_ELEV_KP,  PID_ELEV_KI,  PID_ELEV_KD  = 12000, 300.0, 3500.0
ROLL_LIMIT  = 20.0;  PITCH_LIMIT = 20.0
YAW_LIMIT   = 80.0;  ELEV_LIMIT  = 17000.0
DEAD_XY     = 0.02      # metres — ignore tiny lateral errors
DEAD_YAW    = 5.0       # degrees
DEAD_ELEV   = 0.03      # fractional Z error
BASE_HOVER_THRUST = 55000

# ── States ────────────────────────────────────────────────────────────────────
S_SEARCHING   = 'SEARCHING'
S_CALIBRATING = 'CALIBRATING'
S_LOCKED      = 'LOCKED'


# ─────────────────────────────────────────────────────────────────────────────
# PID controller
# ─────────────────────────────────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp = kp; self.ki = ki; self.kd = kd; self.limit = limit
        self.integral = self.prev_error = 0.0
        self.last_t = time.time()

    def reset(self):
        self.integral = self.prev_error = 0.0
        self.last_t = time.time()

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
# CRTP helpers
# ─────────────────────────────────────────────────────────────────────────────
def _cksum(b: bytes) -> int:
    return sum(b) & 0xFF

def _make_pkt(port: int, ch: int, payload: bytes) -> bytes:
    hdr  = ((port & 0xF) << 4) | (ch & 0x3)
    body = bytes([hdr]) + payload
    return body + bytes([_cksum(body)])

def _send_recv(sock, pkt: bytes, timeout=0.4, retries=3):
    want_port = (pkt[0] >> 4) & 0xF
    want_ch   = pkt[0] & 0x3
    old = sock.gettimeout()
    sock.settimeout(timeout)
    resp = None
    for _ in range(retries):
        try:
            sock.sendto(pkt, (cfg.DRONE_IP, cfg.DRONE_PORT))
            data, _ = sock.recvfrom(256)
            if len(data) >= 2:
                if (data[0] >> 4) & 0xF == want_port and data[0] & 0x3 == want_ch:
                    resp = data[1:-1]
                    break
        except socket.timeout:
            pass
    sock.settimeout(old)
    return resp


# ─────────────────────────────────────────────────────────────────────────────
# DroneIMU — background CRTP log-block reader
#   Reads stabilizer.roll/pitch/yaw + gyro.x/y/z at 10 Hz.
#   Fully optional — if WiFi not connected, status stays OFFLINE/NO DRONE.
# ─────────────────────────────────────────────────────────────────────────────
class DroneIMU:
    # Variables we want from the log TOC
    WANT = [
        ('stabilizer', 'roll'),
        ('stabilizer', 'pitch'),
        ('stabilizer', 'yaw'),
        ('gyro', 'x'),
        ('gyro', 'y'),
        ('gyro', 'z'),
    ]
    BLOCK_ID = 1

    def __init__(self):
        self._lock   = threading.Lock()
        self._data   = {}
        self._status = 'OFFLINE'
        self._stop   = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True

    @property
    def data(self):
        with self._lock:
            return dict(self._data)

    @property
    def status(self):
        with self._lock:
            return self._status

    def _set(self, s):
        with self._lock:
            self._status = s

    def _run(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(0.5)

            # ── Fetch log TOC to find variable IDs ────────────────────────────
            self._set('TOC...')
            resp = _send_recv(sock, _make_pkt(5, 0, bytes([0x01])))  # GetInfo
            if resp is None or len(resp) < 3:
                self._set('NO DRONE')
                return

            total = struct.unpack_from('<H', resp, 1)[0]
            found = {}   # (group, name) → (toc_idx, type_nibble)

            for idx in range(min(total, 400)):
                if self._stop:
                    return
                pkt  = _make_pkt(5, 0, bytes([0x00]) + struct.pack('<H', idx))
                resp = _send_recv(sock, pkt, timeout=0.3, retries=2)
                if resp and len(resp) >= 5:
                    try:
                        payload  = resp[4:]
                        n1       = payload.index(0)
                        grp      = payload[:n1].decode('ascii', errors='replace')
                        rest     = payload[n1+1:]
                        n2       = rest.index(0)
                        name     = rest[:n2].decode('ascii', errors='replace')
                        vtype    = resp[3] & 0x0F   # lower nibble = CRTP type
                        if (grp, name) in [(g, n) for g, n in self.WANT]:
                            found[(grp, name)] = (idx, vtype)
                    except (ValueError, IndexError):
                        pass
                if len(found) == len(self.WANT):
                    break   # found all we need early

            if not found:
                self._set('VAR_NOT_FOUND')
                return

            # ── Create log block ──────────────────────────────────────────────
            # Delete any stale block first
            _send_recv(sock, _make_pkt(5, 1, bytes([0x05, self.BLOCK_ID])), retries=1)
            time.sleep(0.05)

            # CMD_CREATE_BLOCK = 0x00, then (type_byte=0x08 float, var_id 2-byte LE)*
            create_payload = bytes([0x00, self.BLOCK_ID])
            ordered_keys   = [k for k in self.WANT if k in found]
            for (g, n) in ordered_keys:
                var_idx, _ = found[(g, n)]
                create_payload += bytes([0x08]) + struct.pack('<H', var_idx)  # 0x08 = float

            resp = _send_recv(sock, _make_pkt(5, 1, create_payload))
            if resp is None or (len(resp) >= 3 and resp[2] != 0x00):
                self._set('BLK_ERR')
                return

            # ── Start block at 10 Hz (period = 10 × 10 ms = 100 ms) ──────────
            resp = _send_recv(sock, _make_pkt(5, 1,
                              bytes([0x03, self.BLOCK_ID, 10])))
            if resp is None:
                self._set('START_ERR')
                return

            self._set('LIVE')
            var_names = [n for (g, n) in ordered_keys]  # e.g. ['roll','pitch',...]

            # ── Receive log data on Port 5, Channel 2 ─────────────────────────
            sock.settimeout(1.0)
            while not self._stop:
                try:
                    raw, _ = sock.recvfrom(256)
                    if len(raw) < 2:
                        continue
                    rp = (raw[0] >> 4) & 0xF
                    rc = raw[0] & 0x3
                    if rp != 5 or rc != 2:
                        continue
                    payload = raw[1:-1]          # strip header & checksum
                    if len(payload) < 4:
                        continue
                    if payload[0] != self.BLOCK_ID:
                        continue
                    # payload: [block_id, ts_lo, ts_mid, ts_hi, f0, f1, ...]
                    data_bytes = payload[4:]
                    vals = {}
                    for i, name in enumerate(var_names):
                        offset = i * 4
                        if offset + 4 <= len(data_bytes):
                            vals[name] = struct.unpack_from('<f', data_bytes, offset)[0]
                    with self._lock:
                        self._data.update(vals)
                except socket.timeout:
                    pass
                except Exception:
                    pass

        except Exception as e:
            self._set(f'ERR:{str(e)[:20]}')
        finally:
            try:
                sock.close()
            except Exception:
                pass


# ─────────────────────────────────────────────────────────────────────────────
# Window capture (scrcpy)
# ─────────────────────────────────────────────────────────────────────────────
def grab_window(hwnd):
    try:
        l, t, r, b = win32gui.GetWindowRect(hwnd)
        w, h = r - l, b - t
        if w <= 0 or h <= 0:
            return None
        hdc = win32gui.GetWindowDC(hwnd)
        mdc = win32ui.CreateDCFromHandle(hdc)
        sdc = mdc.CreateCompatibleDC()
        bmp = win32ui.CreateBitmap()
        bmp.CreateCompatibleBitmap(mdc, w, h)
        sdc.SelectObject(bmp)
        sdc.BitBlt((0, 0), (w, h), mdc, (0, 0), win32con.SRCCOPY)
        raw = bmp.GetBitmapBits(True)
        win32gui.DeleteObject(bmp.GetHandle())
        sdc.DeleteDC(); mdc.DeleteDC()
        win32gui.ReleaseDC(hwnd, hdc)
        img = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 4)
        return cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGRA2BGR), (640, 480))
    except Exception:
        return None


def find_window(title):
    hwnd = win32gui.FindWindow(None, title)
    return hwnd if hwnd else None


# ─────────────────────────────────────────────────────────────────────────────
# HUD helpers
# ─────────────────────────────────────────────────────────────────────────────
def _panel(frame, x, y, w, h, alpha=0.60):
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, y), (x + w, y + h), (10, 10, 10), -1)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

def _txt(frame, text, x, y, scale=0.52, color=(220, 220, 220), thick=1):
    cv2.putText(frame, text, (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)

def _sf(v, d=2):
    """Signed float string: +1.23 / -0.45"""
    return f"{v:+.{d}f}"


# ─────────────────────────────────────────────────────────────────────────────
# Draw full HUD
# ─────────────────────────────────────────────────────────────────────────────
def draw_hud(frame, state, streak, pose, home, target_z, cmd, imu_vals, imu_status, tracked_id):
    fh, fw = frame.shape[:2]

    # ── Left panel: marker pose + home delta + Target Z ────────────────────────
    panel_h = 180 if state == S_LOCKED else 75
    _panel(frame, 0, 0, 318, panel_h)

    id_str = f"ID {tracked_id}" if tracked_id is not None else "ANY ID"
    _txt(frame, f"TARGET MARKER: {id_str}", 8, 18, 0.56, (80, 255, 80), 2)

    if pose:
        x, y, z, yaw = pose
        _txt(frame, f"X {_sf(x)}m   Y {_sf(y)}m",     8, 38, 0.52, (180, 255, 180))
        _txt(frame, f"Z {z:.3f}m   Yaw {yaw:+.1f}°",  8, 57, 0.52, (180, 255, 180))
    else:
        _txt(frame, "No pose data", 8, 38, 0.50, (140, 140, 140))

    if state == S_LOCKED and home and pose:
        hx, hy, hz, hyaw = home
        x, y, z, yaw     = pose
        _txt(frame, f"HOME X {_sf(hx)}  Y {_sf(hy)}",
             8, 80, 0.48, (130, 180, 255))
        
        # TARGET Z (Adjustable) vs CURRENT Z
        _txt(frame, f"TARGET Z: {target_z:.3f}m (UP/DN to adj)", 8, 100, 0.50, (255, 100, 255), 2)
        
        dx   = x  - hx
        dy   = y  - hy
        dz   = z  - target_z  # if z > target, drone is too high
        dyaw = (yaw - hyaw + 180) % 360 - 180
        
        _txt(frame, f"dX {_sf(dx)}  dY {_sf(dy)}  dZ-Err {_sf(dz)}m",
             8, 120, 0.50, (255, 210, 80))
        _txt(frame, f"dYaw {dyaw:+.1f}°   streak {streak}",
             8, 140, 0.50, (255, 210, 80))
        
        # colour-coded lateral offset indicator
        mag_xy = math.hypot(dx, dy)
        dot_color = (0, 220, 0) if mag_xy < 0.05 else (0, 140, 255) if mag_xy < 0.15 else (0, 50, 255)
        _txt(frame, f"|dXY| {mag_xy:.3f}m", 8, 162, 0.50, dot_color)
        _txt(frame, "[GROUND CAMERA UP]", 8, 182, 0.45, (100, 100, 100))

    # ── Right panel: counter-motion commands ──────────────────────────────────
    if state == S_LOCKED and cmd is not None:
        cr, cp, cy_v, ct = cmd
        _panel(frame, fw - 215, 0, 215, 120)
        _txt(frame, "COUNTER-MOTION CMD", fw - 210, 18,  0.50, (255, 170, 60), 2)
        _txt(frame, f"Roll    {cr:+7.2f} deg",   fw - 210, 38,  0.50, (255, 230, 100))
        _txt(frame, f"Pitch   {cp:+7.2f} deg",   fw - 210, 58,  0.50, (255, 230, 100))
        _txt(frame, f"Yaw   {cy_v:+7.2f} deg/s", fw - 210, 78,  0.50, (255, 230, 100))
        _txt(frame, f"Thrust {int(ct):6d}",       fw - 210, 98,  0.52, (80,  255, 180))

        # ── Tilt-direction arrow indicator ────────────────────────────────────
        ax, ay = fw - 107, 158
        r_circ = 40
        _panel(frame, ax - r_circ - 4, ay - r_circ - 4,
               (r_circ + 4) * 2, (r_circ + 4) * 2 + 20, alpha=0.5)
        cv2.circle(frame, (ax, ay), r_circ, (60, 60, 60), -1)
        cv2.circle(frame, (ax, ay), r_circ, (120, 120, 120), 1)
        # Crosshair
        cv2.line(frame, (ax - r_circ, ay), (ax + r_circ, ay), (60, 60, 60), 1)
        cv2.line(frame, (ax, ay - r_circ), (ax, ay + r_circ), (60, 60, 60), 1)
        # Arrow: direction drone would tilt to counter error
        mag = min(1.0, math.hypot(cr, cp) / PITCH_LIMIT)
        if mag > 0.04:
            ang = math.atan2(cp, -cr)   # +pitch → down screen, +roll → right
            ex  = int(ax + r_circ * 0.85 * mag * math.cos(ang))
            ey  = int(ay + r_circ * 0.85 * mag * math.sin(ang))
            arrow_color = (0, 200, 80) if mag < 0.4 else (0, 120, 255) if mag < 0.75 else (0, 40, 255)
            cv2.arrowedLine(frame, (ax, ay), (ex, ey), arrow_color, 2, tipLength=0.35)
        cv2.circle(frame, (ax, ay), 4, (255, 255, 255), -1)
        _txt(frame, "tilt dir", ax - 22, ay + r_circ + 16, 0.40, (160, 160, 160))

    # ── IMU panel (bottom, full width) ────────────────────────────────────────
    imu_top = fh - 102
    _panel(frame, 0, imu_top, fw, 66)

    _txt(frame, f"DRONE IMU  [{imu_status}]", 8, imu_top + 16, 0.50,
         (80, 200, 255) if imu_status == 'LIVE' else (120, 120, 120), 1)

    if imu_vals:
        def fv(key):
            v = imu_vals.get(key, float('nan'))
            return f"{v:+7.1f}" if not math.isnan(v) else "  --- "
        _txt(frame,
             f"Stab   R {fv('roll')}  P {fv('pitch')}  Y {fv('yaw')}  deg",
             8, imu_top + 36, 0.48, (170, 220, 255))
        _txt(frame,
             f"Gyro   X {fv('x')}  Y {fv('y')}  Z {fv('z')}  deg/s",
             8, imu_top + 54, 0.48, (170, 220, 255))
    else:
        _txt(frame, "Connect PC WiFi to drone hotspot to see live IMU data",
             8, imu_top + 40, 0.46, (100, 100, 100))

    # ── Status bar (very bottom) ──────────────────────────────────────────────
    bar_top = fh - 36
    _panel(frame, 0, bar_top, fw, 36, alpha=0.82)

    if state == S_SEARCHING:
        col    = (0, 100, 255)
        status = f"SEARCHING for ANY ArUco Marker  —  point camera at marker"
    elif state == S_CALIBRATING:
        col    = (0, 200, 255)
        status = f"CALIBRATING ID {tracked_id}  {streak}/{MIN_STREAK} frames  —  hold still..."
    else:
        col    = (0, 220, 80)
        status = f"LOCKED | R=Recalibrate | UP/DN=Adj Height | Q=Quit"

    _txt(frame, status, 10, bar_top + 24, 0.60, col, 2)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("=" * 60)
    print("  ArUco ID-7 Visualizer w/ Target Height Adjustment")
    print("=" * 60)
    print(f"  Marker size : {MARKER_SIZE*100:.0f} cm  |  Streak for lock: {MIN_STREAK} frames")
    print(f"  Drone WiFi  : {cfg.DRONE_IP}:{cfg.DRONE_PORT}  (optional — for IMU)")
    print(f"  UP/DOWN ARROWS = adjust target hover height")
    print()
    print("  Starting scrcpy camera window...")

    proc = subprocess.Popen([
        SCRCPY_EXE, '--video-source=camera', '--camera-facing=back',
        '--no-audio', f'--window-title={WIN_TITLE}', '--max-fps=30',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    hwnd = None
    for _ in range(30):
        hwnd = find_window(WIN_TITLE)
        if hwnd:
            break
        time.sleep(0.5)

    if not hwnd:
        print("ERROR: scrcpy window did not appear.")
        print("  → Check USB connection and tap Allow on phone screen.")
        proc.terminate()
        sys.exit(1)

    print(f"  Camera ready (hwnd={hwnd})")
    print(f"  Point camera at ANY ArUco Marker — press Q to quit\n")

    # ── ArUco setup ────────────────────────────────────────────────────────────
    aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_params.adaptiveThreshWinSizeMin  = 3
    aruco_params.adaptiveThreshWinSizeMax  = 53
    aruco_params.adaptiveThreshWinSizeStep = 4
    aruco_params.minMarkerPerimeterRate    = 0.03
    aruco_params.cornerRefinementMethod    = cv2.aruco.CORNER_REFINE_SUBPIX
    aruco_params.errorCorrectionRate       = 0.6
    try:
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    except AttributeError:
        detector = None

    clahe       = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    cam_matrix  = None
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    # ── PIDs (simulation only) ─────────────────────────────────────────────────
    pid_roll  = PID(PID_ROLL_KP,  PID_ROLL_KI,  PID_ROLL_KD,  ROLL_LIMIT)
    pid_pitch = PID(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PITCH_LIMIT)
    pid_yaw   = PID(PID_YAW_KP,   PID_YAW_KI,   PID_YAW_KD,   YAW_LIMIT)
    pid_elev  = PID(PID_ELEV_KP,  PID_ELEV_KI,  PID_ELEV_KD,  ELEV_LIMIT)

    # ── Drone IMU background thread ────────────────────────────────────────────
    imu = DroneIMU()
    imu.start()

    # ── State ──────────────────────────────────────────────────────────────────
    state       = S_SEARCHING
    streak      = 0
    smooth_pos  = None          # EMA-smoothed (x, y, z, yaw)
    home_pose   = None          # calibration reference pose (hx, hy, hz, hyaw)
    target_z    = 1.0           # Adjustable desired hover height
    cmd         = None          # (roll, pitch, yaw, thrust) counter-motion
    last_seen_t = 0.0           # time of last marker detection
    tracked_id  = None          # which marker ID we are currently tracking

    while True:
        frame = grab_window(hwnd)
        if frame is None:
            hwnd = find_window(WIN_TITLE)
            time.sleep(0.05)
            continue

        fh, fw = frame.shape[:2]

        if cam_matrix is None:
            cam_matrix = np.array([[fw, 0, fw / 2],
                                   [0,  fw, fh / 2],
                                   [0,  0,  1      ]], dtype=np.float32)

        # ── ArUco detect ───────────────────────────────────────────────────────
        gray = clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        try:
            if detector:
                corners, ids, _ = detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, aruco_dict, parameters=aruco_params)
        except Exception:
            ids = None

        # ── Find the target marker ─────────────────────────────────────────────
        pose = None
        if ids is not None:
            # If we don't have a tracked_id yet (searching), pick the first one we see
            if tracked_id is None and len(ids) > 0:
                tracked_id = int(ids[0][0])
                print(f"[DETECT] Found new marker ID: {tracked_id}. Locking on.")

            # Look for the tracked_id
            valid = [i for i, mid in enumerate(ids.flatten()) if mid == tracked_id]

            # Draw all detected markers dimly (grey) so the user can see others
            if corners:
                for idx_c, corner in enumerate(corners):
                    mid = int(ids.flatten()[idx_c])
                    if mid == tracked_id:
                        continue   # we'll draw the target specially below
                    pts = corner[0].astype(np.int32)
                    cv2.polylines(frame, [pts], True, (80, 80, 80), 1)
                    cx_o = int(np.mean(pts[:, 0]))
                    cy_o = int(np.mean(pts[:, 1]))
                    _txt(frame, f"ID{mid} (ignored)", cx_o - 30, cy_o,
                         0.40, (80, 80, 80))

            if valid:
                c    = corners[valid[0]][0]
                diag = math.hypot(c[2][0] - c[0][0], c[2][1] - c[0][1])

                # Draw target marker prominently
                pts = c.astype(np.int32)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                for pt in c:
                    cv2.circle(frame, tuple(pt.astype(int)), 5, (0, 200, 255), -1)
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))
                cv2.drawMarker(frame, (cx, cy), (0, 255, 0),
                               cv2.MARKER_CROSS, 32, 2)
                _txt(frame, f"ID {tracked_id}", cx - 20, cy - 20,
                     0.65, (0, 255, 255), 2)

                if diag >= 20:
                    half    = MARKER_SIZE / 2.0
                    obj_pts = np.array([[-half,  half, 0], [ half,  half, 0],
                                        [ half, -half, 0], [-half, -half, 0]],
                                       dtype=np.float32)
                    _, rvec, tvec = cv2.solvePnP(
                        obj_pts, c, cam_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    tv  = tvec.flatten()
                    sx  = (np.mean(c[:, 0]) - fw / 2) / fw * tv[2]
                    sy  = (np.mean(c[:, 1]) - fh / 2) / fw * tv[2]
                    yaw_raw = math.degrees(
                        math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0]))
                    raw = (float(sx), float(sy), float(tv[2]), yaw_raw)

                    if smooth_pos is None:
                        smooth_pos = raw
                    else:
                        dyaw = (yaw_raw - smooth_pos[3] + 180) % 360 - 180
                        if abs(dyaw) > 45:
                            dyaw = 0.0
                        raw2 = (raw[0], raw[1], raw[2], smooth_pos[3] + dyaw)
                        smooth_pos = tuple(
                            EMA_ALPHA * r + (1 - EMA_ALPHA) * s
                            for r, s in zip(raw2, smooth_pos))

                    pose        = smooth_pos
                    last_seen_t = time.time()

                    # Z label near marker
                    cv2.putText(frame, f"Z={tv[2]:.3f}m",
                                (cx + 18, cy + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 255, 200), 2, cv2.LINE_AA)

        # ── State machine ──────────────────────────────────────────────────────
        if pose is None:
            time_since = time.time() - last_seen_t
            if state == S_LOCKED and time_since < HOLD_SECS:
                pass     # hold LOCKED briefly after losing marker
            elif state == S_LOCKED and time_since >= HOLD_SECS:
                state      = S_SEARCHING
                streak     = 0
                smooth_pos = None
                cmd        = None
                tracked_id = None
                print("[LOST] Marker gone — resetting to SEARCHING for ANY marker")
            else:
                state      = S_SEARCHING
                streak     = 0
                smooth_pos = None
                tracked_id = None
        else:
            if state == S_SEARCHING:
                state  = S_CALIBRATING
                streak = 1
                pid_roll.reset(); pid_pitch.reset()
                pid_yaw.reset();  pid_elev.reset()
                print(f"[DETECT] Calibrating ({MIN_STREAK} frames)...")
            elif state == S_CALIBRATING:
                streak += 1
                if streak >= MIN_STREAK:
                    home_pose = smooth_pos
                    target_z  = home_pose[2]  # Lock initial Z as target
                    state     = S_LOCKED
                    print(f"[LOCKED] Home  X={home_pose[0]:+.3f}  Y={home_pose[1]:+.3f}"
                          f"  Z={home_pose[2]:.3f}m  Yaw={home_pose[3]:+.1f}°")
                    print(f"[LOCKED] Target Z set to {target_z:.3f}m")
            # else: already LOCKED — keep computing

        # ── Counter-motion PID (Ground Camera Perspective) ─────────────────────
        if state == S_LOCKED and home_pose and pose:
            x, y, z, yaw    = pose
            hx, hy, hz, hyaw = home_pose

            dx   = x - hx
            dy   = y - hy
            dyaw = (yaw - hyaw + 180) % 360 - 180

            ex   = dx   if abs(dx)   > DEAD_XY  else 0.0
            ey   = dy   if abs(dy)   > DEAD_XY  else 0.0
            eyaw = dyaw if abs(dyaw) > DEAD_YAW else 0.0

            # Ground camera pointing UP: 
            # If drone moves right in frame (positive X), ex is positive.
            # To move left, it must roll LEFT (negative angle, depending on drone orientation).
            # Note: ex is inverted to counteract, but ey maps correctly:
            # OpenCV Y increases downwards. Drone forward drift moves marker UP (-Y).
            # -Y requires backward pitch (-Pitch). So +ey maps to +pitch.
            cr    = pid_roll.compute(-ex)
            cp    = pid_pitch.compute(ey)
            cy_v  = pid_yaw.compute(-eyaw)

            # Elevation PID (Ground Camera Perspective):
            # Camera measures distance TO drone. 
            # If Z > Target_Z, drone is TOO HIGH, error is positive.
            # Base thrust needs to DECREASE.
            if target_z > 0:
                elev_err = (z / target_z) - 1.0  # +ve means too high
                eff_elev = elev_err if abs(elev_err) > DEAD_ELEV else 0.0
                ct_adj   = pid_elev.compute(eff_elev)
                # **CRITICAL INVERSION HERE**: To fix +ve error (too high), drop thrust.
                # So we SUBTRACT ct_adj from BASE_HOVER_THRUST.
                ct  = max(0, min(65535, BASE_HOVER_THRUST - int(ct_adj)))
            else:
                ct  = BASE_HOVER_THRUST
            cmd = (cr + cfg.TRIM_ROLL,
                   cp + cfg.TRIM_PITCH,
                   cy_v + cfg.TRIM_YAW,
                   ct)
        elif state != S_LOCKED:
            cmd = None

        # ── IMU data ───────────────────────────────────────────────────────────
        imu_vals   = imu.data   if imu.status == 'LIVE' else None
        imu_status = imu.status

        # ── Draw HUD ───────────────────────────────────────────────────────────
        draw_hud(frame, state, streak, pose, home_pose, target_z, cmd, imu_vals, imu_status, tracked_id)

        cv2.imshow("ArUco Cam — Visualizer (UP/DN = Target Z)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('Q'):
            break
        elif key == ord('r') or key == ord('R'):
            state      = S_SEARCHING
            streak     = 0
            smooth_pos = None
            home_pose  = None
            target_z   = 1.0
            cmd        = None
            tracked_id = None
            pid_roll.reset(); pid_pitch.reset()
            pid_yaw.reset();  pid_elev.reset()
            print("[RECAL] Home pose cleared — waiting for new lock on ANY marker")
        
        # ── UP/DOWN arrows for Target Z adjustment ─────────────────────────────
        elif key == 82:  # UP arrow
            target_z += 0.02  # increase height by 2cm
            print(f"[Z-ADJ] Target Z increased to {target_z:.3f}m")
        elif key == 84:  # DOWN arrow
            target_z = max(0.1, target_z - 0.02)  # decrease height by 2cm
            print(f"[Z-ADJ] Target Z decreased to {target_z:.3f}m")

    imu.stop()
    proc.terminate()
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == '__main__':
    main()
