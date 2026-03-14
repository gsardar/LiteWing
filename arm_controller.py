import cv2
import numpy as np
import win32gui
import win32ui
import win32con
import time
import math
import socket
import json
import os
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from collections import deque

# ── Session Calibration Logger ──
CALIB_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib_logs")
os.makedirs(CALIB_LOG_DIR, exist_ok=True)

def save_calib_session(data: dict):
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(CALIB_LOG_DIR, f"session_{ts}.json")
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"[Calib] Session saved → {path}")

print("--- Neurotech: Bionic ROS Launcher (Tasks API) ---")

# Local UDP broadcast to Visualizer
VIZ_ADDR = ("127.0.0.1", 1235)
viz_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# MediaPipe Setup (Tasks API)
# Note: You need pose_landmarker.task and hand_landmarker.task in the same folder
try:
    print("Initializing AI Models...")
    # Pose Detector
    pose_base = python.BaseOptions(model_asset_path='pose_landmarker.task')
    pose_options = vision.PoseLandmarkerOptions(base_options=pose_base, 
                                               running_mode=vision.RunningMode.VIDEO)
    pose_detector = vision.PoseLandmarker.create_from_options(pose_options)
    
    # Hand Detector (for Pinch)
    hand_base = python.BaseOptions(model_asset_path='hand_landmarker.task')
    hand_options = vision.HandLandmarkerOptions(base_options=hand_base, 
                                                running_mode=vision.RunningMode.VIDEO)
    hand_detector = vision.HandLandmarker.create_from_options(hand_options)
    print("AI Models Ready.")
except Exception as e:
    print(f"CRITICAL ERROR: Failed to load models: {e}")
    print("Ensure 'pose_landmarker.task' and 'hand_landmarker.task' are in the project folder.")
    exit(1)

# Trajectory Storage
path_points = deque(maxlen=100)
is_drawing = False

def capture_window_bitblt(hwnd):
    if not hwnd or not win32gui.IsWindow(hwnd): return None
    try:
        left, top, right, bot = win32gui.GetWindowRect(hwnd)
        w, h = right - left, bot - top
        if w <= 0 or h <= 0: return None
        hwndDC = win32gui.GetWindowDC(hwnd)
        mfcDC = win32ui.CreateDCFromHandle(hwndDC)
        saveDC = mfcDC.CreateCompatibleDC()
        saveBitMap = win32ui.CreateBitmap()
        saveBitMap.CreateCompatibleBitmap(mfcDC, w, h)
        saveDC.SelectObject(saveBitMap)
        saveDC.BitBlt((0, 0), (w, h), mfcDC, (0, 0), win32con.SRCCOPY)
        bits = saveBitMap.GetBitmapBits(True)
        saveDC.DeleteDC()
        mfcDC.DeleteDC()
        win32gui.ReleaseDC(hwnd, hwndDC)
        win32gui.DeleteObject(saveBitMap.GetHandle())
        if len(bits) != w * h * 4: return None
        img = np.frombuffer(bits, dtype='uint8').reshape((h, w, 4))
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)[31:-8, 8:-8].copy() if h > 50 else img.copy()
    except: return None

def find_window(substring):
    def callback(hwnd, res):
        if win32gui.IsWindowVisible(hwnd) and substring.lower() in win32gui.GetWindowText(hwnd).lower():
            res.append(hwnd)
    res = []
    win32gui.EnumWindows(callback, res)
    return res[0] if res else None

def draw_3d_path(frame, points):
    if len(points) < 2: return
    for i in range(1, len(points)):
        p1 = points[i-1] # (x, y, z, mode)
        p2 = points[i]
        z_scale = 1 + (p2[2] * 2) 
        thickness = max(1, int(2 * z_scale))
        mode = p2[3]
        
        # Green for FIST (Macro), Red for PINCH (Micro)
        if "FIST" in mode:
            color = (0, 255, int(150 * (1 - p2[2]))) 
        else:
            color = (0, int(150 * (1 - p2[2])), 255)
            
        cv2.line(frame, (p1[0], p1[1]), (p2[0], p2[1]), color, thickness)

def main():
    target_titles = ["ARM_CAM", "NEURO_CAM_SOURCE", "scrcpy"]
    cv2.namedWindow("Neuro-Link: Bionic Control Center", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Neuro-Link: Bionic Control Center", 860, 640)

    # Sensitivity trackbar (1..40 → 0.05..2.00, step 0.05)
    cv2.createTrackbar("Sensitivity x0.05", "Neuro-Link: Bionic Control Center", 10, 40, lambda v: None)

    hwnd = None
    
    # ── Calibration State Machine ──
    # 0=WAITING  1=NOISE(auto 2.5s)  2=RAISE_READY  3=RAISE_REC  4=LOWER_READY  5=LOWER_REC  6=ACTIVE
    NOISE_SECS      = 2.5
    calib_state     = 0
    calib_start_time = None
    noise_samples   = []
    noise_mean      = 0.0
    noise_std       = 2.0    # degrees — updated from noise recording
    base_angle      = None
    max_angle       = None
    min_angle       = None
    calib_range     = 30.0
    up_dir          = 1      # +1 if raising increases angle, -1 if decreasing

    # Stationary detection for delta-based elevation
    stationary_angle  = None
    prev_raw_angle    = None
    still_counter     = 0
    STILL_THRESHOLD   = 2.5
    STILL_FRAMES      = 15

    # Fist/gesture state tracking for jerk-free re-engage
    prev_gesture_active = False
    reengage_warmup = 0
    held_z              = 0.5   # arm_z held between gesture releases
    arm_z_base          = 0.5   # arm_z at moment of gesture activation

    
    frame_timestamp_ms = 0
    arm_z = 0.5  # start strictly at 0.5 (Hover)
    key = 0

    # Camera rotation cycling: O key cycles through 0°, 90°CW, 180°, 90°CCW
    ROT_CODES  = [None, cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_180, cv2.ROTATE_90_COUNTERCLOCKWISE]
    ROT_LABELS = ["0°", "90° CW", "180°", "90° CCW"]
    rotation_idx = 2  # default 180° for upside-down camera

    detect_flash_until = 0  # timestamp until detection banner is shown

    # Sensitivity: scales how much motion maps to drone Z change.
    # 0.5 = arm must travel 2× calibrated range to reach Z limit (easier to control)
    # 1.0 = calibrated range maps exactly to full Z range
    # Tunable live with '[' and ']'
    sensitivity = 0.5

    while True:
        if key == ord('q'): break
        if key == ord('r'):
            path_points.clear()
            calib_state = 0
            calib_start_time = None
            noise_samples = []
            noise_mean = 0.0; noise_std = 2.0
            base_angle = max_angle = min_angle = None
            stationary_angle = None
            prev_raw_angle = None
            still_counter = 0
            prev_gesture_active = False
            held_z = 0.5; arm_z_base = 0.5; arm_z = 0.5
        if key == ord('o'):
            rotation_idx = (rotation_idx + 1) % len(ROT_CODES)
        if key == ord('d'):
            detect_flash_until = time.time() + 2.5
        if key == ord(']'):
            sensitivity = round(min(sensitivity + 0.05, 2.0), 2)
            cv2.setTrackbarPos("Sensitivity x0.05", "Neuro-Link: Bionic Control Center", int(sensitivity / 0.05))
        if key == ord('['):
            sensitivity = round(max(sensitivity - 0.05, 0.05), 2)
            cv2.setTrackbarPos("Sensitivity x0.05", "Neuro-Link: Bionic Control Center", int(sensitivity / 0.05))

        # Always read trackbar as primary source (mouse drag overrides key)
        tb_val = max(1, cv2.getTrackbarPos("Sensitivity x0.05", "Neuro-Link: Bionic Control Center"))
        sensitivity = round(tb_val * 0.05, 2)

        if hwnd is None:
            for t in target_titles:
                hwnd = find_window(t)
                if hwnd: break
            if not hwnd:
                try: viz_sock.sendto(b"0.000,0.000,0.500,RELAXED", VIZ_ADDR)
                except: pass
                splash = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(splash, "WAITING FOR CAMERA...", (180, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 150, 255), 2)
                cv2.putText(splash, "ARM_CAM window not found", (200, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
                cv2.imshow("Neuro-Link: Bionic Control Center", splash)
                key = cv2.waitKey(500) & 0xFF
                continue

        frame = capture_window_bitblt(hwnd)
        if frame is None: hwnd = None; continue
        if ROT_CODES[rotation_idx] is not None:
            frame = cv2.rotate(frame, ROT_CODES[rotation_idx])

        h, w, _ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        frame_timestamp_ms = int(time.time() * 1000)
        
        # 1. Pose Processing — Elbow + Wrist Anchor
        pose_res = pose_detector.detect_for_video(mp_image, frame_timestamp_ms)
        wrist_pos = None
        pose_valid = False
        raw_angle = None

        if pose_res.pose_landmarks:
            lms = pose_res.pose_landmarks[0]
            sh, eb, wr = lms[12], lms[14], lms[16]  # shoulder, elbow, wrist
            sx, sy = int(sh.x * w), int(sh.y * h)
            ex, ey = int(eb.x * w), int(eb.y * h)
            wx, wy = int(wr.x * w), int(wr.y * h)

            if sh.visibility > 0.2 and eb.visibility > 0.2 and wr.visibility > 0.2:
                pose_valid = True
                cv2.line(frame, (sx, sy), (ex, ey), (100, 255, 100), 3)   # bicep
                cv2.line(frame, (ex, ey), (wx, wy), (0, 200, 255), 3)     # forearm
                cv2.circle(frame, (sx, sy), 6, (255, 100,   0), -1)       # shoulder
                cv2.circle(frame, (ex, ey), 9, (  0, 255,   0), -1)       # elbow (joint)
                cv2.circle(frame, (wx, wy), 6, (  0, 100, 255), -1)       # wrist

                # True elbow flexion angle: arccos of the angle at the elbow vertex
                # v1 = elbow→shoulder, v2 = elbow→wrist
                v1x, v1y = sx - ex, sy - ey
                v2x, v2y = wx - ex, wy - ey
                mag = math.sqrt(v1x**2 + v1y**2) * math.sqrt(v2x**2 + v2y**2)
                if mag > 0:
                    raw_angle = math.degrees(math.acos(max(-1.0, min(1.0, (v1x*v2x + v1y*v2y) / mag))))
                wrist_pos = (wx, wy)
        # 2. Hand Processing — needed for Fist-based Calibration
        hand_res = hand_detector.detect_for_video(mp_image, frame_timestamp_ms)
        pinch_active = False
        fist_active = False
        hand_wrist_pos = None
        hand_mcp_pos = None
        hand_angle = None

        if hand_res.hand_landmarks:
            for hlms in hand_res.hand_landmarks:
                # Pinch Detection
                t_tip, i_tip = hlms[4], hlms[8]
                dist = math.hypot(t_tip.x - i_tip.x, t_tip.y - i_tip.y)
                if dist < 0.05:
                    pinch_active = True
                    cv2.circle(frame, (int(i_tip.x*w), int(i_tip.y*h)), 15, (0,0,255), -1)

                # Fist Detection
                wrist = hlms[0]
                hand_wrist_pos = (int(wrist.x*w), int(wrist.y*h))
                cv2.circle(frame, hand_wrist_pos, 7, (0, 200, 255), -1)

                # Forearm angle from hand: wrist(0) → middle-MCP(9) approximates forearm direction
                mcp = hlms[9]
                hand_mcp_pos = (int(mcp.x * w), int(mcp.y * h))
                hand_angle = math.degrees(math.atan2(
                    hand_wrist_pos[1] - hand_mcp_pos[1],
                    hand_mcp_pos[0]   - hand_wrist_pos[0]
                ))

                fingers_closed = 0
                for tip_idx, pip_idx in [(8, 6), (12, 10), (16, 14), (20, 18)]:
                    tip, pip = hlms[tip_idx], hlms[pip_idx]
                    dist_tip = math.hypot(tip.x - wrist.x, tip.y - wrist.y)
                    dist_pip = math.hypot(pip.x - wrist.x, pip.y - wrist.y)
                    if dist_tip < dist_pip:
                        fingers_closed += 1
                
                if fingers_closed >= 3 and not pinch_active:
                    fist_active = True
                    # Only draw FIST label if we are active (not calibrating)
                    if calib_state == 6:
                        cv2.putText(frame, "FIST", hand_wrist_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3)

        if not pose_valid and hand_wrist_pos is not None and hand_angle is not None:
            # Use hand orientation (wrist→MCP) as forearm angle substitute
            wrist_pos = hand_wrist_pos
            raw_angle = hand_angle
            pose_valid = True
            cv2.line(frame, hand_wrist_pos, hand_mcp_pos, (0, 200, 255), 3)
            cv2.putText(frame, "HAND MODE", (10, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

        # Detection status banner (triggered by 'd' key)
        if time.time() < detect_flash_until:
            det_color = (0, 180, 60) if pose_valid else (30, 30, 200)
            det_text  = "ARM DETECTED" if pose_valid else "ARM NOT DETECTED  —  try pressing O to rotate"
            cv2.rectangle(frame, (10, h - 65), (w - 10, h - 18), det_color, -1)
            cv2.putText(frame, det_text, (22, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # ── Calibration Logic ──
        if key in [ord('c'), ord('C')]:
            if not pose_valid or raw_angle is None:
                cv2.putText(frame, "! ARM NOT DETECTED — SHOW ARM TO CAMERA !", (20, h//2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            else:
                if calib_state == 0:
                    # Start timed noise recording
                    noise_samples = []
                    calib_state = 1
                    calib_start_time = time.time()
                elif calib_state == 2:
                    # Start raise recording
                    max_angle = raw_angle
                    calib_state = 3
                elif calib_state == 3:
                    # Stop raise recording
                    calib_state = 4
                elif calib_state == 4:
                    # Start lower recording
                    min_angle = raw_angle
                    calib_state = 5
                elif calib_state == 5:
                    # Stop lower recording → go ACTIVE
                    calib_range = abs(max_angle - min_angle)
                    if calib_range < 8: calib_range = 8
                    up_dir = 1 if max_angle > base_angle else -1
                    calib_state = 6
                    stationary_angle = raw_angle
                    # ── Save this session to disk for review ──
                    save_calib_session({
                        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                        "noise_mean_deg": round(noise_mean, 2),
                        "noise_std_deg":  round(noise_std,  2),
                        "base_angle_deg": round(base_angle, 2),
                        "max_angle_deg":  round(max_angle,  2),
                        "min_angle_deg":  round(min_angle,  2),
                        "calib_range_deg": round(calib_range, 2),
                        "up_dir": up_dir,
                        "sensitivity": sensitivity,
                        "rotation_label": ROT_LABELS[rotation_idx],
                    })

        # Auto-advance noise recording when timer expires
        if calib_state == 1 and calib_start_time is not None:
            if pose_valid and raw_angle is not None:
                noise_samples.append(raw_angle)
            if time.time() - calib_start_time >= NOISE_SECS:
                if noise_samples:
                    noise_mean = float(np.mean(noise_samples))
                    noise_std  = max(float(np.std(noise_samples)), 1.0)
                base_angle = noise_mean
                max_angle  = noise_mean
                min_angle  = noise_mean
                calib_state = 2

        # Continuous angle tracking during recording phases
        if pose_valid and raw_angle is not None:
            if calib_state == 3:
                # Raise: track furthest deviation from base in any direction
                if abs(raw_angle - base_angle) > abs(max_angle - base_angle):
                    max_angle = raw_angle
            elif calib_state == 5:
                # Lower: track furthest deviation in OPPOSITE direction to raise
                if abs(raw_angle - base_angle) > abs(min_angle - base_angle):
                    if (raw_angle - base_angle) * (max_angle - base_angle) <= 0:
                        min_angle = raw_angle

        # ── Stationary Detection (Active Mode, only when not gesturing) ──
        gesture_active = fist_active or pinch_active
        if calib_state == 6 and pose_valid and raw_angle is not None:
            if prev_raw_angle is not None:
                if abs(raw_angle - prev_raw_angle) < STILL_THRESHOLD:
                    still_counter = min(still_counter + 1, STILL_FRAMES * 3)
                else:
                    still_counter = max(still_counter - 3, 0)
                # Only drift reference when arm is still and NOT gesturing
                if still_counter >= STILL_FRAMES and stationary_angle is not None and not gesture_active:
                    stationary_angle = 0.97 * stationary_angle + 0.03 * raw_angle
            prev_raw_angle = raw_angle

        # ── Z-Elevation Mapping: jerk-free, noise-gated ──
        if calib_state == 6 and pose_valid and raw_angle is not None:
            gesture_just_on  = gesture_active and not prev_gesture_active
            gesture_just_off = not gesture_active and prev_gesture_active

            if gesture_just_on:
                # Snap angle reference to current position → zero jerk on re-engage
                stationary_angle = raw_angle
                arm_z_base = arm_z         # continue elevation from exactly where it is
                reengage_warmup = 5        # block movement for 5 frames while arm settles
            elif gesture_just_off:
                held_z = arm_z             # freeze elevation on release
                reengage_warmup = 0

            if gesture_active and stationary_angle is not None:
                if reengage_warmup > 0:
                    # Warm-up: re-snap reference each frame so the first real frame has deviation≈0
                    stationary_angle = raw_angle
                    arm_z_base = arm_z
                    reengage_warmup -= 1
                else:
                    deviation = (raw_angle - stationary_angle) * up_dir
                    # Noise gate: ignore micro-jitter within recorded noise floor
                    if abs(deviation) > noise_std * 1.5:
                        effective_range = calib_range / sensitivity  # larger range = less sensitive
                        target_z = np.clip(arm_z_base + deviation / effective_range, 0.0, 1.0)
                        arm_z = 0.06 * target_z + 0.94 * arm_z   # gentle smooth — less twitchy

            prev_gesture_active = gesture_active

        # 3. Path Recording & Modes
        active_mode = "RELAXED"
        if calib_state < 6:
            active_mode = "CALIBRATING"
        else:
            if fist_active: active_mode = "FIST (MACRO)"
            elif pinch_active: active_mode = "PINCH (MICRO)"

            if (pinch_active or fist_active) and wrist_pos:
                path_points.append((wrist_pos[0], wrist_pos[1], arm_z, active_mode))
            
            # Broadcast to 3D Visualizer (always stream current known state)
            if wrist_pos:
                mapped_z = arm_z - 0.5   # -0.5..+0.5
                # Locked to elevation only — X/Y hardcoded 0 until XY control is wired
                pkt = f"0.000,0.000,{mapped_z:.3f},{active_mode}"
                try: viz_sock.sendto(pkt.encode(), VIZ_ADDR)
                except: pass
        
        draw_3d_path(frame, path_points)

        # HUD
        hud_h = 160 if calib_state < 6 else 130
        cv2.rectangle(frame, (10, 10), (530, hud_h), (0,0,0), -1)
        ang_now = f"{raw_angle:.1f}°" if raw_angle is not None else "no signal"

        if calib_state == 0:
            cv2.putText(frame, "HOLD ARM NATURAL  ->  PRESS 'C'", (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
            cv2.putText(frame, f"Elbow angle now: {ang_now}  (noise profile will auto-record 2.5s)", (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (160, 160, 160), 1)
        elif calib_state == 1:
            elapsed = time.time() - calib_start_time if calib_start_time else 0
            pct = min(elapsed / NOISE_SECS, 1.0)
            cv2.putText(frame, "RECORDING NOISE PROFILE — HOLD STILL", (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 220, 100), 2)
            cv2.putText(frame, f"angle: {ang_now}   samples: {len(noise_samples)}", (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
            bar_x, bar_y, bar_w, bar_h = 20, 85, 440, 11
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x+bar_w, bar_y+bar_h), (40,40,40), -1)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x+int(bar_w*pct), bar_y+bar_h), (0,200,100), -1)
        elif calib_state == 2:
            cv2.putText(frame, "RAISE READY  ->  PRESS 'C' TO START", (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            cv2.putText(frame, f"Noise baseline: {noise_mean:.1f}° ± {noise_std:.1f}°   now: {ang_now}", (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        elif calib_state == 3:
            cv2.putText(frame, "RAISE ARM UP  ->  PRESS 'C' TO STOP", (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            peak = f"{max_angle:.1f}" if max_angle is not None else "--"
            dev  = abs(max_angle - base_angle) if max_angle is not None and base_angle is not None else 0
            cv2.putText(frame, f"now: {ang_now}   peak: {peak}°   dev: {dev:.1f}°", (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        elif calib_state == 4:
            cv2.putText(frame, "LOWER READY  ->  PRESS 'C' TO START", (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
            cv2.putText(frame, f"Raise locked: {max_angle:.1f}°   now: {ang_now}", (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        elif calib_state == 5:
            cv2.putText(frame, "LOWER ARM DOWN  ->  PRESS 'C' TO STOP", (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 165, 255), 2)
            trough = f"{min_angle:.1f}" if min_angle is not None else "--"
            dev    = abs(min_angle - base_angle) if min_angle is not None and base_angle is not None else 0
            cv2.putText(frame, f"now: {ang_now}   trough: {trough}°   dev: {dev:.1f}°", (20, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        else:
            cv2.putText(frame, "BIONIC NODE: ACTIVE", (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            still_str = "STILL" if still_counter >= STILL_FRAMES else "moving"
            stat_str  = f"{stationary_angle:.1f}" if stationary_angle is not None else "--"
            cv2.putText(frame, f"Z:{arm_z:.2f}  elbow:{ang_now}  ref:{stat_str}  [{still_str}]", (20, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 200, 200), 1)
            cv2.putText(frame, f"Sensitivity: {sensitivity:.2f}  ( [ to decrease  ] to increase )",
                        (20, 108), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 180, 80), 1)

        if calib_state < 6:
            cv2.putText(frame, f"O: rotate [{ROT_LABELS[rotation_idx]}]   D: detect   R: reset",
                        (20, 148), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)

        color = (255, 255, 255)
        if "FIST" in active_mode: color = (0, 255, 0)
        elif "PINCH" in active_mode: color = (0, 0, 255)
        elif "CALIB" in active_mode: color = (0, 165, 255)
        
        cv2.putText(frame, active_mode, (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        cv2.imshow("Neuro-Link: Bionic Control Center", frame)
        key = cv2.waitKey(5) & 0xFF

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
