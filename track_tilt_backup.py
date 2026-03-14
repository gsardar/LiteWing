import cv2
import numpy as np
import win32gui
import win32ui
import win32con
import time
import math
import threading

print("--- Neurotech Drone Ground Station: FOS Replacement ---")

# Global Detector (Loaded in thread to keep UI responsive)
detector_hands = None
is_loading = True

def load_ai():
    global detector_hands, is_loading
    try:
        import mediapipe as mp
        from mediapipe.tasks import python
        from mediapipe.tasks.python import vision
        base_options = python.BaseOptions(model_asset_path='hand_landmarker.task')
        options = vision.HandLandmarkerOptions(base_options=base_options,
                                               num_hands=1,
                                               min_hand_detection_confidence=0.7)
        detector_hands = vision.HandLandmarker.create_from_options(options)
        print("Secondary AI (Hand) Loaded.")
    except Exception:
        pass # Hand tracking is optional for this drone use case
    finally:
        is_loading = False

# Start loading in background
threading.Thread(target=load_ai, daemon=True).start()

# Initialize ArUco (Default for Drone)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()
try:
    detector_aruco = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    has_aruco = True
except AttributeError:
    has_aruco = False

# Modes: 0=ARUCO (Default), 1=HAND, 2=COLOR
modes = ["ARUCO MARKER", "HAND LANDMARKS", "COLOR BLOB"]
current_mode = 0
baseline_size = None
baseline_angle = None

def find_window(substrings):
    def callback(hwnd, windows):
        if win32gui.IsWindowVisible(hwnd):
            t = win32gui.GetWindowText(hwnd).lower()
            for s in substrings:
                if s.lower() in t: windows.append(hwnd); return
    res = []
    win32gui.EnumWindows(callback, res)
    return res[0] if res else None

def capture_frame(hwnd):
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
        frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return frame[31:-8, 8:-8] if h > 50 else frame
    except: return None

def main():
    global current_mode, baseline_size, baseline_angle
    source_titles = ["neuro", "scrcpy", "cam"]
    cv2.namedWindow("Zero-Latency AI Tracker", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Zero-Latency AI Tracker", 640, 480)
    cv2.setWindowProperty("Zero-Latency AI Tracker", cv2.WND_PROP_TOPMOST, 1)

    hwnd = None
    last_process = 0

    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key in [ord('1'), ord('2'), ord('3')]:
            current_mode = int(chr(key)) - 1
            baseline_size = baseline_angle = None
            print(f"Mode set to: {modes[current_mode]}")

        # Sync Window with Camera Engine
        if hwnd is None:
            hwnd = find_window(source_titles)
            if not hwnd:
                empty = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(empty, "CONNECTING TO DRONE CAMERA...", (120, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,150,255), 2)
                cv2.imshow("Zero-Latency AI Tracker", empty)
                continue

        # Capture Frame
        frame = capture_frame(hwnd)
        if frame is None:
            hwnd = None
            continue

        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2
        
        # Draw Center Crosshair
        cv2.drawMarker(frame, (center_x, center_y), (100, 100, 100), markerType=cv2.MARKER_CROSS, markerSize=40, thickness=2)

        cur_pos = None # (x, y, size, angle)
        
        # AI Processing (Throttled for stability)
        if time.time() - last_process > 0.033:
            last_process = time.time()
            
            # MODE 0: ARUCO (Drone Vertical)
            if current_mode == 0 and has_aruco:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector_aruco.detectMarkers(gray)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    c = corners[0][0]
                    # Calculate center of the marker
                    mx = int(np.mean(c[:, 0]))
                    my = int(np.mean(c[:, 1]))
                    # Size (Diagonal) for Z-Elevation
                    m_size = math.hypot(c[2][0]-c[0][0], c[2][1]-c[0][1])
                    # Angle (Orientation)
                    m_angle = math.degrees(math.atan2(c[1][1]-c[0][1], c[1][0]-c[0][0]))
                    cur_pos = (mx, my, m_size, m_angle)
                    cv2.circle(frame, (mx, my), 10, (0, 255, 0), -1)

            # MODE 1: HAND (Optional)
            elif current_mode == 1 and detector_hands:
                proc_img = cv2.resize(frame, (0,0), fx=0.5, fy=0.5)
                rgb = cv2.cvtColor(proc_img, cv2.COLOR_BGR2RGB)
                import mediapipe as mp
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                results = detector_hands.detect(mp_img)
                if results.hand_landmarks:
                    lms = results.hand_landmarks[0]
                    p1 = (int(lms[0].x * w), int(lms[0].y * h))
                    p2 = (int(lms[9].x * w), int(lms[9].y * h))
                    cur_pos = (int((p1[0]+p2[0])/2), int((p1[1]+p2[1])/2), math.hypot(p2[0]-p1[0], p2[1]-p1[1]), math.degrees(math.atan2(p2[1]-p1[1], p2[0]-p1[0])))
                    for lm in lms: cv2.circle(frame, (int(lm.x*w), int(lm.y*h)), 4, (0,255,0), -1)

            # MODE 2: COLOR (Drone Body)
            elif current_mode == 2:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                m = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)) + cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))
                cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if cnts:
                    l = max(cnts, key=cv2.contourArea)
                    if cv2.contourArea(l) > 500:
                        r = cv2.minAreaRect(l)
                        box = np.int0(cv2.boxPoints(r))
                        cv2.drawContours(frame, [box], 0, (0,0,255), 2)
                        cur_pos = (int(r[0][0]), int(r[0][1]), max(r[1]), r[2])

        # Calibration
        if key == ord('c') and cur_pos:
            baseline_size, baseline_angle = cur_pos[2], cur_pos[3]
            print("Baseline Calibrated (Drone on Ground).")

        # Ground Station Output
        if cur_pos:
            mx, my, ms, ma = cur_pos
            # 1. Delta X/Y from Camera Center (Normalized -1 to 1)
            dx = (mx - center_x) / (w / 2)
            dy = (my - center_y) / (h / 2)
            
            # 2. Elevation (Z) relative to baseline
            elevation = ms / baseline_size if baseline_size else 1.0
            tilt = ma - baseline_angle if baseline_angle else 0.0

            # Draw Drone Position Vector
            cv2.line(frame, (center_x, center_y), (mx, my), (0, 255, 255), 2)

            # HUD Rendering
            cv2.rectangle(frame, (10, 10), (350, 160), (0,0,0), -1)
            cv2.putText(frame, f"MODE: {modes[current_mode]}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"X DISP: {dx:+.2f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"Y DISP: {dy:+.2f}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"Z ELEV: {elevation:.2f}x", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cv2.putText(frame, f"TILT  : {tilt:.1f} DEG", (200, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        elif not baseline_size:
            cv2.putText(frame, "CENTER DRONE AND PRESS 'C' TO CALIBRATE", (10, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        # Scale for Ground Station Display
        disp = cv2.resize(frame, (640, 480))
        cv2.imshow("Zero-Latency AI Tracker", disp)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
