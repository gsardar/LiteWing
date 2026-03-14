"""
drone_direct/test_camera.py
============================
Tests phone camera + ArUco detection ONLY — no drone control.
Uses win32gui window capture from scrcpy (works on Windows with scrcpy 3.x).

Run:
  cd g:/Projects/Hackathon/Neurotech
  .venv/Scripts/python.exe drone_direct/test_camera.py

Controls (in the preview window):
  Q — quit
"""

import subprocess
import sys
import os
import time
import math
import cv2
import numpy as np
import win32gui
import win32ui
import win32con

SCRCPY_EXE   = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'scrcpy',
                 'scrcpy-win64-v3.0.2', 'scrcpy.exe'))
WINDOW_TITLE = 'ARUCO_CAM_TEST'
MARKER_ID    = 7
MARKER_SIZE  = 0.08   # metres (8cm)


def grab_window(hwnd):
    """Capture a win32 window to a BGR numpy array."""
    try:
        left, top, right, bot = win32gui.GetClientRect(hwnd)
        w = right - left
        h = bot - top
        if w <= 0 or h <= 0:
            return None
        hwnd_dc   = win32gui.GetWindowDC(hwnd)
        mfc_dc    = win32ui.CreateDCFromHandle(hwnd_dc)
        save_dc   = mfc_dc.CreateCompatibleDC()
        bmp       = win32ui.CreateBitmap()
        bmp.CreateCompatibleBitmap(mfc_dc, w, h)
        save_dc.SelectObject(bmp)
        save_dc.BitBlt((0, 0), (w, h), mfc_dc, (0, 0), win32con.SRCCOPY)
        bmp_info  = bmp.GetInfo()
        raw       = bmp.GetBitmapBits(True)
        img       = np.frombuffer(raw, dtype=np.uint8).reshape(
                        bmp_info['bmHeight'], bmp_info['bmWidth'], 4)
        frame     = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        win32gui.DeleteObject(bmp.GetHandle())
        save_dc.DeleteDC()
        mfc_dc.DeleteDC()
        win32gui.ReleaseDC(hwnd, hwnd_dc)
        return frame
    except Exception:
        return None


def find_window(title):
    hwnd = win32gui.FindWindow(None, title)
    return hwnd if hwnd else None


def main():
    print(f"Launching scrcpy camera window …")
    proc = subprocess.Popen([
        SCRCPY_EXE,
        '--video-source=camera',
        '--camera-facing=back',
        '--no-audio',
        f'--window-title={WINDOW_TITLE}',
        '--max-fps=30',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Wait for window to appear
    hwnd = None
    for _ in range(30):
        hwnd = find_window(WINDOW_TITLE)
        if hwnd:
            break
        time.sleep(0.5)

    if not hwnd:
        print("ERROR: scrcpy window did not appear.")
        print("  → Check phone USB connection and tap Allow on phone screen.")
        proc.terminate()
        sys.exit(1)

    print(f"Camera window found (hwnd={hwnd})")
    print("Point camera at ArUco ID 7 marker — press Q to quit\n")

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
        detector = None

    clahe       = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    cam_matrix  = None
    dist_coeffs = np.zeros((4, 1), dtype=np.float32)

    while True:
        frame = grab_window(hwnd)
        if frame is None:
            hwnd = find_window(WINDOW_TITLE)
            time.sleep(0.05)
            continue

        frame = cv2.resize(frame, (640, 480))
        fh, fw = frame.shape[:2]

        if cam_matrix is None:
            cam_matrix = np.array([[fw, 0, fw/2],
                                   [0, fw, fh/2],
                                   [0,  0,    1]], dtype=np.float32)

        gray = clahe.apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        if detector:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, aruco_dict, parameters=aruco_params)

        status = "SEARCHING for ArUco ID 7 ..."
        color  = (0, 100, 255)

        if ids is not None:
            valid = [i for i, mid in enumerate(ids.flatten()) if mid == MARKER_ID]
            if valid:
                c    = corners[valid[0]][0]
                diag = math.hypot(c[2][0]-c[0][0], c[2][1]-c[0][1])
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))
                cv2.drawMarker(frame, (cx, cy), (0, 255, 0),
                               cv2.MARKER_CROSS, 30, 3)

                if diag >= 20:
                    half    = MARKER_SIZE / 2.0
                    obj_pts = np.array([[-half, half, 0], [half, half, 0],
                                        [half, -half, 0], [-half, -half, 0]],
                                       dtype=np.float32)
                    _, _, tvec = cv2.solvePnP(
                        obj_pts, c, cam_matrix, dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    z = float(tvec.flatten()[2])
                    status = f"DETECTED  Z={z:.3f}m  diag={diag:.0f}px"
                    color  = (0, 255, 0)
                    cv2.putText(frame, f"Z = {z:.3f} m", (cx - 60, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                else:
                    status = f"TOO CLOSE (diag={diag:.0f}px)"
                    color  = (0, 200, 255)

        cv2.rectangle(frame, (0, fh - 36), (fw, fh), (20, 20, 20), -1)
        cv2.putText(frame, status, (10, fh - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        cv2.imshow("ArUco Camera Test", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    proc.terminate()
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == '__main__':
    main()
