"""
Neurotech - Phone Autonomous Drone Controller
Runs entirely on Android via Termux.

Requirements (run in Termux):
  pkg install python libopencv
  pip install numpy

Setup:
  1. Start the IP Webcam app (Neurotech_Camera_AdFree.apk) on this phone
     - It will serve the camera at http://localhost:8080
  2. Connect phone WiFi to drone (ESP-DRONE_xxx, password: 12345678)
  3. Run this script: python phone_controller.py

Controls (keyboard in Termux):
  Enter  = Toggle arm/disarm motors
  Ctrl+C = Emergency stop
"""

import cv2
import numpy as np
import socket
import struct
import time
import threading
import sys

# --- Configuration ---
CAMERA_URL   = "http://localhost:8080/video"   # IP Webcam app
DRONE_IP     = "192.168.43.42"
DRONE_PORT   = 2390

# PID tuning (start conservative, increase if drone response is too slow)
KP           = 8.0    # How aggressively to correct position (degrees per unit error)
MAX_TILT     = 10.0   # Max roll/pitch tilt in degrees

BASE_THRUST  = 38000  # Hover thrust — tune this first!
THRUST_KP    = 3000   # How much to adjust thrust based on marker size

# ArUco marker dict (must match your printed marker)
ARUCO_DICT   = cv2.aruco.DICT_6X6_250


# --- UDP socket for drone commands ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.1)

armed     = False
running   = True
base_thrust = BASE_THRUST


def cksum(data: bytes) -> int:
    return sum(data) & 0xFF


def send_setpoint(roll: float, pitch: float, yaw: float, thrust: int):
    """Send a raw CRTP RPYT setpoint packet to the drone."""
    header  = (3 << 4) | 0   # Port 3, Channel 0
    payload = struct.pack('<BfffH', header, roll, pitch, yaw, max(0, min(65535, int(thrust))))
    sock.sendto(payload + bytes([cksum(payload)]), (DRONE_IP, DRONE_PORT))


def command_thread():
    """Background thread: listen for Enter key to arm/disarm."""
    global armed, running, base_thrust
    print("\n  Press ENTER to ARM/DISARM. Ctrl+C = Emergency Stop.\n")
    while running:
        try:
            input()
            armed = not armed
            if armed:
                base_thrust = BASE_THRUST
                print(f"[✓] ARMED  — Thrust base: {base_thrust}")
            else:
                print("[✓] DISARMED")
        except (EOFError, KeyboardInterrupt):
            break


# --- ArUco Setup ---
aruco_dict   = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters()
try:
    detector  = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    use_new   = True
except AttributeError:
    use_new   = False   # Older OpenCV

baseline_size = None    # Marker size at "hover height" — set when armed


def detect_aruco(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if use_new:
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if ids is not None and len(corners) > 0:
        c  = corners[0][0]
        mx = float(np.mean(c[:, 0]))
        my = float(np.mean(c[:, 1]))
        sz = float(np.linalg.norm(c[2] - c[0]))   # diagonal = size proxy
        return mx, my, sz
    return None


def main():
    global armed, running, baseline_size, base_thrust

    print("=" * 50)
    print("  Neurotech Phone Drone Controller")
    print("=" * 50)
    print(f"  Camera  : {CAMERA_URL}")
    print(f"  Drone   : {DRONE_IP}:{DRONE_PORT}")
    print(f"  Thrust  : {BASE_THRUST}")
    print()

    # Start keyboard thread
    t = threading.Thread(target=command_thread, daemon=True)
    t.start()

    # Open camera
    print("[*] Connecting to IP Webcam...")
    cap = cv2.VideoCapture(CAMERA_URL)
    if not cap.isOpened():
        print("[✗] Cannot open camera stream!")
        print("    Make sure IP Webcam app is running on this phone.")
        sys.exit(1)
    print("[✓] Camera connected!\n")

    last_cmd  = time.time()
    last_seen = time.time()

    try:
        while running:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("[!] Camera frame lost, retrying...")
                time.sleep(0.1)
                continue

            h, w = frame.shape[:2]
            cx, cy = w / 2, h / 2

            result = detect_aruco(frame)

            if armed:
                if result:
                    mx, my, sz = result
                    last_seen = time.time()

                    # Set baseline on first detection after arming
                    if baseline_size is None:
                        baseline_size = sz
                        print(f"[*] Baseline size set: {sz:.1f}px (place drone at hover height first!)")

                    # Normalized error (-1.0 to 1.0)
                    dx = (mx - cx) / (w / 2)
                    dy = (my - cy) / (h / 2)

                    # P-controller for roll & pitch
                    cmd_roll  = -dx * KP
                    cmd_pitch =  dy * KP
                    cmd_roll  = max(-MAX_TILT, min(MAX_TILT, cmd_roll))
                    cmd_pitch = max(-MAX_TILT, min(MAX_TILT, cmd_pitch))

                    # Altitude: adjust thrust based on marker size relative to baseline
                    if baseline_size:
                        size_ratio = sz / baseline_size
                        thrust = base_thrust - int((size_ratio - 1.0) * THRUST_KP)
                        thrust = max(10000, min(65000, thrust))
                    else:
                        thrust = base_thrust

                    send_setpoint(cmd_roll, cmd_pitch, 0, thrust)

                    print(f"  dx={dx:+.2f}  dy={dy:+.2f}  roll={cmd_roll:+.1f}  pitch={cmd_pitch:+.1f}  thr={thrust}  sz={sz:.0f}", end='\r')

                else:
                    # Marker lost — hover in place (neutral commands)
                    if time.time() - last_seen > 1.5:
                        print("\n[!] Marker lost for >1.5s — DISARMING for safety!")
                        armed = False
                        baseline_size = None
                    else:
                        send_setpoint(0, 0, 0, base_thrust)
            else:
                # Disarmed — send zero thrust
                send_setpoint(0, 0, 0, 0)

            time.sleep(0.02)  # ~50 Hz

    except KeyboardInterrupt:
        print("\n[!] Emergency stop!")
    finally:
        running = False
        print("[*] Sending zero thrust...")
        for _ in range(20):
            send_setpoint(0, 0, 0, 0)
            time.sleep(0.02)
        cap.release()
        sock.close()
        print("[✓] Stopped safely.")


if __name__ == "__main__":
    main()
