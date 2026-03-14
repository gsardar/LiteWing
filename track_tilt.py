"""
Neurotech Ground Station — Operator Terminal
Phone (phone_balance.py) owns all flight control: ArUco, PIDs, drone comms.
Laptop owns: keyboard commands, HUD display, flashlight.

Keys:
  ESC          = Emergency stop (cuts motors immediately)
  R            = Reconnect to phone relay
  SPACE        = Arm / Disarm
  H            = Toggle hover-only (altitude hold, no lateral)
  Arrow UP/DN  = Thrust +/- 1500
  +/-          = Thrust +/- 5000
  F            = Flashlight toggle
  Q (window)   = Quit

Setup:
  adb forward tcp:9391 tcp:9391
  python track_tilt.py
"""

import cv2
import numpy as np
import win32gui
import win32ui
import win32con
import time
import threading
import keyboard
import socket
import struct
import subprocess
import os

# ── Protocol: Laptop → Phone  (4 bytes) ──────────────────────────────────────
# B  flags        bit0=armed  bit1=emergency  bit2=hover_only
# h  thrust_delta signed int16 — cumulative nudge from arrow keys
# B  checksum
CMD_FMT  = '<Bh'
CMD_BODY = struct.calcsize(CMD_FMT)   # 3 bytes
CMD_FULL = CMD_BODY + 1               # 4 bytes

# ── Protocol: Phone → Laptop  (17 bytes) ─────────────────────────────────────
# f  x_m          lateral offset metres
# f  y_m          forward offset metres
# f  z_m          depth metres  (-1 = not detected)
# I  base_thrust  current thrust value on phone
# B  checksum
POS_FMT  = '<fffI'
POS_BODY = struct.calcsize(POS_FMT)   # 16 bytes
POS_FULL = POS_BODY + 1               # 17 bytes

FLAG_ARMED     = 0x01
FLAG_EMERGENCY = 0x02
FLAG_HOVER     = 0x04

def cksum(data: bytes) -> int:
    return sum(data) & 0xFF

# ── Shared state ──────────────────────────────────────────────────────────────
flashlight_on   = False
sock            = None
drone_connected = False
EMERGENCY       = False
taking_off      = False
hover_only      = False
thrust_delta    = 0       # cumulative thrust nudge sent to phone
phone_pos       = None    # (x_m, y_m, z_m, base_thrust) latest from phone
phone_pos_lock  = threading.Lock()

# ── Flashlight ────────────────────────────────────────────────────────────────
def toggle_flashlight():
    global flashlight_on
    flashlight_on = not flashlight_on
    state = "on" if flashlight_on else "off"
    try:
        adb = os.path.join(os.getcwd(), "platform-tools", "adb.exe")
        if not os.path.exists(adb):
            adb = "adb"
        subprocess.run([adb, "shell", "cmd", "torch", state],
                       capture_output=True, check=False)
        print(f"[*] Flashlight: {state.upper()}")
    except Exception as e:
        print(f"[!] Flashlight failed: {e}")

# ── Send command to phone ─────────────────────────────────────────────────────
def send_cmd(armed=False, emergency=False):
    global thrust_delta
    try:
        if sock and drone_connected:
            flags = ((FLAG_ARMED     if armed     else 0) |
                     (FLAG_EMERGENCY if emergency else 0) |
                     (FLAG_HOVER     if hover_only else 0))
            body = struct.pack(CMD_FMT, flags, max(-32000, min(32000, thrust_delta)))
            sock.sendall(body + bytes([cksum(body)]))
    except Exception:
        pass

# ── Receive position reports from phone ───────────────────────────────────────
def _recv_loop():
    global phone_pos
    buf = b''
    while True:
        try:
            if not sock:
                time.sleep(0.05)
                continue
            chunk = sock.recv(256)
            if not chunk:
                time.sleep(0.05)
                continue
            buf += chunk
            while len(buf) >= POS_FULL:
                msg  = buf[:POS_FULL]
                buf  = buf[POS_FULL:]
                body = msg[:POS_BODY]
                if cksum(body) != msg[POS_BODY]:
                    continue
                x_m, y_m, z_m, bt = struct.unpack(POS_FMT, body)
                with phone_pos_lock:
                    phone_pos = (x_m, y_m, z_m, bt)
        except Exception:
            time.sleep(0.05)

threading.Thread(target=_recv_loop, daemon=True).start()

# ── Phone connection ──────────────────────────────────────────────────────────
def init_drone():
    global sock, drone_connected
    print("[*] Connecting to phone (127.0.0.1:9391)...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('127.0.0.1', 9391))
        drone_connected = True
        print("[*] Phone relay CONNECTED.")
        # Send idle packets so phone knows laptop is alive
        for _ in range(20):
            send_cmd(armed=False)
            time.sleep(0.1)
    except Exception as e:
        print(f"[!] Connection failed: {e}")
        print("    Run: adb forward tcp:9391 tcp:9391")

threading.Thread(target=init_drone, daemon=True).start()

def on_reconnect():
    global sock, drone_connected
    print("[*] Reconnecting...")
    drone_connected = False
    if sock:
        try: sock.close()
        except Exception: pass
        sock = None
    threading.Thread(target=init_drone, daemon=True).start()

def on_emergency():
    global EMERGENCY, taking_off, thrust_delta
    print("\n[!!! EMERGENCY STOP !!!]")
    EMERGENCY    = True
    taking_off   = False
    thrust_delta = 0
    send_cmd(armed=False, emergency=True)

keyboard.add_hotkey('esc', on_emergency)
keyboard.add_hotkey('r',   on_reconnect)

# ── Window capture (scrcpy HUD display) ───────────────────────────────────────
def find_window(titles):
    res = []
    def cb(hwnd, _):
        if win32gui.IsWindowVisible(hwnd):
            t = win32gui.GetWindowText(hwnd).lower()
            for s in titles:
                if s in t:
                    res.append(hwnd)
                    return
    win32gui.EnumWindows(cb, None)
    return res[0] if res else None

def capture_frame(hwnd):
    if not hwnd or not win32gui.IsWindow(hwnd):
        return None
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
        sDC.BitBlt((0,0),(w,h), mDC,(0,0), win32con.SRCCOPY)
        bits = bmp.GetBitmapBits(True)
        sDC.DeleteDC(); mDC.DeleteDC()
        win32gui.ReleaseDC(hwnd, hDC)
        win32gui.DeleteObject(bmp.GetHandle())
        if len(bits) != w * h * 4:
            return None
        img   = np.frombuffer(bits, dtype='uint8').reshape((h, w, 4))
        frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return frame[31:-8, 8:-8] if h > 50 else frame
    except Exception:
        return None

# ── HUD ───────────────────────────────────────────────────────────────────────
def draw_status(frame, armed, bt):
    fh, fw = frame.shape[:2]
    col  = (0, 255, 0) if drone_connected else (0, 0, 255)
    conn = "CONNECTED" if drone_connected else "DISCONNECTED"
    arm  = f"ARM {int(bt)}" if armed else f"IDLE {int(bt)}"
    arm_col = (0, 165, 255) if armed else (140, 140, 140)
    cv2.rectangle(frame, (8, fh-62), (fw-8, fh-4), (0,0,0), -1)
    cv2.putText(frame, f"DRONE: {conn}", (18, fh-40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
    cv2.putText(frame, arm, (18, fh-16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, arm_col, 2)
    f_col = (0,255,255) if flashlight_on else (100,100,100)
    cv2.putText(frame, "TORCH: ON [F]" if flashlight_on else "TORCH: OFF [F]",
                (fw-180, fh-16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, f_col, 2)

def draw_emergency(frame):
    fh, fw = frame.shape[:2]
    cv2.rectangle(frame, (0,0), (fw,fh), (0,0,200), 6)
    cv2.putText(frame, "!!! EMERGENCY STOP !!!", (fw//2-200, fh//2),
                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,0,255), 3)
    cv2.putText(frame, "Press R to reset", (fw//2-120, fh//2+50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100,100,255), 2)

# ── Main loop ─────────────────────────────────────────────────────────────────
def main():
    global taking_off, EMERGENCY, hover_only, thrust_delta, drone_connected

    THRUST_STEP = 1500
    ARROW_UP    = 2490368
    ARROW_DOWN  = 2621440

    source_titles = ["neuro", "scrcpy", "cam"]
    cv2.namedWindow("Neurotech Ground Station", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Neurotech Ground Station", 640, 480)
    cv2.setWindowProperty("Neurotech Ground Station", cv2.WND_PROP_TOPMOST, 1)

    hwnd       = None
    last_arm_t = 0.0
    last_send  = 0.0

    while True:
        raw_key  = cv2.waitKey(1)
        key_char = raw_key & 0xFF

        # ── Emergency screen ───────────────────────────────────────────────
        if EMERGENCY:
            empty = np.zeros((480, 640, 3), dtype=np.uint8)
            draw_emergency(empty)
            cv2.imshow("Neurotech Ground Station", empty)
            if key_char == ord('q'):
                break
            if key_char == ord('r'):
                EMERGENCY    = False
                taking_off   = False
                thrust_delta = 0
                on_reconnect()
            continue

        # ── Keys ───────────────────────────────────────────────────────────
        if key_char == ord('q'):
            break
        if key_char == ord('f'):
            toggle_flashlight()
        if key_char == ord('h'):
            hover_only = not hover_only
            print(f"[*] Hover-only: {'ON' if hover_only else 'OFF'}")

        if key_char == ord(' ') and drone_connected:
            if time.time() - last_arm_t > 0.5:
                last_arm_t = time.time()
                taking_off = not taking_off
                if not taking_off:
                    thrust_delta = 0
                print(f"[!] {'ARMED' if taking_off else 'DISARMED'}")

        # Thrust nudge (accumulates — phone applies to base_thrust)
        if taking_off and drone_connected:
            if raw_key == ARROW_UP:
                thrust_delta += THRUST_STEP
                print(f"  Thrust nudge → {thrust_delta:+d}")
            elif raw_key == ARROW_DOWN:
                thrust_delta -= THRUST_STEP
                print(f"  Thrust nudge → {thrust_delta:+d}")
            elif key_char in (ord('+'), ord('=')):
                thrust_delta += 5000
                print(f"  [BOOST+] nudge → {thrust_delta:+d}")
            elif key_char == ord('-'):
                thrust_delta -= 5000
                print(f"  [BOOST-] nudge → {thrust_delta:+d}")

        # ── Send cmd at ~30 fps ────────────────────────────────────────────
        if time.time() - last_send > 0.033:
            last_send = time.time()
            send_cmd(armed=taking_off, emergency=False)

        # ── Camera display ─────────────────────────────────────────────────
        if hwnd is None:
            hwnd = find_window(source_titles)
            if not hwnd:
                empty = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(empty, "WAITING FOR CAMERA...", (120, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,150,255), 2)
                with phone_pos_lock:
                    bt = phone_pos[3] if phone_pos else 0
                draw_status(empty, taking_off, bt)
                cv2.imshow("Neurotech Ground Station", empty)
                continue

        frame = capture_frame(hwnd)
        if frame is None:
            hwnd = None
            continue

        fh, fw = frame.shape[:2]
        cx, cy = fw//2, fh//2
        cv2.drawMarker(frame, (cx,cy), (80,80,80),
                       cv2.MARKER_CROSS, markerSize=40, thickness=2)

        # ── Overlay phone position data ────────────────────────────────────
        with phone_pos_lock:
            ppos = phone_pos

        if ppos is not None:
            x_m, y_m, z_m, bt = ppos
            cv2.rectangle(frame, (8,8), (440,120), (0,0,0), -1)
            hover_lbl = "HOVER [H]" if hover_only else "BALANCE [H]"
            hover_col = (0,255,255) if hover_only else (0,200,0)
            cv2.putText(frame, f"PHONE ArUco  |  {hover_lbl}", (18,36),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, hover_col, 2)
            if z_m > 0:
                cv2.putText(frame, f"X:{x_m:+.2f}  Y:{y_m:+.2f}  Z:{z_m:.2f}m", (18,72),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            else:
                cv2.putText(frame, "SEARCHING FOR ArUco...", (18,72),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,165,255), 2)
            cv2.putText(frame, f"Thrust:{int(bt)}  nudge:{thrust_delta:+d}", (18,108),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180,180,255), 1)
        elif taking_off:
            cv2.putText(frame, "CLIMBING... (phone searching ArUco)", (cx-240, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,165,255), 2)
        else:
            cv2.putText(frame, "PRESS SPACE TO ARM", (cx-160, cy+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        with phone_pos_lock:
            bt = phone_pos[3] if phone_pos else 35000
        draw_status(frame, taking_off, bt)
        cv2.imshow("Neurotech Ground Station", cv2.resize(frame, (640,480)))

    # ── Shutdown ───────────────────────────────────────────────────────────
    send_cmd(armed=False, emergency=False)
    if sock:
        sock.close()
    cv2.destroyAllWindows()
    keyboard.unhook_all()

if __name__ == "__main__":
    main()
