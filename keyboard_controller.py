"""
Neurotech Drone Keyboard Controller
Left hand : W A S D   (pitch/roll)
Right hand : Arrow Up/Down = altitude, Arrow Left/Right = pinch toggle
Q or Esc   : quit

Uses only built-in msvcrt + pyserial (already installed).
Does NOT send any data until the FIRST key press (as requested).
"""

import msvcrt
import serial
import socket
import time
import os

PORT   = 'COM5'
BAUD   = 115200
STEP   = 0.05       # movement increment per key press

VIZ_ADDR = ("127.0.0.1", 1235)

# Extended-key scan codes returned by msvcrt.getch() after a 0x00 or 0xE0 prefix
ARROW_UP    = 72
ARROW_DOWN  = 80
ARROW_LEFT  = 75
ARROW_RIGHT = 77

def clear():
    os.system("cls")

def draw_hud(x, y, z, pinch, started, status):
    clear()
    print("=" * 50)
    print("  NEUROTECH DRONE KEYBOARD CONTROLLER")
    print("=" * 50)
    print(f"  W/S  : Pitch  (Y)  |  A/D  : Roll (X)")
    print(f"  UP/DN: Altitude(Z) |  L/R  : Pinch toggle")
    print(f"  Q/Esc: Quit")
    print("-" * 50)
    if not started:
        print("  *** WAITING FOR FIRST KEY PRESS ***")
    else:
        print(f"  ACTIVE — sending to {PORT}")
    print("-" * 50)
    print(f"  X (Roll)   : {x:+.2f}")
    print(f"  Y (Pitch)  : {y:+.2f}")
    print(f"  Z (Alt)    : {z:.2f}")
    print(f"  PINCH      : {'ACTIVE' if pinch else 'OFF'}")
    print("-" * 50)
    print(f"  {status}")
    print("=" * 50)

def main():
    status = "Connecting..."
    ser = None
    esp32_alive = False
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
        # Wait for the ESP32 to boot and settle after the serial line opens
        time.sleep(2.0)
        
        # Drain boot logs
        boot_logs = ser.read_all()
        
        # Ping the ESP32: send a dummy command and see if it echoes back
        ser.write(b'0.00,0.00,0.50,0\n')
        time.sleep(0.1)
        resp = ser.read_all()
        if resp or b'READY' in boot_logs:
            esp32_alive = True
            msg = resp.decode('utf-8', errors='ignore').strip()
            if not msg: msg = "READY"
            status = f"ESP32 READY on {PORT} | {msg[:60]}"
        else:
            status = f"COM5 open but ESP32 silent — flash drone_usb_bridge.ino first!"
        ser.timeout = 0.05
    except Exception as e:
        status = f"Cannot open {PORT}: {e}"

    viz_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    x, y, z, pinch = 0.0, 0.0, 0.5, 0
    started = False

    draw_hud(x, y, z, pinch, started, status)

    while True:
        # Non-blocking key check
        if not msvcrt.kbhit():
            time.sleep(0.05)
            # Still drain ESP32 output even if no key pressed
            if ser and started:
                while ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        status = f"[ESP32] {line}"
            continue

        ch = msvcrt.getch()

        # Esc or extended key prefix
        if ch in (b'\x00', b'\xe0'):
            ext = ord(msvcrt.getch())
            if ext == ARROW_UP:
                z = min(1.0, z + STEP);   started = True
            elif ext == ARROW_DOWN:
                z = max(0.0, z - STEP);   started = True
            elif ext == ARROW_LEFT:
                pinch = 0;                started = True
            elif ext == ARROW_RIGHT:
                pinch = 1;                started = True
        else:
            key = ch.decode('utf-8', errors='ignore').lower()
            if key in ('q', '\x1b'):
                break
            elif key == 'w':
                y = min(1.0,  y + STEP);  started = True
            elif key == 's':
                y = max(-1.0, y - STEP);  started = True
            elif key == 'a':
                x = max(-1.0, x - STEP);  started = True
            elif key == 'd':
                x = min(1.0,  x + STEP);  started = True

        if started:
            payload = f"{x:.2f},{y:.2f},{z:.2f},{pinch}\n"
            if ser:
                try:
                    ser.write(payload.encode())
                    # Drain any echo / response from ESP32
                    time.sleep(0.02)
                    while ser.in_waiting:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            status = f"[ESP32] {line}"
                except Exception as e:
                    status = f"Serial error: {e}"
            else:
                status = f"(no serial) cmd: {payload.strip()}"

            # Forward to local visualizer
            viz_mode = "PINCH (MICRO)" if pinch else "FIST (MACRO)"
            viz_pkt  = f"{x:.3f},{y:.3f},{z - 0.5:.3f},{viz_mode}"
            try:
                viz_sock.sendto(viz_pkt.encode(), VIZ_ADDR)
            except Exception:
                pass

        draw_hud(x, y, z, pinch, started, status)

    if ser:
        ser.close()
    viz_sock.close()
    clear()
    print("Controller stopped.")

if __name__ == "__main__":
    main()
