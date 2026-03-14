"""
spin_test.py — Direct UDP motor test
Laptop connects to drone WiFi hotspot directly (no phone relay needed).

Prerequisites:
  1. Connect laptop WiFi to drone hotspot
  2. Run: python spin_test.py
"""

import socket
import struct
import time

DRONE_IP   = '192.168.43.42'
DRONE_PORT = 2390

SPIN_THRUST   = 25000
SPIN_DURATION = 2.0


def cksum(data: bytes) -> int:
    return sum(data) & 0xFF


def make_rpyt(roll, pitch, yaw, thrust) -> bytes:
    header = (3 << 4) | 0  # Port 3, Channel 0 = 0x30
    t = max(0, min(65535, int(thrust)))
    data = struct.pack('<BfffH', header, float(roll), float(pitch), float(yaw), t)
    return data + bytes([cksum(data)])


def main():
    print("=" * 50)
    print("  ESP-Drone Direct Spin Test")
    print("=" * 50)
    print(f"  Drone : {DRONE_IP}:{DRONE_PORT} (UDP direct)")
    print()
    print("  ⚠️  REMOVE PROPELLERS BEFORE PROCEEDING ⚠️")
    print()
    input("  Press ENTER to spin, Ctrl+C to abort...")
    print()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(roll, pitch, yaw, thrust, duration, label):
        print(f"  [{label}]  thrust={int(thrust)}  ({duration}s)")
        pkt = make_rpyt(roll, pitch, yaw, thrust)
        t_end = time.time() + duration
        while time.time() < t_end:
            sock.sendto(pkt, (DRONE_IP, DRONE_PORT))
            time.sleep(0.02)

    try:
        send(0, 0, 0, 0,           2.0, "UNLOCK (zero)")
        send(0, 0, 0, SPIN_THRUST, SPIN_DURATION, "SPIN")
        send(0, 0, 0, 0,           1.0, "STOP")
        print()
        print("[OK] Done.")
    except KeyboardInterrupt:
        print("\n[!] Abort — cutting motors")
        sock.sendto(make_rpyt(0, 0, 0, 0), (DRONE_IP, DRONE_PORT))
    finally:
        sock.close()


if __name__ == "__main__":
    main()
