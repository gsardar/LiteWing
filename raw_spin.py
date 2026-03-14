"""
Neurotech - Raw Spin Test via Phone Relay
PC  --USB/ADB-->  Phone  --WiFi-->  Drone

Prerequisites:
  1. Start relay.py in Termux on phone
  2. Run: adb forward tcp:9391 tcp:9391
  3. Run this script
"""

import socket
import struct
import time

RELAY_HOST = '127.0.0.1'
RELAY_PORT = 9391          # ADB forwards this to phone relay

SPIN_THRUST   = 15000
SPIN_DURATION = 2.0


def cksum(data: bytes) -> int:
    return sum(data) & 0xFF


def make_rpyt(roll, pitch, yaw, thrust) -> bytes:
    """
    CRTP packet layout (from crtp_commander_rpyt.c):
      Byte 0   : header = (port << 4) | channel = 0x30
      Bytes 1-4: roll  (float32, little-endian)
      Bytes 5-8: pitch (float32, little-endian)
      Bytes 9-12: yaw  (float32, little-endian)
      Bytes 13-14: thrust (uint16, little-endian)
      Byte 15  : checksum = sum of bytes 0-14

    pk->data in firmware = bytes 1..14 (after header stripping by CRTP layer)
    So the struct CommanderCrtpLegacyValues maps to bytes 1-14 correctly.
    """
    header = (3 << 4) | 0   # Port 3, Channel 0 = 0x30
    t = max(0, min(65535, int(thrust)))
    data = struct.pack('<BfffH', header, float(roll), float(pitch), float(yaw), t)
    return data + bytes([cksum(data)])


def main():
    print("=" * 50)
    print("  ESP-Drone Spin Test  (via Phone Relay)")
    print("=" * 50)
    print(f"  Relay : {RELAY_HOST}:{RELAY_PORT}")
    print(f"  Ensure: adb forward tcp:9391 tcp:9391 is active")
    print()
    print("  ⚠️  PROPELLERS OFF BEFORE PROCEEDING ⚠️")
    print()
    input("  Press ENTER to spin, Ctrl+C to abort...")
    print()

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((RELAY_HOST, RELAY_PORT))
        print(f"[✓] Connected to phone relay!")

        def send(roll, pitch, yaw, thrust, duration, label):
            print(f"  [{label}]  thrust={int(thrust)}  ({duration}s)")
            pkt = make_rpyt(roll, pitch, yaw, thrust)
            t_end = time.time() + duration
            while time.time() < t_end:
                sock.sendall(pkt)
                time.sleep(0.02)

        send(0, 0, 0, 0,            2.0, "UNLOCK (zero)")
        send(0, 0, 0, SPIN_THRUST,  SPIN_DURATION, "SPIN")
        send(0, 0, 0, 0,            1.0, "STOP")

        sock.close()
        print()
        print("[✓] Done! Motors should have spun briefly.")

    except ConnectionRefusedError:
        print("[✗] Cannot connect to phone relay!")
        print("    1. Is relay.py running in Termux on phone?")
        print("    2. Did you run: adb forward tcp:9391 tcp:9391")
    except KeyboardInterrupt:
        print("\n[!] Emergency stop")
        sock.sendall(make_rpyt(0, 0, 0, 0))
        sock.close()


if __name__ == "__main__":
    main()
