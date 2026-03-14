"""
Neurotech Phone Relay — runs on Android via Termux
Bridges commands from PC (over ADB/USB) to the drone (over WiFi)

Setup on phone:
  pkg install python
  python relay.py

Then on PC run:
  adb forward tcp:9391 tcp:9391
"""

import socket
import threading

LISTEN_HOST = '127.0.0.1'   # Only accept from ADB tunnel
LISTEN_PORT = 9391           # PC will forward here via ADB

DRONE_IP    = '192.168.43.42'
DRONE_PORT  = 2390

def handle_client(conn, addr):
    print(f"[+] PC connected from {addr}")
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        while True:
            data = conn.recv(256)
            if not data:
                break
            udp.sendto(data, (DRONE_IP, DRONE_PORT))
    except Exception as e:
        print(f"[!] {e}")
    finally:
        udp.close()
        conn.close()
        print(f"[-] PC disconnected")

def main():
    print("=" * 45)
    print("  Neurotech Phone Relay")
    print("=" * 45)
    print(f"  Listening : {LISTEN_HOST}:{LISTEN_PORT} (TCP)")
    print(f"  Forwarding: {DRONE_IP}:{DRONE_PORT} (UDP)")
    print()
    print("  On PC, run:")
    print("    adb forward tcp:9391 tcp:9391")
    print()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((LISTEN_HOST, LISTEN_PORT))
    server.listen(5)
    print(f"[*] Relay ready. Waiting for PC connection...")

    while True:
        conn, addr = server.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()

if __name__ == "__main__":
    main()
