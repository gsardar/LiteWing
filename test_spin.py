"""
Neurotech Drone - Motor Spin Test
Connects to ESP-Drone via Wi-Fi (192.168.43.42:2390) and sends a short
low-thrust spin to verify the control link is working.

SAFETY: Remove propellers before running this test!
"""

import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = 'udp://192.168.43.42:2390'

# How long to spin (seconds) and at what thrust (0–65535)
# 15000 is a gentle spin-up — not enough to lift, just enough to confirm motors respond
SPIN_DURATION = 2.0
SPIN_THRUST   = 15000

def run_test():
    print("=" * 50)
    print("  ESP-Drone Motor Spin Test")
    print("=" * 50)
    print(f"  URI:      {URI}")
    print(f"  Thrust:   {SPIN_THRUST}")
    print(f"  Duration: {SPIN_DURATION}s")
    print()
    print("  ⚠️  MAKE SURE PROPELLERS ARE OFF ⚠️")
    print()
    input("  Press ENTER to connect and spin, or Ctrl+C to abort...")

    cflib.crtp.init_drivers()
    print(f"\n[*] Connecting to {URI}...")

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            print("[✓] Connected!")
            cf = scf.cf

            print("[*] Sending zero-thrust unlock commands (2s)...")
            t_end = time.time() + 2.0
            while time.time() < t_end:
                cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(0.05)

            print(f"[*] SPINNING at thrust={SPIN_THRUST} for {SPIN_DURATION}s ...")
            t_end = time.time() + SPIN_DURATION
            while time.time() < t_end:
                cf.commander.send_setpoint(0, 0, 0, SPIN_THRUST)
                time.sleep(0.05)

            print("[*] Cutting motors (sending zero thrust)...")
            for _ in range(20):
                cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(0.05)

            print("[✓] Done! Motors should be stopped.")

    except KeyboardInterrupt:
        print("\n[!] Aborted by user.")
    except Exception as e:
        print(f"\n[✗] Error: {e}")
        print("\n  Possible causes:")
        print("  - PC not connected to drone Wi-Fi (ESP-DRONE_...)")
        print("  - Drone not powered on")
        print("  - Wrong firmware (needs official LiteWing.bin)")

if __name__ == "__main__":
    run_test()
