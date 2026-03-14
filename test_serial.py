import serial
import time

PORTS = ['COM3', 'COM4', 'COM5']
BAUD = 115200

def test_connection():
    for port in PORTS:
        try:
            print(f"\n--- Testing {port} at {BAUD} ---")
            ser = serial.Serial(port, BAUD, timeout=1)
            # Toggle DTR/RTS to hard reset ESP32 usually
            ser.setDTR(False)
            ser.setRTS(False)
            time.sleep(0.1)
            ser.setDTR(True)
            ser.setRTS(True)
            
            time.sleep(1.5) # Wait for ESP32 boot
            
            # Read whatever comes out
            out = ser.read_all()
            if out:
                try:
                    out_str = out.decode('utf-8', errors='ignore')
                except:
                    out_str = str(out)
                print(f"[RECV]\n{repr(out_str)}")
                if "Drone" in out_str or "Ready" in out_str or "ESP" in out_str or "EEPROM" in out_str:
                    print(f">>> SUSPECTED DRONE PORT FOUND: {port} <<<")
            else:
                print("[RECV] No data")
            ser.close()
        except Exception as e:
            print(f"Error on {port}: {e}")

if __name__ == "__main__":
    test_connection()
