"""
drone_direct/sensor_probe.py
============================
Queries the ESP-Drone firmware via CRTP over UDP to enumerate all registered
sensors by reading the Parameter TOC and Logging TOC.

Usage:
  1. Connect PC WiFi to drone hotspot
  2. Run:  python sensor_probe.py

Outputs every sensor/variable group the firmware reports, so you can see
exactly what hardware is present and initialised.
"""

import socket
import time
import struct
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'tuner'))
import config as cfg

DRONE_IP   = cfg.DRONE_IP
DRONE_PORT = cfg.DRONE_PORT
TIMEOUT    = 1.0   # seconds to wait for each response
RETRIES    = 5


# ── CRTP helpers ─────────────────────────────────────────────────────────────

def _cksum(data: bytes) -> int:
    return sum(data) & 0xFF


def make_packet(port: int, channel: int, payload: bytes) -> bytes:
    header = ((port & 0xF) << 4) | (channel & 0x3)
    body   = bytes([header]) + payload
    return body + bytes([_cksum(body)])


def send_recv(sock, packet: bytes, retries: int = RETRIES):
    """Send packet, return first response that echoes the same port/channel."""
    port    = (packet[0] >> 4) & 0xF
    channel = packet[0] & 0x3
    for attempt in range(retries):
        sock.sendto(packet, (DRONE_IP, DRONE_PORT))
        try:
            data, _ = sock.recvfrom(256)
            if len(data) >= 2:
                rport    = (data[0] >> 4) & 0xF
                rchannel = data[0] & 0x3
                if rport == port and rchannel == channel:
                    return data[1:-1]   # strip header and checksum
        except socket.timeout:
            pass
    return None


# ── Parameter TOC (Port 2, Channel 0) ────────────────────────────────────────
# Used to enumerate all firmware parameters (includes sensor config/values)

PARAM_PORT = 2
PARAM_CH   = 0


def get_param_count(sock):
    pkt  = make_packet(PARAM_PORT, PARAM_CH, bytes([0x01]))  # GetInfo
    resp = send_recv(sock, pkt)
    if resp:
        print(f"DEBUG: Parameter GetInfo resp ({len(resp)} bytes): {resp.hex()}")
    if resp and len(resp) >= 3:
        # response: [cmd=0x01, count(2LE), crc(4LE)]
        count = struct.unpack_from('<H', resp, 1)[0]
        crc = None
        if len(resp) >= 7:
            crc = struct.unpack_from('<I', resp, 3)[0]
        return count, crc
    return None, None


def get_param_item(sock, idx: int):
    pkt  = make_packet(PARAM_PORT, PARAM_CH,
                       bytes([0x00]) + struct.pack('<H', idx))  # GetItem
    resp = send_recv(sock, pkt)
    if resp and len(resp) >= 4:
        # response: [cmd=0x00, idx(2LE), type, group\0name\0]
        try:
            raw = resp[4:]
            null = raw.index(0)
            group = raw[:null].decode('ascii', errors='replace')
            rest  = raw[null+1:]
            null2 = rest.index(0)
            name  = rest[:null2].decode('ascii', errors='replace')
            return group, name
        except (ValueError, IndexError):
            pass
    return None, None


# ── Logging TOC (Port 5, Channel 0) ──────────────────────────────────────────
# Used to enumerate all loggable variables (sensor readings)

LOG_PORT = 5
LOG_CH   = 0


def get_log_count(sock):
    pkt  = make_packet(LOG_PORT, LOG_CH, bytes([0x01]))  # GetInfo
    resp = send_recv(sock, pkt)
    if resp and len(resp) >= 3:
        count = struct.unpack_from('<H', resp, 1)[0]
        return count
    return None


def get_log_item(sock, idx: int):
    pkt  = make_packet(LOG_PORT, LOG_CH,
                       bytes([0x00]) + struct.pack('<H', idx))  # GetItem
    resp = send_recv(sock, pkt)
    if resp and len(resp) >= 4:
        try:
            raw  = resp[4:]
            null = raw.index(0)
            group = raw[:null].decode('ascii', errors='replace')
            rest  = raw[null+1:]
            null2 = rest.index(0)
            name  = rest[:null2].decode('ascii', errors='replace')
            return group, name
        except (ValueError, IndexError):
            pass
    return None, None


# ── Sensor keyword filter ─────────────────────────────────────────────────────
SENSOR_KEYWORDS = {
    'acc', 'gyro', 'mag', 'baro', 'pressure', 'temp', 'imu', 'mpu',
    'icm', 'lsm', 'bmp', 'ms56', 'spl', 'flow', 'zrange', 'range',
    'tof', 'vl53', 'pmw', 'paa', 'bat', 'pm', 'stabilizer',
}

def is_sensor_related(group: str, name: str) -> bool:
    combined = (group + '.' + name).lower()
    return any(kw in combined for kw in SENSOR_KEYWORDS)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  ESP-Drone Sensor Probe (CRTP TOC query)")
    print("=" * 60)
    print(f"  Drone : {DRONE_IP}:{DRONE_PORT}")
    print()
    print("  Make sure your WiFi is connected to the drone hotspot.")
    print()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(TIMEOUT)

    # ── Parameter TOC ─────────────────────────────────────────────────────────
    print("── Querying Parameter TOC …")
    count, crc = get_param_count(sock)
    if count is None:
        print("  [!] No response — is the drone on and WiFi connected?")
    else:
        crc_str = f"{crc:#010x}" if crc is not None else "N/A"
        print(f"  Found {count} parameters  (CRC={crc_str})")
        if count > 2000:
            print(f"  [!] Count seems high, limiting to 2000 items for safety.")
            count = 2000
            
        sensor_params = []
        all_groups = set()
        for i in range(count):
            g, n = get_param_item(sock, i)
            if i < 5:
                print(f"    DEBUG: Item {i} -> {g}.{n}")
            if g is not None:
                all_groups.add(g)
                if is_sensor_related(g, n):
                    sensor_params.append(f"{g}.{n}")
            if i % 20 == 0 and i > 0:
                print(f"  … {i}/{count}", end='\r')

        print()
        print(f"\n  All parameter groups ({len(all_groups)}):")
        for grp in sorted(all_groups):
            print(f"    {grp}")

        if sensor_params:
            print(f"\n  Sensor-related parameters ({len(sensor_params)}):")
            for p in sorted(sensor_params):
                print(f"    {p}")

    # ── Logging TOC ───────────────────────────────────────────────────────────
    print()
    print("── Querying Logging TOC …")
    lcount = get_log_count(sock)
    if lcount is None:
        print("  [!] No response from logging TOC")
    else:
        print(f"  Found {lcount} log variables")
        sensor_logs = []
        log_groups  = set()
        for i in range(lcount):
            g, n = get_log_item(sock, i)
            if g is not None:
                log_groups.add(g)
                if is_sensor_related(g, n):
                    sensor_logs.append(f"{g}.{n}")
            if i % 20 == 0 and i > 0:
                print(f"  … {i}/{lcount}", end='\r')

        print()
        print(f"\n  All log groups ({len(log_groups)}):")
        for grp in sorted(log_groups):
            print(f"    {grp}")

        if sensor_logs:
            print(f"\n  Sensor-related log variables ({len(sensor_logs)}):")
            for v in sorted(sensor_logs):
                print(f"    {v}")

    sock.close()
    print()
    print("Done.")


if __name__ == '__main__':
    main()
