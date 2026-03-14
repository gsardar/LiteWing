"""
drone_direct/tuner/config.py
============================
All tunable parameters. Edit here — hover_gui.py hot-reloads this every 0.5s.
Connection: PC WiFi connected to drone's hotspot.
"""

# ── Network ──────────────────────────────────────────────────────────────────
DRONE_IP   = '192.168.43.42'   # Default for drone softAP
DRONE_PORT = 2390            # ESP-Drone CRTP UDP port

# ── Thrust ───────────────────────────────────────────────────────────────────
HOVER_THRUST     = 38000   # tune this first! increase until it lifts
THRUST_RAMP_TIME = 2.0     # seconds to ramp 0 -> HOVER_THRUST on arm
THRUST_MIN       = 10000   # safety floor
THRUST_MAX       = 60000   # safety ceiling

# ── Arm / Disarm ─────────────────────────────────────────────────────────────
UNLOCK_DURATION  = 1.0     # seconds of zero-thrust to unlock ESCs
DESCENT_STEPS    = 40
DESCENT_SECS     = 2.5

# ── Attitude trim (degrees) ──────────────────────────────────────────────────
# Positive roll  → tilts right   Negative → left
# Positive pitch → tilts forward  Negative → backward
TRIM_ROLL  = 0.0
TRIM_PITCH = 0.0
TRIM_YAW   = 0.0    # degrees/s

# ── Packet rate ───────────────────────────────────────────────────────────────
SEND_HZ = 50
