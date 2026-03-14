"""
drone_direct/tuner/hover_gui.py
================================
Real-time slider UI for hand-tuning drone hover.
Sends CRTP RPYT packets directly to the drone over UDP while you adjust sliders.

How to tune (holding drone in hand):
  1. ARM — slowly increase Thrust until the drone feels light in your hand
  2. Notice which direction it wants to pull/rotate
  3. Adjust Roll/Pitch/Yaw trim sliders to counteract
  4. Hit "Save to config.py" to persist

Run:
  cd drone_direct/tuner
  python hover_gui.py
"""

import socket
import struct
import sys
import os
import threading
import time

# Allow running from repo root too
sys.path.insert(0, os.path.dirname(__file__))
import config as cfg

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSlider, QLabel, QPushButton, QGroupBox, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont


# ── Protocol Selection ────────────────────────────────────────────────────────
# Set this to True if using the 'keyboard_control.ino' firmware.
# Set this to False if using the original binary CRTP firmware.
USE_CSV_PROTOCOL = True

def _cksum(data: bytes) -> int:
    return sum(data) & 0xFF

def make_rpyt(roll: float, pitch: float, yaw: float, thrust: int) -> bytes:
    if USE_CSV_PROTOCOL:
        # Map to -1.0 to 1.0 range expected by firmware
        # roll/pitch are degrees, thrust is 0-65535
        x = max(-1.0, min(1.0, roll / 20.0))
        y = max(-1.0, min(1.0, pitch / 20.0))
        z = max(0.0, min(1.0, thrust / 65535.0))
        p = 0
        return f"{x:.2f},{y:.2f},{z:.2f},{p}\n".encode()

    t       = max(0, min(65535, int(thrust)))
    payload = struct.pack('<BfffH', 0x30, float(roll), float(pitch), float(yaw), t)
    return payload + bytes([_cksum(payload)])


# ── UDP sender thread ─────────────────────────────────────────────────────────
class DroneLink(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.sock       = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock      = threading.Lock()
        self._armed     = False
        self._emergency = False
        self._thrust    = cfg.HOVER_THRUST
        self._roll      = cfg.TRIM_ROLL
        self._pitch     = cfg.TRIM_PITCH
        self._yaw       = cfg.TRIM_YAW
        self._running   = True

    def _send(self, roll, pitch, yaw, thrust):
        try:
            self.sock.sendto(make_rpyt(roll, pitch, yaw, thrust),
                             (cfg.DRONE_IP, cfg.DRONE_PORT))
        except OSError as e:
            # Silently catch unreachable network/host errors to prevent thread crash
            # Error 10051 is WinError unreachable network
            pass

    def set_params(self, thrust, roll, pitch, yaw):
        with self._lock:
            self._thrust = thrust
            self._roll   = roll
            self._pitch  = pitch
            self._yaw    = yaw

    def arm(self):
        threading.Thread(target=self._ramp_arm, daemon=True).start()

    def disarm(self):
        threading.Thread(target=self._ramp_disarm, daemon=True).start()

    def emergency(self):
        with self._lock:
            self._emergency = True
            self._armed     = False

    def clear_emergency(self):
        with self._lock:
            self._emergency = False

    @property
    def is_armed(self):
        with self._lock:
            return self._armed

    @property
    def is_emergency(self):
        with self._lock:
            return self._emergency

    def _ramp_arm(self):
        self._send(0, 0, 0, 0)
        time.sleep(cfg.UNLOCK_DURATION)
        steps  = max(10, int(cfg.THRUST_RAMP_TIME * cfg.SEND_HZ))
        target = cfg.HOVER_THRUST
        with self._lock:
            self._thrust = 0
            self._armed  = True
        for i in range(steps):
            with self._lock:
                self._thrust = int(target * (i + 1) / steps)
            time.sleep(cfg.THRUST_RAMP_TIME / steps)
        with self._lock:
            self._thrust = target

    def _ramp_disarm(self):
        with self._lock:
            from_t = self._thrust
        delay = cfg.DESCENT_SECS / cfg.DESCENT_STEPS
        for i in range(cfg.DESCENT_STEPS):
            with self._lock:
                self._thrust = max(0, int(from_t * (1 - (i+1)/cfg.DESCENT_STEPS)))
            time.sleep(delay)
        with self._lock:
            self._thrust = 0
            self._armed  = False

    def run(self):
        interval = 1.0 / cfg.SEND_HZ
        SMOOTH = 0.08
        cur_t, cur_r, cur_p, cur_y = 0.0, 0.0, 0.0, 0.0
        while self._running:
            with self._lock:
                armed = self._armed
                emerg = self._emergency
                tgt_t = float(self._thrust)
                tgt_r, tgt_p, tgt_y = self._roll, self._pitch, self._yaw
            cur_t += (tgt_t - cur_t) * SMOOTH
            cur_r += (tgt_r - cur_r) * SMOOTH
            cur_p += (tgt_p - cur_p) * SMOOTH
            cur_y += (tgt_y - cur_y) * SMOOTH
            if armed and not emerg:
                self._send(cur_r, cur_p, cur_y, int(cur_t))
            else:
                self._send(0, 0, 0, 0)
            time.sleep(interval)

    def stop(self):
        self._running = False
        try:
            self._send(0, 0, 0, 0)
            self.sock.close()
        except Exception:
            pass


# ── Vertical slider with label ─────────────────────────────────────────────────
class ParamSlider(QWidget):
    def __init__(self, title, min_v, max_v, default, scale=1.0,
                 unit='', decimals=0, color='#00bcd4', desc=''):
        super().__init__()
        self.scale = scale
        self.decimals = decimals
        self.unit = unit

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(4)

        # Title
        lbl = QLabel(title.upper())
        lbl.setFont(QFont('Segoe UI', 9, QFont.Bold))
        lbl.setStyleSheet('color:#666; letter-spacing:1px;')
        lbl.setAlignment(Qt.AlignCenter)

        # Description — plain-English explanation
        desc_lbl = QLabel(desc)
        desc_lbl.setFont(QFont('Segoe UI', 8))
        desc_lbl.setStyleSheet('color:#3a3a5a; font-style:italic;')
        desc_lbl.setAlignment(Qt.AlignCenter)
        desc_lbl.setWordWrap(True)

        # Live value display
        self.val_lbl = QLabel()
        self.val_lbl.setFont(QFont('Segoe UI', 26, QFont.Bold))
        self.val_lbl.setAlignment(Qt.AlignCenter)
        self.val_lbl.setStyleSheet(f'color:{color};')

        self.slider = QSlider(Qt.Vertical)
        self.slider.setRange(min_v, max_v)
        self.slider.setValue(default)
        self.slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.slider.setStyleSheet(f"""
            QSlider::groove:vertical {{
                background:#1e1e2e; width:10px; border-radius:5px;
            }}
            QSlider::handle:vertical {{
                background:{color}; height:24px; width:24px;
                margin:0 -7px; border-radius:12px;
            }}
            QSlider::sub-page:vertical {{
                background:{color}; border-radius:5px;
            }}
        """)
        self.slider.valueChanged.connect(self._update_label)

        lay.addWidget(lbl)
        lay.addWidget(desc_lbl)
        lay.addWidget(self.val_lbl)
        lay.addWidget(self.slider, stretch=1)
        self._update_label(default)

    def _update_label(self, raw):
        v = raw * self.scale
        txt = f"{int(v)}" if self.decimals == 0 else f"{v:+.{self.decimals}f}"
        self.val_lbl.setText(txt + self.unit)

    def value(self):
        return self.slider.value() * self.scale

    def set_value(self, real):
        self.slider.setValue(int(real / self.scale))


# ── Main window ───────────────────────────────────────────────────────────────
class TunerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.drone = DroneLink()
        self.drone.start()
        self.setWindowTitle("Hover Tuner — drone_direct")
        self.setMinimumSize(800, 520)
        self._build_ui()

        self._poll = QTimer()
        self._poll.timeout.connect(self._refresh)
        self._poll.start(120)

        self._cfg_mtime = self._get_mtime()
        self._hot = QTimer()
        self._hot.timeout.connect(self._hot_reload)
        self._hot.start(500)

    def _build_ui(self):
        root = QWidget()
        root.setStyleSheet("""
            QWidget { background:#111120; color:#ddd;
                      font-family:'Segoe UI',sans-serif; }
            QGroupBox { border:1px solid #2a2a3e; border-radius:10px;
                        margin-top:12px; padding:8px;
                        font-size:10px; font-weight:bold;
                        color:#444; letter-spacing:1px; }
            QGroupBox::title { subcontrol-origin:margin; left:12px; padding:0 4px; }
        """)
        self.setCentralWidget(root)
        v = QVBoxLayout(root)
        v.setContentsMargins(16, 12, 16, 12)
        v.setSpacing(10)

        # Header
        h = QLabel("Hover Tuner")
        h.setFont(QFont('Segoe UI', 18, QFont.Bold))
        h.setStyleSheet('color:#fff;')
        sub = QLabel(f"{cfg.DRONE_IP}:{cfg.DRONE_PORT}  ·  Hold drone in hand → ARM → find hover thrust → fix drift with trims")
        sub.setStyleSheet('color:#444; font-size:10px;')
        v.addWidget(h)
        v.addWidget(sub)

        # Sliders
        sg = QGroupBox("Parameters  (sliders send live UDP)")
        sl = QHBoxLayout(sg)
        sl.setSpacing(16)

        self.sl_thrust = ParamSlider(
            "Thrust", cfg.THRUST_MIN, cfg.THRUST_MAX, cfg.HOVER_THRUST,
            color='#ff6b6b',
            desc="Motor power. Slide up until the drone feels light in your hand. Too low = won't lift. Too high = flies away."
        )
        self.sl_roll = ParamSlider(
            "Roll trim", -200, 200, int(cfg.TRIM_ROLL * 10),
            scale=0.1, unit='°', decimals=1, color='#ffd93d',
            desc="Left / Right tilt correction. If drone pulls RIGHT → go negative. If it pulls LEFT → go positive."
        )
        self.sl_pitch = ParamSlider(
            "Pitch trim", -200, 200, int(cfg.TRIM_PITCH * 10),
            scale=0.1, unit='°', decimals=1, color='#6bcb77',
            desc="Forward / Backward tilt correction. If drone leans FORWARD → go negative. Leans BACK → go positive."
        )
        self.sl_yaw = ParamSlider(
            "Yaw trim", -300, 300, int(cfg.TRIM_YAW * 10),
            scale=0.1, unit='°/s', decimals=1, color='#845ef7',
            desc="Spin correction. If drone slowly rotates CLOCKWISE → go negative. COUNTER-clockwise → go positive."
        )

        for s in (self.sl_thrust, self.sl_roll, self.sl_pitch, self.sl_yaw):
            s.slider.valueChanged.connect(self._push)
            sl.addWidget(s)
        v.addWidget(sg, stretch=1)

        # Buttons
        bg = QGroupBox("Controls")
        bl = QHBoxLayout(bg)
        bl.setSpacing(10)

        self.btn_arm  = self._btn("ARM", '#22aa55', self._arm_toggle, h=50)
        self.btn_emrg = self._btn("⚠ EMERGENCY", '#dd3333', self._emergency, h=50)
        self.btn_rst  = self._btn("Reset Trims", '#2a2a3e', self._reset, h=50)
        self.btn_save = self._btn("💾 Save config", '#1a3a5c', self._save, h=50)

        for b in (self.btn_arm, self.btn_emrg, self.btn_rst, self.btn_save):
            bl.addWidget(b)
        v.addWidget(bg)

        # Status
        self.status = QLabel("DISARMED")
        self.status.setFont(QFont('Segoe UI', 12, QFont.Bold))
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setFixedHeight(34)
        self.status.setStyleSheet(
            'background:#1a1a2e; border-radius:8px; color:#ffd93d; padding:4px;')
        v.addWidget(self.status)

    def _btn(self, text, color, slot, h=44):
        b = QPushButton(text)
        b.setFixedHeight(h)
        b.setFont(QFont('Segoe UI', 11, QFont.Bold))
        b.setStyleSheet(f'background:{color}; color:#fff; border-radius:8px;')
        b.clicked.connect(slot)
        return b

    def _push(self):
        self.drone.set_params(int(self.sl_thrust.value()),
                               self.sl_roll.value(),
                               self.sl_pitch.value(),
                               self.sl_yaw.value())

    def _arm_toggle(self):
        if self.drone.is_emergency: return
        self.drone.disarm() if self.drone.is_armed else self.drone.arm()

    def _emergency(self):
        if self.drone.is_emergency:
            self.drone.clear_emergency()
            self.btn_emrg.setText("⚠ EMERGENCY")
            self.btn_emrg.setStyleSheet('background:#dd3333; color:#fff; border-radius:8px;')
        else:
            self.drone.emergency()
            self.btn_emrg.setText("Clear Emergency")
            self.btn_emrg.setStyleSheet('background:#555; color:#aaa; border-radius:8px;')

    def _reset(self):
        self.sl_roll.set_value(0.0)
        self.sl_pitch.set_value(0.0)
        self.sl_yaw.set_value(0.0)
        self._push()

    def _save(self):
        import re
        path = os.path.join(os.path.dirname(__file__), 'config.py')
        with open(path, encoding='utf-8') as f:
            text = f.read()
        vals = {
            'HOVER_THRUST': int(self.sl_thrust.value()),
            'TRIM_ROLL':    round(self.sl_roll.value(), 1),
            'TRIM_PITCH':   round(self.sl_pitch.value(), 1),
            'TRIM_YAW':     round(self.sl_yaw.value(), 1),
        }
        for key, val in vals.items():
            text = re.sub(rf'^({re.escape(key)}\s*=\s*)\S+',
                          rf'\g<1>{val}', text, flags=re.MULTILINE)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        self.status.setText(f"✓ Saved — Thrust:{vals['HOVER_THRUST']}  "
                            f"R:{vals['TRIM_ROLL']:+.1f}°  "
                            f"P:{vals['TRIM_PITCH']:+.1f}°  "
                            f"Y:{vals['TRIM_YAW']:+.1f}°/s")

    def _get_mtime(self):
        try:
            return os.path.getmtime(os.path.join(os.path.dirname(__file__), 'config.py'))
        except Exception:
            return 0

    def _hot_reload(self):
        mt = self._get_mtime()
        if mt <= self._cfg_mtime:
            return
        self._cfg_mtime = mt
        try:
            import importlib, config as _c
            importlib.reload(_c)
            self.sl_thrust.set_value(_c.HOVER_THRUST)
            self.sl_roll.set_value(_c.TRIM_ROLL)
            self.sl_pitch.set_value(_c.TRIM_PITCH)
            self.sl_yaw.set_value(_c.TRIM_YAW)
            self._push()
            self.status.setText("↺  config.py reloaded")
        except Exception as e:
            self.status.setText(f"Reload error: {e}")

    def _refresh(self):
        armed = self.drone.is_armed
        emerg = self.drone.is_emergency
        thr   = int(self.sl_thrust.value())
        if emerg:
            self.status.setText("⚠  EMERGENCY — motors cut")
            self.status.setStyleSheet('background:#3a1111; border-radius:8px; color:#ff6b6b; padding:4px;')
            self.btn_arm.setStyleSheet('background:#2a2a2a; color:#444; border-radius:8px;')
        elif armed:
            self.status.setText(f"✈  ARMED   Thrust: {thr}")
            self.status.setStyleSheet('background:#0d2b1a; border-radius:8px; color:#6bcb77; padding:4px;')
            self.btn_arm.setText("DISARM")
            self.btn_arm.setStyleSheet('background:#cc3333; color:white; border-radius:8px;')
        else:
            self.status.setText("○  DISARMED  —  press ARM to begin")
            self.status.setStyleSheet('background:#1a1a2e; border-radius:8px; color:#ffd93d; padding:4px;')
            self.btn_arm.setText("ARM")
            self.btn_arm.setStyleSheet('background:#22aa55; color:white; border-radius:8px;')

    def closeEvent(self, event):
        if self.drone.is_armed:
            self.drone.disarm()
            time.sleep(cfg.DESCENT_SECS + 0.3)
        self.drone.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = TunerWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
