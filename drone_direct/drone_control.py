"""
drone_direct/drone_control.py
==============================
EMG bump visualizer + live ESP-Drone controller.

Run directly:
  cd drone_direct
  python drone_control.py

Two-channel gesture detection:
  Right hand (Ch 1)
    1 bump  → GO UP      – step hover-thrust up by THRUST_STEP
    2 bumps → GO DOWN    – step hover-thrust down by THRUST_STEP

  Left hand (Ch 2)
    1 bump  → GO DOWN    – step hover-thrust down by THRUST_STEP
    2 bumps → EMERGENCY  – immediate motor cutoff

Drone panel (bottom):
  [EMG CONTROL]  [ARM]  [SOFT LAND]  [EMERGENCY STOP]   Thrust ±   Status badge
"""

import socket
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSlider, QGroupBox,
    QFrame, QScrollArea,
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
from pylsl import StreamInlet, resolve_streams
from scipy.signal import butter, filtfilt

# ── Import drone config ───────────────────────────────────────────────────────
_TUNER = Path(__file__).parent / 'tuner'
sys.path.insert(0, str(_TUNER))

try:
    import config as drone_cfg
    DRONE_AVAILABLE = True
except Exception as _imp_err:
    DRONE_AVAILABLE = False
    print(f"[drone_control] Drone config unavailable: {_imp_err}")


# ── Drone protocol ────────────────────────────────────────────────────────────
def _cksum(data: bytes) -> int:
    return sum(data) & 0xFF


def make_rpyt(roll: float, pitch: float, yaw: float, thrust: int) -> bytes:
    t       = max(0, min(65535, int(thrust)))
    payload = struct.pack('<BfffH', 0x30, float(roll), float(pitch), float(yaw), t)
    return payload + bytes([_cksum(payload)])


class DroneLink(threading.Thread):
    """Minimal UDP sender thread — no camera stabilizer."""

    def __init__(self):
        super().__init__(daemon=True)
        self.sock       = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock      = threading.Lock()
        self._armed     = False
        self._emergency = False
        self._landing   = False
        self._running   = True
        self._thrust    = drone_cfg.HOVER_THRUST if DRONE_AVAILABLE else 40000
        self._roll      = drone_cfg.TRIM_ROLL    if DRONE_AVAILABLE else 6.325
        self._pitch     = drone_cfg.TRIM_PITCH   if DRONE_AVAILABLE else 0.885
        self._yaw       = drone_cfg.TRIM_YAW     if DRONE_AVAILABLE else 0.0

    def _send(self, roll, pitch, yaw, thrust):
        try:
            self.sock.sendto(
                make_rpyt(roll, pitch, yaw, thrust),
                (drone_cfg.DRONE_IP, drone_cfg.DRONE_PORT))
        except OSError:
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

    def soft_land(self):
        threading.Thread(target=self._soft_land_thread, daemon=True).start()

    def emergency(self):
        with self._lock:
            self._emergency = True
            self._armed     = False
            self._landing   = False

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

    @property
    def is_landing(self):
        with self._lock:
            return self._landing

    def _ramp_arm(self):
        self._send(0, 0, 0, 0)
        time.sleep(drone_cfg.UNLOCK_DURATION)
        steps = max(10, int(drone_cfg.THRUST_RAMP_TIME * drone_cfg.SEND_HZ))
        with self._lock:
            target       = max(drone_cfg.THRUST_MIN, self._thrust)
            self._thrust = 0
            self._armed  = True
        for i in range(steps):
            with self._lock:
                self._thrust = int(target * (i + 1) / steps)
            time.sleep(drone_cfg.THRUST_RAMP_TIME / steps)
        with self._lock:
            self._thrust = target

    def _ramp_disarm(self):
        with self._lock:
            from_t = self._thrust
        delay = drone_cfg.DESCENT_SECS / drone_cfg.DESCENT_STEPS
        for i in range(drone_cfg.DESCENT_STEPS):
            with self._lock:
                self._thrust = max(
                    0, int(from_t * (1 - (i + 1) / drone_cfg.DESCENT_STEPS)))
            time.sleep(delay)
        with self._lock:
            self._thrust = 0
            self._armed  = False

    def _soft_land_thread(self):
        LAND_SECS  = 3
        LAND_STEPS = 100
        LAND_CURVE = 0.60
        with self._lock:
            if not self._armed or self._emergency:
                return
            self._landing = True
            from_t = self._thrust
        delay = LAND_SECS / LAND_STEPS
        for i in range(LAND_STEPS):
            with self._lock:
                if self._emergency:
                    self._landing = False
                    return
                progress     = (i + 1) / LAND_STEPS
                self._thrust = max(0, int(from_t * (1 - progress ** LAND_CURVE)))
            time.sleep(delay)
        with self._lock:
            self._thrust  = 0
            self._armed   = False
            self._landing = False

    def run(self):
        interval = 1.0 / drone_cfg.SEND_HZ
        cur_r = cur_p = cur_y = cur_t = 0.0
        while self._running:
            with self._lock:
                armed = self._armed
                emerg = self._emergency
                tgt_t = float(max(drone_cfg.THRUST_MIN,
                                  min(drone_cfg.THRUST_MAX, self._thrust)))
                tgt_r = self._roll
                tgt_p = self._pitch
                tgt_y = self._yaw
            cur_r += (tgt_r - cur_r) * 0.08
            cur_p += (tgt_p - cur_p) * 0.08
            cur_y += (tgt_y - cur_y) * 0.08
            cur_t += (tgt_t - cur_t) * 0.15
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


# ── EMG constants ─────────────────────────────────────────────────────────────
DISPLAY_SECS  = 4.0
N_CHANNELS    = 2

CH_COLORS     = ['#ffd93d', '#6bcb77']
CH_DIM_COLORS = ['#3d3000', '#003318']
CH_NAMES      = ['Right Hand  (Ch 1)', 'Left Hand  (Ch 2)']

CH_ACTIONS = [
    # right hand (Ch 1)
    {1: ('GO UP',      '#6bcb77'),
     2: ('GO DOWN',    '#ff9f43')},
    # left hand (Ch 2)
    {1: ('EMERGENCY',  '#ff4444'),
     3: ('EMG TOGGLE', '#ffd93d')},
]

COOLDOWN_RATIO = 0.25
MAX_HISTORY    = 15

# ── Drone action constants ────────────────────────────────────────────────────
THRUST_STEP = 4000    # hover-thrust increment per GO UP / GO DOWN gesture


# ── Per-channel bump-group state machine ──────────────────────────────────────
class ChannelDetector:
    VALLEY_RATIO = 0.45

    def __init__(self):
        self.state               = 'idle'
        self._in_peak            = False
        self._had_valley         = True
        self.peak_count          = 0
        self._window_end         = 0
        self._sample_idx         = 0
        self._group_start_sample = 0
        self.last_action         = 0
        self.action_fired        = False

    @property
    def samples_since_group_start(self) -> int:
        return self._sample_idx - self._group_start_sample

    def process(self, disp_val: float, threshold: float,
                window_samples: int) -> bool:
        self._sample_idx += 1
        idx   = self._sample_idx
        above = disp_val > threshold
        deep  = disp_val < threshold * self.VALLEY_RATIO

        if self.state == 'idle':
            if not above:
                self._in_peak = False
                if deep:
                    self._had_valley = True
            elif above and not self._in_peak and self._had_valley:
                self._in_peak            = True
                self._had_valley         = False
                self.peak_count          = 1
                self._group_start_sample = idx
                self._window_end         = idx + window_samples
                self.state               = 'grouping'
            return False

        if self.state == 'grouping':
            if idx > self._window_end:
                self.last_action  = self.peak_count
                self.action_fired = True
                self.state        = 'cooldown'
                self._in_peak     = False
                self._had_valley  = False
                return True
            if not above:
                self._in_peak = False
                if deep:
                    self._had_valley = True
            elif above and not self._in_peak and self._had_valley:
                self._in_peak    = True
                self._had_valley = False
                self.peak_count += 1
            return False

        # cooldown
        if disp_val < threshold * COOLDOWN_RATIO:
            self.state       = 'idle'
            self._in_peak    = False
            self._had_valley = True
            return False
        return True


# ── Main window ───────────────────────────────────────────────────────────────
class DroneControlApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EMG Drone Control")
        self.resize(1200, 860)

        # ── EMG state ─────────────────────────────────────────────────────────
        self.inlet         = None
        self.sampling_rate = 500

        self.total_samples   = int(DISPLAY_SECS * self.sampling_rate)
        self.process_window  = 1.0
        self.process_samples = int(self.process_window * self.sampling_rate)
        self.thresh          = [400.0, 400.0]

        self.raw_data    = np.zeros((N_CHANNELS, self.total_samples))
        self.env_data    = np.zeros((N_CHANNELS, self.total_samples))
        self.baseline    = np.zeros(N_CHANNELS)
        self.ignore_mask = np.zeros((N_CHANNELS, self.total_samples), dtype=bool)

        self.detectors = [ChannelDetector(), ChannelDetector()]

        # ── Drone state ────────────────────────────────────────────────────────
        self._drone        = None
        self._base_thrust  = (drone_cfg.HOVER_THRUST
                              if DRONE_AVAILABLE else 39209)
        self._emg_enabled  = False

        self._init_ui()
        self._connect_lsl()
        self._connect_drone()

        self.timer = QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(16)   # ~60 Hz

    # ── UI ────────────────────────────────────────────────────────────────────
    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        central.setStyleSheet(
            "background:#111120; color:#ddd; font-family:'Segoe UI';")

        outer = QHBoxLayout(central)
        outer.setSpacing(8)
        outer.setContentsMargins(10, 8, 6, 8)

        left = QWidget()
        root = QVBoxLayout(left)
        root.setSpacing(6)
        root.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(left, stretch=1)
        outer.addWidget(self._build_history_panel())

        # Status
        self.lbl_status = QLabel("Connecting to LSL…")
        self.lbl_status.setFont(QFont('Segoe UI', 12, QFont.Bold))
        self.lbl_status.setStyleSheet("color:gray;")
        root.addWidget(self.lbl_status)

        # EMG controls
        ctrl = QHBoxLayout()
        ctrl.setSpacing(10)
        self._thresh_lbls = []
        for ch in range(N_CHANNELS):
            grp = QGroupBox(f"Ch{ch + 1} Threshold  (0 – 3000)")
            grp.setStyleSheet(self._grp_css())
            gl = QHBoxLayout(grp)
            lbl = QLabel(f"{int(self.thresh[ch])}")
            lbl.setFixedWidth(46)
            sl = QSlider(Qt.Horizontal)
            sl.setRange(0, 3000)
            sl.setValue(int(self.thresh[ch]))
            sl.valueChanged.connect(lambda v, c=ch: self._on_thresh(c, v))
            gl.addWidget(sl)
            gl.addWidget(lbl)
            self._thresh_lbls.append(lbl)
            ctrl.addWidget(grp, stretch=2)

        wg = QGroupBox("Analysis Window")
        wg.setStyleSheet(self._grp_css())
        wl = QHBoxLayout(wg)
        self.lbl_window = QLabel(f"{self.process_window:.1f} s")
        self.lbl_window.setFixedWidth(46)
        self.sl_window = QSlider(Qt.Horizontal)
        self.sl_window.setRange(3, 40)
        self.sl_window.setValue(10)
        self.sl_window.valueChanged.connect(self._on_window)
        wl.addWidget(self.sl_window)
        wl.addWidget(self.lbl_window)
        ctrl.addWidget(wg, stretch=2)

        btn_bl = QPushButton("Reset Baseline")
        btn_bl.setFixedHeight(38)
        btn_bl.setStyleSheet(
            "background:#1a3a5c; color:#ddd; border-radius:6px; font-size:11px;")
        btn_bl.clicked.connect(self._reset_baseline)
        ctrl.addWidget(btn_bl)
        root.addLayout(ctrl)

        # Plots
        pg.setConfigOption('background', '#111120')
        pg.setConfigOption('foreground', '#888')

        self.plots          = []
        self.curves_active  = []
        self.curves_dim     = []
        self.thresh_lines   = []
        self.window_regions = []

        for ch in range(N_CHANNELS):
            pw = pg.PlotWidget()
            pw.setTitle(f"  {CH_NAMES[ch]}", color=CH_COLORS[ch])
            pw.showGrid(x=True, y=True, alpha=0.2)
            pw.setXRange(0, self.total_samples)
            pw.setYRange(0, 1000)
            pw.setMinimumHeight(210)

            wr = pg.LinearRegionItem(
                values=[self.total_samples - self.process_samples,
                        self.total_samples],
                movable=False,
                brush=pg.mkBrush(80, 100, 255, 20),
            )
            wr.lines[0].setPen(pg.mkPen('#4466dd', width=1.5, style=Qt.DashLine))
            wr.lines[1].setPen(pg.mkPen(None))
            wr.setZValue(-10)
            pw.addItem(wr)
            self.window_regions.append(wr)

            c_dim = pw.plot(pen=pg.mkPen(color=CH_DIM_COLORS[ch], width=2))
            c_act = pw.plot(pen=pg.mkPen(color=CH_COLORS[ch], width=2))

            tl = pg.InfiniteLine(
                angle=0,
                pen=pg.mkPen('#ff4444', width=1.5, style=Qt.DashLine))
            tl.setValue(self.thresh[ch])
            pw.addItem(tl)

            self.plots.append(pw)
            self.curves_active.append(c_act)
            self.curves_dim.append(c_dim)
            self.thresh_lines.append(tl)
            root.addWidget(pw, stretch=1)

        # Last-action row
        act_row = QHBoxLayout()
        act_row.setSpacing(12)
        self.lbl_actions = []
        for ch in range(N_CHANNELS):
            short = CH_NAMES[ch].split('  ')[0]
            lbl = QLabel(f"{short}:  —")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setFont(QFont('Segoe UI', 15, QFont.Bold))
            lbl.setFixedHeight(44)
            lbl.setStyleSheet(
                "background:#1a1a2e; border-radius:8px; color:#444; padding:4px;")
            self.lbl_actions.append(lbl)
            act_row.addWidget(lbl)
        root.addLayout(act_row)

        # Drone control panel
        root.addWidget(self._build_drone_panel())

    # ── History panel ─────────────────────────────────────────────────────────
    def _build_history_panel(self) -> QWidget:
        panel = QWidget()
        panel.setFixedWidth(200)
        panel.setStyleSheet("background:#0d0d1a; border-radius:10px;")
        vl = QVBoxLayout(panel)
        vl.setSpacing(4)
        vl.setContentsMargins(8, 8, 8, 8)

        title = QLabel("Action History")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont('Segoe UI', 11, QFont.Bold))
        title.setStyleSheet("color:#555; background:transparent; padding-bottom:4px;")
        vl.addWidget(title)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("background:#222; max-height:1px; border:none;")
        vl.addWidget(line)

        hdr = QHBoxLayout()
        for name, color in [('Right', CH_COLORS[0]), ('Left', CH_COLORS[1])]:
            h = QLabel(name)
            h.setAlignment(Qt.AlignCenter)
            h.setFont(QFont('Segoe UI', 9, QFont.Bold))
            h.setStyleSheet(f"color:{color}; background:transparent; padding:4px 0;")
            hdr.addWidget(h)
        vl.addLayout(hdr)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet("background:transparent; border:none;")

        inner = QWidget()
        inner.setStyleSheet("background:transparent;")
        cols = QHBoxLayout(inner)
        cols.setSpacing(4)
        cols.setContentsMargins(0, 0, 0, 0)

        self._hist_layouts = []
        for _ in range(N_CHANNELS):
            col = QWidget()
            col.setStyleSheet("background:transparent;")
            col_layout = QVBoxLayout(col)
            col_layout.setSpacing(3)
            col_layout.setContentsMargins(0, 0, 0, 0)
            col_layout.setAlignment(Qt.AlignTop)
            self._hist_layouts.append(col_layout)
            cols.addWidget(col)

        scroll.setWidget(inner)
        vl.addWidget(scroll, stretch=1)
        return panel

    def _add_history(self, ch: int, label: str, color: str):
        layout = self._hist_layouts[ch]
        pill = QLabel(label)
        pill.setAlignment(Qt.AlignCenter)
        pill.setFont(QFont('Segoe UI', 8, QFont.Bold))
        pill.setFixedHeight(22)
        pill.setStyleSheet(
            f"color:{color}; background:#15152a; "
            f"border:1px solid {color}; border-radius:4px;")
        layout.insertWidget(0, pill)
        if layout.count() > MAX_HISTORY:
            item = layout.takeAt(layout.count() - 1)
            if item and item.widget():
                item.widget().deleteLater()

    # ── Drone control panel ───────────────────────────────────────────────────
    def _build_drone_panel(self) -> QWidget:
        grp = QGroupBox("Drone Control")
        grp.setStyleSheet(
            "QGroupBox { border:1px solid #2a3a5a; border-radius:8px; "
            "margin-top:10px; padding:6px; font-size:10px; color:#4466dd; }"
            "QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 3px; }")
        row = QHBoxLayout(grp)
        row.setSpacing(10)

        # EMG CONTROL toggle
        self.btn_emg = QPushButton("EMG CONTROL: OFF")
        self.btn_emg.setFixedHeight(40)
        self.btn_emg.setCheckable(True)
        self.btn_emg.setChecked(False)
        self.btn_emg.setStyleSheet(self._drone_btn_css('#2a1a2a', '#888'))
        self.btn_emg.clicked.connect(self._on_emg_toggle)
        row.addWidget(self.btn_emg)

        row.addWidget(self._sep())

        # ARM
        self.btn_arm = QPushButton("ARM")
        self.btn_arm.setFixedHeight(40)
        self.btn_arm.setStyleSheet(self._drone_btn_css('#1a5c1a', '#6bcb77'))
        self.btn_arm.clicked.connect(self._on_arm)
        row.addWidget(self.btn_arm)

        # SOFT LAND
        self.btn_land = QPushButton("SOFT LAND")
        self.btn_land.setFixedHeight(40)
        self.btn_land.setStyleSheet(self._drone_btn_css('#1a3a5c', '#00bcd4'))
        self.btn_land.clicked.connect(self._on_soft_land)
        row.addWidget(self.btn_land)

        # EMERGENCY
        self.btn_emerg = QPushButton("EMERGENCY STOP")
        self.btn_emerg.setFixedHeight(40)
        self.btn_emerg.setStyleSheet(self._drone_btn_css('#5c1a1a', '#ff4444'))
        self.btn_emerg.clicked.connect(self._on_emergency)
        row.addWidget(self.btn_emerg)

        row.addStretch()

        # Thrust display + nudge buttons
        row.addWidget(self._sep())
        thrust_box = QVBoxLayout()
        self.lbl_thrust = QLabel(f"Thrust: {self._base_thrust}")
        self.lbl_thrust.setFont(QFont('Segoe UI', 11, QFont.Bold))
        self.lbl_thrust.setStyleSheet("color:#ffd93d;")
        thrust_box.addWidget(self.lbl_thrust)
        nudge_row = QHBoxLayout()
        for delta, label in [(-THRUST_STEP, f"−{THRUST_STEP//1000}k"),
                              (+THRUST_STEP, f"+{THRUST_STEP//1000}k")]:
            b = QPushButton(label)
            b.setFixedSize(52, 28)
            b.setStyleSheet(
                "background:#1e1e2e; color:#ddd; border:1px solid #333; "
                "border-radius:4px; font-size:10px;")
            b.clicked.connect(lambda _chk, d=delta: self._adjust_thrust(d))
            nudge_row.addWidget(b)
        thrust_box.addLayout(nudge_row)
        row.addLayout(thrust_box)

        row.addWidget(self._sep())

        # Status badge
        self.lbl_drone_status = QLabel("● OFFLINE")
        self.lbl_drone_status.setFont(QFont('Segoe UI', 13, QFont.Bold))
        self.lbl_drone_status.setStyleSheet("color:#555;")
        self.lbl_drone_status.setAlignment(Qt.AlignVCenter)
        row.addWidget(self.lbl_drone_status)

        if not DRONE_AVAILABLE:
            for btn in (self.btn_arm, self.btn_land, self.btn_emerg):
                btn.setEnabled(False)
                btn.setToolTip("tuner/config.py not found")
            self.lbl_drone_status.setText("● BACKEND MISSING")
            self.lbl_drone_status.setStyleSheet("color:#ff4444;")

        return grp

    @staticmethod
    def _sep() -> QFrame:
        f = QFrame()
        f.setFrameShape(QFrame.VLine)
        f.setStyleSheet("color:#2a2a3e;")
        return f

    @staticmethod
    def _drone_btn_css(bg: str, fg: str) -> str:
        return (f"QPushButton {{ background:{bg}; color:{fg}; "
                f"border:1px solid {fg}55; border-radius:6px; font-size:12px; "
                f"font-weight:bold; }}"
                f"QPushButton:hover {{ background:{fg}33; }}"
                f"QPushButton:disabled {{ background:#1a1a2e; color:#333; border-color:#222; }}")

    @staticmethod
    def _grp_css() -> str:
        return (
            "QGroupBox { border:1px solid #2a2a3e; border-radius:8px; "
            "margin-top:10px; padding:4px; font-size:10px; color:#555; }"
            "QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 3px; }")

    # ── Drone button handlers ─────────────────────────────────────────────────
    def _on_emg_toggle(self):
        # Called when the button is clicked (Qt already toggled checked state)
        self._emg_enabled = self.btn_emg.isChecked()
        self._refresh_emg_btn()

    def _toggle_emg(self):
        """Toggle EMG control (called by 3× left-hand flex)."""
        self._emg_enabled = not self._emg_enabled
        self.btn_emg.setChecked(self._emg_enabled)
        self._refresh_emg_btn()

    def _refresh_emg_btn(self):
        if self._emg_enabled:
            self.btn_emg.setText("EMG CONTROL: ON")
            self.btn_emg.setStyleSheet(self._drone_btn_css('#1a3a1a', '#6bcb77'))
        else:
            self.btn_emg.setText("EMG CONTROL: OFF")
            self.btn_emg.setStyleSheet(self._drone_btn_css('#2a1a2a', '#888'))

    def _on_arm(self):
        if self._drone and not self._drone.is_armed:
            self._drone.set_params(
                self._base_thrust,
                drone_cfg.TRIM_ROLL,
                drone_cfg.TRIM_PITCH,
                drone_cfg.TRIM_YAW,
            )
            self._drone.arm()

    def _on_soft_land(self):
        if self._drone:
            self._drone.soft_land()

    def _on_emergency(self):
        if not self._drone:
            return
        if self._drone.is_emergency:
            self._drone.clear_emergency()
            self.btn_emerg.setText("EMERGENCY STOP")
            self.btn_emerg.setStyleSheet(self._drone_btn_css('#5c1a1a', '#ff4444'))
        else:
            self._drone.emergency()
            self.btn_emerg.setText("CLEAR EMERGENCY")
            self.btn_emerg.setStyleSheet(self._drone_btn_css('#555', '#aaa'))
            # EMG control stays enabled — disable only via 3× left-hand flex

    def _adjust_thrust(self, delta: int):
        if not DRONE_AVAILABLE:
            return
        lo = drone_cfg.THRUST_MIN
        hi = drone_cfg.THRUST_MAX
        self._base_thrust = max(lo, min(hi, self._base_thrust + delta))
        self.lbl_thrust.setText(f"Thrust: {self._base_thrust}")
        if self._drone and self._drone.is_armed:
            self._drone.set_params(
                self._base_thrust,
                drone_cfg.TRIM_ROLL,
                drone_cfg.TRIM_PITCH,
                drone_cfg.TRIM_YAW,
            )

    # ── Slider callbacks ──────────────────────────────────────────────────────
    def _on_thresh(self, ch, val):
        self.thresh[ch] = float(val)
        self._thresh_lbls[ch].setText(str(val))
        self.thresh_lines[ch].setValue(float(val))

    def _on_window(self, val):
        self.process_window  = val * 0.1
        self.process_samples = int(self.process_window * self.sampling_rate)
        self.lbl_window.setText(f"{self.process_window:.1f} s")
        for wr in self.window_regions:
            wr.setRegion(
                [self.total_samples - self.process_samples, self.total_samples])

    def _reset_baseline(self):
        for ch in range(N_CHANNELS):
            self.baseline[ch] = float(np.mean(self.env_data[ch]))

    # ── LSL ───────────────────────────────────────────────────────────────────
    def _connect_lsl(self):
        try:
            streams = resolve_streams(wait_time=1.0)
        except Exception as e:
            streams = []
            print(f"[drone_control] LSL resolve error: {e}")

        if streams:
            try:
                self.inlet = StreamInlet(streams[0])
                sr = int(self.inlet.info().nominal_srate())
                if sr > 0:
                    self.sampling_rate   = sr
                    self.total_samples   = int(DISPLAY_SECS * sr)
                    self.process_samples = int(self.process_window * sr)
                    self.raw_data    = np.zeros((N_CHANNELS, self.total_samples))
                    self.env_data    = np.zeros((N_CHANNELS, self.total_samples))
                    self.ignore_mask = np.zeros(
                        (N_CHANNELS, self.total_samples), dtype=bool)
                    for pw in self.plots:
                        pw.setXRange(0, self.total_samples)
                self.lbl_status.setText(
                    f"Connected: {self.inlet.info().name()}  "
                    f"({self.sampling_rate} Hz)")
                self.lbl_status.setStyleSheet(
                    "color:#6bcb77; font-size:12px; font-weight:bold;")
            except Exception as e:
                print(f"[drone_control] LSL inlet error: {e}")
                self.lbl_status.setText("LSL error — demo mode")
                self.lbl_status.setStyleSheet(
                    "color:#ff9800; font-size:12px; font-weight:bold;")
        else:
            self.lbl_status.setText("LSL not found — demo mode")
            self.lbl_status.setStyleSheet(
                "color:#ff6b6b; font-size:12px; font-weight:bold;")

    # ── Drone ─────────────────────────────────────────────────────────────────
    def _connect_drone(self):
        if not DRONE_AVAILABLE:
            return
        try:
            self._drone = DroneLink()
            self._drone.start()
            ip   = drone_cfg.DRONE_IP
            port = drone_cfg.DRONE_PORT
            self.lbl_drone_status.setText(f"● {ip}:{port}")
            self.lbl_drone_status.setStyleSheet(
                "color:#4466dd; font-size:11px; font-weight:bold;")
        except Exception as e:
            print(f"[drone_control] DroneLink init error: {e}")
            self._drone = None

    # ── Signal processing ─────────────────────────────────────────────────────
    def _make_envelope(self, raw: np.ndarray) -> np.ndarray:
        if len(raw) < 10:
            return np.abs(raw)
        nyq = self.sampling_rate / 2.0
        b, a = butter(4, 5.0 / nyq, btype='high')
        hp   = filtfilt(b, a, raw)
        win  = max(2, int(0.04 * self.sampling_rate))
        rms  = np.sqrt(np.convolve(hp ** 2, np.ones(win) / win, mode='valid'))
        return np.pad(rms, (len(raw) - len(rms), 0), 'constant')

    # ── Main update loop ──────────────────────────────────────────────────────
    def _update(self):
        if self.inlet:
            samples, _ = self.inlet.pull_chunk(timeout=0.0, max_samples=32)
        else:
            samples = [[np.random.randn() * 60, np.random.randn() * 60]
                       for _ in range(4)]

        n = len(samples)

        if n:
            for s in samples:
                for ch in range(N_CHANNELS):
                    self.raw_data[ch] = np.roll(self.raw_data[ch], -1)
                    val = float(s[ch]) if ch < len(s) else 0.0
                    self.raw_data[ch, -1] = val
            self.ignore_mask = np.roll(self.ignore_mask, -n, axis=1)
            self.ignore_mask[:, -n:] = False

        for ch in range(N_CHANNELS):
            self.env_data[ch] = self._make_envelope(self.raw_data[ch])
            recent = float(np.mean(
                self.env_data[ch, -int(0.2 * self.sampling_rate):]))
            if recent < self.baseline[ch] + self.thresh[ch] * 0.4:
                self.baseline[ch] += 0.002 * (
                    float(self.env_data[ch, -1]) - self.baseline[ch])

        if n:
            for i in range(n):
                buf_pos = self.total_samples - n + i
                for ch in range(N_CHANNELS):
                    raw_env  = float(self.env_data[ch][buf_pos])
                    disp_val = max(0.0, raw_env - self.baseline[ch])

                    ignored = self.detectors[ch].process(
                        disp_val, self.thresh[ch], self.process_samples)
                    self.ignore_mask[ch, buf_pos] = ignored

                    if self.detectors[ch].action_fired:
                        self.detectors[ch].action_fired = False
                        self._show_action(ch, self.detectors[ch].last_action)
                        span  = self.detectors[ch].samples_since_group_start
                        retro = max(0, buf_pos - span)
                        self.ignore_mask[ch, retro : buf_pos + 1] = True

        for ch in range(N_CHANNELS):
            disp = np.maximum(0.0, self.env_data[ch] - self.baseline[ch])
            mask = self.ignore_mask[ch]
            act  = disp.copy(); act[mask]  = np.nan
            dim  = disp.copy(); dim[~mask] = np.nan
            self.curves_active[ch].setData(act)
            self.curves_dim[ch].setData(dim)
            peak = float(np.nanmax(disp)) if np.any(np.isfinite(disp)) else 0.0
            self.plots[ch].setYRange(0, min(5000.0, max(peak * 1.2, 1000.0)))

        self._update_drone_status()

    def _update_drone_status(self):
        if self._drone is None:
            return
        if self._drone.is_emergency:
            text, color = "● EMERGENCY", "#ff4444"
        elif self._drone.is_landing:
            text, color = "● LANDING", "#ff9f43"
        elif self._drone.is_armed:
            text, color = "● ARMED", "#6bcb77"
        else:
            ip = drone_cfg.DRONE_IP if DRONE_AVAILABLE else "—"
            text, color = f"● DISARMED  {ip}", "#4466dd"
        self.lbl_drone_status.setText(text)
        self.lbl_drone_status.setStyleSheet(
            f"color:{color}; font-size:12px; font-weight:bold;")

    # ── Action classification → label → drone command ─────────────────────────
    def _show_action(self, ch: int, peak_count: int):
        actions = CH_ACTIONS[ch]
        max_key = max(k for k in actions if k > 0)
        count   = min(peak_count, max_key)
        label, color = actions.get(count, (f'{peak_count}× FLEX', '#aaa'))

        short = CH_NAMES[ch].split('  ')[0]
        self.lbl_actions[ch].setText(f"{short}:  {label}")
        self.lbl_actions[ch].setStyleSheet(
            f"background:#1a1a2e; border-radius:8px; color:{color}; "
            f"padding:4px; font-size:15px; font-weight:bold;")
        self._add_history(ch, label, color)
        if label == 'EMG TOGGLE' or self._emg_enabled:
            self._drone_command(label)

    def _drone_command(self, label: str):
        if label == 'EMG TOGGLE':
            self._toggle_emg()
            return
        if self._drone is None:
            return
        if label == 'GO UP':
            if not self._drone.is_armed:
                self._on_arm()
            else:
                self._adjust_thrust(+THRUST_STEP)
        elif label == 'GO DOWN':
            self._adjust_thrust(-THRUST_STEP)
        elif label == 'EMERGENCY':
            self._on_emergency()

    # ── Clean shutdown ────────────────────────────────────────────────────────
    def closeEvent(self, event):
        if self._drone:
            self._drone.emergency()
            self._drone.stop()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    ex = DroneControlApp()
    ex.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
