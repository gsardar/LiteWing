"""
Neurotech: 3D Drone Visualizer
Listens on UDP 127.0.0.1:1235 for telemetry from arm_controller.py
Protocol: "X,Y,Z,MODE"   e.g. "0.12,-0.05,0.45,FIST (MACRO)"
"""

import socket
import threading
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
from collections import deque

# ─── Shared state ────────────────────────────────────────────────────────────
state = {
    "x": 0.0, "y": 0.0, "z": 0.0,
    "mode": "RELAXED",
    "roll": 0.0, "pitch": 0.0   # future: from drone sensor
}
trail_xyz = deque(maxlen=80)  # 3-D flight trail

# ─── UDP listener (runs in background thread) ────────────────────────────────
def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", 1235))
    sock.settimeout(0.1)
    print("[Visualizer] Listening on UDP 127.0.0.1:1235")
    while True:
        try:
            data, _ = sock.recvfrom(256)
            parts = data.decode().strip().split(",", 3)
            if len(parts) >= 3:
                state["x"]    = float(parts[0])
                state["y"]    = float(parts[1])
                state["z"]    = float(parts[2])
                state["mode"] = parts[3].strip() if len(parts) > 3 else "RELAXED"
        except socket.timeout:
            pass
        except Exception as e:
            print(f"[UDP] {e}")

threading.Thread(target=udp_listener, daemon=True).start()

# ─── Keyboard override (arrow keys for testing without arm) ──────────────────
kb_override = {"x": 0.0, "y": 0.0, "z": 0.0, "mode": None}

def on_key(event):
    STEP = 0.15
    if event.key == 'right':  kb_override["x"] = min(kb_override["x"] + STEP, 1.0);  kb_override["mode"] = "FIST (MACRO)"
    elif event.key == 'left': kb_override["x"] = max(kb_override["x"] - STEP, -1.0); kb_override["mode"] = "FIST (MACRO)"
    elif event.key == 'up':   kb_override["y"] = min(kb_override["y"] + STEP, 1.0);  kb_override["mode"] = "FIST (MACRO)"
    elif event.key == 'down': kb_override["y"] = max(kb_override["y"] - STEP, -1.0); kb_override["mode"] = "FIST (MACRO)"
    elif event.key == 'w':    kb_override["z"] = min(kb_override["z"] + STEP, 1.0);  kb_override["mode"] = "PINCH (MICRO)"
    elif event.key == 's':    kb_override["z"] = max(kb_override["z"] - STEP, -0.4); kb_override["mode"] = "PINCH (MICRO)"
    elif event.key == 'q':    plt.close('all')
    elif event.key == 'r':    kb_override.update({"x":0,"y":0,"z":0,"mode":None}); trail_xyz.clear()

# ─── Drone geometry helpers ───────────────────────────────────────────────────
ARM_LEN  = 0.3   # arm half-length (world units)
ROTOR_R  = 0.12  # rotor disc radius
BODY_LEN = 0.18  # body half-extent
ROTOR_CIRCLE = np.linspace(0, 2*np.pi, 36)

ROTOR_OFFSETS = [
    ( ARM_LEN,  ARM_LEN, 0),   # front-right
    (-ARM_LEN,  ARM_LEN, 0),   # front-left
    (-ARM_LEN, -ARM_LEN, 0),   # rear-left
    ( ARM_LEN, -ARM_LEN, 0),   # rear-right
]

def Rz(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def Rx(pitch):
    c, s = np.cos(pitch), np.sin(pitch)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def Ry(roll):
    c, s = np.cos(roll), np.sin(roll)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rot_matrix(roll, pitch, yaw):
    return Rz(yaw) @ Ry(roll) @ Rx(pitch)

def draw_drone(ax, cx, cy, cz, roll=0, pitch=0, mode="RELAXED"):
    R = rot_matrix(roll, pitch, 0)

    # Body cube vertices
    b = BODY_LEN
    corners = np.array([ [b,b,0.04],[b,-b,0.04],[-b,-b,0.04],[-b,b,0.04],
                          [b,b,-0.04],[b,-b,-0.04],[-b,-b,-0.04],[-b,b,-0.04] ])
    rc = (R @ corners.T).T
    rx, ry, rz = rc[:,0]+cx, rc[:,1]+cy, rc[:,2]+cz
    faces = [[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[1,2,6,5],[0,3,7,4]]
    body_verts = [[ [rx[i],ry[i],rz[i]] for i in f ] for f in faces]
    color = (0.2,0.8,0.2) if "FIST" in mode else (0.8,0.2,0.2) if "PINCH" in mode else (0.4,0.6,1.0)
    body = Poly3DCollection(body_verts, alpha=0.8, facecolor=color, edgecolor='k', linewidth=0.5)
    ax.add_collection3d(body)

    # Arms + rotors
    for ox, oy, oz in ROTOR_OFFSETS:
        tip = R @ np.array([ox, oy, oz])
        # Arm
        ax.plot([cx, cx+tip[0]], [cy, cy+tip[1]], [cz, cz+tip[2]], 'k-', lw=2.5)
        # Rotor disc
        rc_x = cx + tip[0] + ROTOR_R * np.cos(ROTOR_CIRCLE)
        rc_y = cy + tip[1] + ROTOR_R * np.sin(ROTOR_CIRCLE)
        rc_z = np.full_like(rc_x, cz + tip[2])
        ax.plot(rc_x, rc_y, rc_z, color='gray', lw=1.2, alpha=0.7)
        ax.plot_surface(
            np.array([rc_x, rc_x]), np.array([rc_y, rc_y]),
            np.array([rc_z-0.01, rc_z+0.01]),
            alpha=0.25, color='silver', shade=False
        )

    # Landing legs
    for ox, oy in [(0.12,0.12),(0.12,-0.12),(-0.12,0.12),(-0.12,-0.12)]:
        tip = R @ np.array([ox, oy, 0])
        leg = R @ np.array([ox, oy, -0.1])
        ax.plot([cx+tip[0], cx+leg[0]], [cy+tip[1], cy+leg[1]], [cz+tip[2], cz+leg[2]], 'k-', lw=1.5)


# ─── Figure setup ────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(10, 8), facecolor='#0d0d1a')
fig.canvas.manager.set_window_title("NEURO-LINK: 3D Drone Control")

# Remove matplotlib default keybindings that conflict with arrow keys
for key in ['keymap.back', 'keymap.forward', 'keymap.left', 'keymap.right', 'keymap.up', 'keymap.down']:
    if key in plt.rcParams:
        plt.rcParams[key] = []

ax  = fig.add_subplot(111, projection='3d')

WORLD = 1.5
ax.set_xlim(-WORLD, WORLD)
ax.set_ylim(-WORLD, WORLD)
ax.set_zlim(0, WORLD * 2)
ax.set_facecolor('#0d0d1a')
fig.patch.set_facecolor('#0d0d1a')
ax.set_xlabel('X  (lateral)', color='#88ccff')
ax.set_ylabel('Y  (forward)', color='#88ccff')
ax.set_zlabel('Z  (altitude)', color='#88ccff')
ax.tick_params(colors='#88ccff')
ax.xaxis.pane.fill = ax.yaxis.pane.fill = ax.zaxis.pane.fill = True
for pane in [ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane]:
    pane.set_facecolor('#101025')

# Ground grid
gx = gy = np.arange(-WORLD, WORLD+0.5, 0.5)
GX, GY = np.meshgrid(gx, gy)
GZ = np.zeros_like(GX)
ax.plot_wireframe(GX, GY, GZ, color='#224466', linewidth=0.4, alpha=0.5)

title_obj = ax.set_title("NEURO-LINK: 3D Drone Control  |  WAITING...", color='white', fontsize=13, pad=12)

# ─── Drone position state (persists between frames) ──────────────────────────
drone_pos = [0.0, 0.0, 0.7]   # [x, y, z]
STEP_SIZE = 0.04               # how far FIST moves per frame

DIRECTIONS = {
    "RIGHT": ( 1,  0),
    "LEFT":  (-1,  0),
    "UP":    ( 0,  1),   # Y-axis forward
    "DOWN":  ( 0, -1),
}

DIR_ARROWS = {
    "RIGHT": (0.9, 0.0),
    "LEFT":  (-0.9, 0.0),
    "UP":    (0.0, 0.9),
    "DOWN":  (0.0, -0.9),
}

def snap_direction(ax_val, ay_val, deadzone=0.2):
    """Snap arm X/Y into one of 4 directions, or None if in deadzone."""
    if abs(ax_val) < deadzone and abs(ay_val) < deadzone:
        return None
    if abs(ax_val) >= abs(ay_val):
        return "RIGHT" if ax_val > 0 else "LEFT"
    else:
        return "UP" if ay_val < 0 else "DOWN"  # screen Y is inverted


# ─── Animation ───────────────────────────────────────────────────────────────
def animate(frame_idx):
    global drone_pos
    ax.cla()

    # Re-apply scene settings
    ax.set_xlim(-WORLD, WORLD)
    ax.set_ylim(-WORLD, WORLD)
    ax.set_zlim(0, WORLD * 2)
    ax.set_facecolor('#0d0d1a')
    ax.set_xlabel('X  (lateral)', color='#88ccff')
    ax.set_ylabel('Y  (forward)', color='#88ccff')
    ax.set_zlabel('Z  (altitude)', color='#88ccff')
    ax.tick_params(colors='#88ccff')
    ax.xaxis.pane.set_facecolor('#101025')
    ax.yaxis.pane.set_facecolor('#101025')
    ax.zaxis.pane.set_facecolor('#101025')

    # Ground grid
    ax.plot_wireframe(GX, GY, GZ, color='#224466', linewidth=0.4, alpha=0.5)

    # Keyboard override wins over UDP when active
    if kb_override["mode"]:
        ax_v  = kb_override["x"]
        ay_v  = kb_override["y"]
        az_v  = kb_override["z"]
        mode  = kb_override["mode"]
    else:
        ax_v  = state["x"]
        ay_v  = state["y"]
        az_v  = state["z"]
        mode  = state["mode"]

    # ── ALWAYS apply direct arm altitude to drone Z ──────
    if mode != "RELAXED":
        # Ensure Z has a base minimum (0.3) so drone doesn't clip through ground
        drone_pos[2] = np.clip(0.5 + az_v * WORLD, 0.3, WORLD*2 - 0.3)

    # ── FIST: discrete 4-direction step movement ─────────
    active_dir = None
    if "FIST" in mode:
        active_dir = snap_direction(ax_v, ay_v)
        if active_dir:
            dx_step, dy_step = DIRECTIONS[active_dir]
            drone_pos[0] = np.clip(drone_pos[0] + dx_step * STEP_SIZE, -WORLD+0.4, WORLD-0.4)
            drone_pos[1] = np.clip(drone_pos[1] + dy_step * STEP_SIZE, -WORLD+0.4, WORLD-0.4)

    # ── PINCH: smooth analog 3D ──────────────────────────
    elif "PINCH" in mode:
        drone_pos[0] = np.clip(ax_v * WORLD * 0.5, -WORLD+0.4, WORLD-0.4)
        drone_pos[1] = np.clip(ay_v * WORLD * 0.5, -WORLD+0.4, WORLD-0.4)

    # ── RELAXED: hover in place ──────────────────────────
    # (drone_pos unchanged)

    dx, dy, dz = drone_pos


    # Trail
    if "RELAXED" not in mode:
        trail_xyz.append((dx, dy, dz))
    
    if len(trail_xyz) > 1:
        tx, ty, tz = zip(*trail_xyz)
        trail_color = '#00ee44' if "FIST" in mode else '#ff3322'
        ax.plot(tx, ty, tz, color=trail_color, lw=2, alpha=0.6)

    # Draw directional arrows on ground for FIST mode
    if "FIST" in mode:
        for label, (axx, axy) in DIR_ARROWS.items():
            color = '#ffff00' if label == active_dir else '#334466'
            ax.text(axx, axy, 0.02, label, color=color, fontsize=9, ha='center', va='center', fontweight='bold')
            ax.annotate(
                "", xy=(axx*0.9, axy*0.9), xytext=(axx*0.4, axy*0.4),
                xycoords='data',
                annotation_clip=False
            )

    # Drone figure
    draw_drone(ax, dx, dy, dz, roll=state["roll"], pitch=state["pitch"], mode=mode)

    # Altitude line from ground
    ax.plot([dx, dx], [dy, dy], [0, dz], color='#88ccff', lw=0.8, ls='--', alpha=0.5)

    # Mode label
    mode_color = '#00ff66' if "FIST" in mode else '#ff4444' if "PINCH" in mode else '#aaaaaa'
    dir_str = f"→ {active_dir}" if active_dir else ""
    ax.set_title(
        f"X={dx:.2f}  Y={dy:.2f}  Z={dz:.2f}  |  {mode}  {dir_str}",
        color=mode_color, fontsize=11, pad=10, fontweight='bold'
    )

    # Ground crosshair
    ax.plot([-WORLD, WORLD], [dy, dy], [0, 0], color='#334455', lw=0.6)
    ax.plot([dx, dx], [-WORLD, WORLD], [0, 0], color='#334455', lw=0.6)


ani = animation.FuncAnimation(fig, animate, interval=50, cache_frame_data=False)
fig.canvas.mpl_connect('key_press_event', on_key)

plt.tight_layout()
print("[Visualizer] 3D Drone Visualizer Running.")
print("[Visualizer] KEYBOARD: Arrow=4-Dir | W/S=Up/Down | R=Reset | Q=Quit")
print("[Visualizer] Waiting for UDP on 127.0.0.1:1235 ...")
plt.show()
