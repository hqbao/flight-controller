import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import sys
import matplotlib
matplotlib.use('macosx' if sys.platform == 'darwin' else 'TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import time
import math

"""
Flight Telemetry Dashboard

Connects to the FC via serial, sends LOG_CLASS_FLIGHT_TELEMETRY,
and displays a real-time dashboard with all key flight data.

Frame layout (66 bytes):
  0–11    Attitude: roll, pitch, yaw (3 × float, radians)
  12–23   Position: x, y, z (3 × float, meters)
  24–35   Velocity: vx, vy, vz (3 × float, m/s)
  36–51   Motors: m1–m8 (8 × int16_t)
  52–63   PID outputs: roll, pitch, yaw (3 × float)
  64      Flight state (uint8_t)
  65      Sensor health bitmask (uint8_t)

Usage:
  python3 flight_telemetry_view.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 38400
SEND_LOG_ID = 0x00

LOG_CLASS_NONE              = 0x00
LOG_CLASS_HEART_BEAT        = 0x09
LOG_CLASS_FLIGHT_TELEMETRY  = 0x12
DB_CMD_LOG_CLASS            = 0x03
DB_CMD_RESET                = 0x07
DB_CMD_CHIP_ID              = 0x09

TELEMETRY_FRAME_SIZE = 66

# --- State names ---
STATE_NAMES = {
    0: 'DISARMED', 1: 'ARMED', 2: 'READY',
    3: 'TAKING_OFF', 4: 'FLYING', 5: 'LANDING', 6: 'TESTING'
}

HEALTH_BITS = ['Gyro', 'Accel', 'Compass', 'Baro', 'Range', 'OptD', 'OptU', 'GPS']

# --- Motor Parameters ---
MIN_SPEED = 150
MAX_SPEED = 1800

# --- UI Constants ---
BG_COLOR       = '#1e1e1e'
PANEL_COLOR    = '#252526'
TEXT_COLOR      = '#cccccc'
DIM_TEXT        = '#888888'
GRID_COLOR      = '#3c3c3c'
ACCENT_BLUE     = '#5599ff'
ACCENT_GREEN    = '#55cc55'
ACCENT_RED      = '#ff5555'
ACCENT_ORANGE   = '#ff9955'
ACCENT_YELLOW   = '#ffcc55'
ACCENT_CYAN     = '#55cccc'
BAR_L1_COLOR    = '#5599ff'
BAR_L2_COLOR    = '#ff9955'
BTN_GREEN       = '#2d5a2d'
BTN_GREEN_HOV   = '#3d7a3d'
BTN_RED         = '#5a2d2d'
BTN_RED_HOV     = '#7a3d3d'
HEALTH_OK       = '#55cc55'
HEALTH_BAD      = '#ff3333'

# Time series history
HISTORY_LEN = 200  # 20 seconds at 10 Hz

# --- Auto-detect serial port ---
ports = serial.tools.list_ports.comports()
print("Scanning for serial ports...")
for port, desc, hwid in sorted(ports):
    if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB', 'COM']):
        SERIAL_PORT = port
        print(f"  \u2713 Auto-selected: {port} ({desc})")
        break
    else:
        print(f"  \u00b7 Skipped: {port} ({desc})")

if not SERIAL_PORT:
    print("  \u2717 No compatible serial port found.")

# --- Global State ---
data_queue = queue.Queue()
g_serial = None
g_logging_active = False
g_chip_id = None


def send_log_class_command(ser, log_class):
    """Send DB frame to set active log class on the flight controller."""
    msg_id = DB_CMD_LOG_CLASS
    msg_class = 0x00
    length = 1
    payload = bytes([log_class])
    header = struct.pack('<2sBBH', b'db', msg_id, msg_class, length)
    checksum = (msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF) + log_class) & 0xFFFF
    frame = header + payload + struct.pack('<H', checksum)
    ser.write(frame)
    ser.flush()
    names = {0x00: 'NONE', 0x12: 'FLIGHT_TELEMETRY'}
    print(f"  \u2192 Log class: {names.get(log_class, f'0x{log_class:02X}')}")


def send_reset_command(ser):
    """Send DB frame to reset the flight controller."""
    msg_id = DB_CMD_RESET
    msg_class = 0x00
    length = 1
    payload = bytes([0x00])
    header = struct.pack('<2sBBH', b'db', msg_id, msg_class, length)
    checksum = (msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF) + 0x00) & 0xFFFF
    frame = header + payload + struct.pack('<H', checksum)
    ser.write(frame)

    ser.flush()
    print("  \u2192 Reset command sent")


def send_chip_id_request(ser):
    """Send DB frame to request the 8-byte unique chip ID."""
    msg_id = DB_CMD_CHIP_ID
    msg_class = 0x00
    length = 1
    payload = bytes([0x00])
    header = struct.pack('<2sBBH', b'db', msg_id, msg_class, length)
    checksum = (msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF) + 0x00) & 0xFFFF
    frame = header + payload + struct.pack('<H', checksum)
    ser.write(frame)

    ser.flush()
    print("  \u2192 Chip ID request sent")


_bad_data_count = 0
_bad_data_last_log = 0

def _sanitize_floats(values, label):
    """Replace NaN/Inf with 0.0 and log a warning (rate-limited)."""
    global _bad_data_count, _bad_data_last_log
    clean = []
    bad = False
    for v in values:
        if not math.isfinite(v):
            bad = True
            clean.append(0.0)
        else:
            clean.append(v)
    if bad:
        _bad_data_count += 1
        now = time.time()
        if now - _bad_data_last_log >= 1.0:
            print(f"  \u26a0 {label} has bad data: {tuple(values)}  ({_bad_data_count} bad frames)")
            _bad_data_count = 0
            _bad_data_last_log = now
    return tuple(clean)


def parse_telemetry(payload):
    """Unpack 66-byte telemetry frame into a dict."""
    if len(payload) != TELEMETRY_FRAME_SIZE:
        return None

    att = _sanitize_floats(struct.unpack_from('<3f', payload, 0), 'att')
    pos = _sanitize_floats(struct.unpack_from('<3f', payload, 12), 'pos')
    vel = _sanitize_floats(struct.unpack_from('<3f', payload, 24), 'vel')
    motors = struct.unpack_from('<8h', payload, 36)
    pid = _sanitize_floats(struct.unpack_from('<3f', payload, 52), 'pid')
    state = payload[64]
    health = payload[65]

    return {
        'att': att,       # (roll, pitch, yaw) degrees
        'pos': pos,       # (x, y, z) meters
        'vel': vel,       # (vx, vy, vz) m/s
        'motors': motors, # (m1..m8) int16
        'pid': pid,       # (roll, pitch, yaw) PID outputs
        'state': state,   # uint8
        'health': health, # bitmask
    }


def serial_reader():
    """Background thread: reads DB frames from FC, parses telemetry payloads."""
    global g_serial
    global g_chip_id
    if not SERIAL_PORT:
        return
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(b'\x00' * 32)
        ser.flush()
        time.sleep(0.05)
        g_serial = ser
        print(f"  \u2713 Connected to {SERIAL_PORT}")
        send_chip_id_request(ser)
        send_log_class_command(ser, LOG_CLASS_HEART_BEAT)

        while True:
            b1 = ser.read(1)
            if not b1:
                continue
            if b1[0] != 0x62 and b1[0] != 0x64:
                continue

            b2 = ser.read(1)
            if not b2:
                continue
            if not ((b1[0] == 0x64 and b2[0] == 0x62) or
                    (b1[0] == 0x62 and b2[0] == 0x64)):
                continue

            id_byte = ser.read(1)
            if not id_byte:
                continue
            msg_id = id_byte[0]

            class_byte = ser.read(1)
            if not class_byte:
                continue

            len_bytes = ser.read(2)
            if len(len_bytes) < 2:
                continue
            length = int.from_bytes(len_bytes, 'little')
            if length > 1024:
                continue

            payload = ser.read(length)
            if len(payload) != length:
                continue
            _ = ser.read(2)  # checksum

            if msg_id == SEND_LOG_ID and length == 8 and g_chip_id is None:
                g_chip_id = payload[:8].hex().upper()
                print(f"  \u2713 Chip ID: {g_chip_id}")
            elif msg_id == SEND_LOG_ID and length == TELEMETRY_FRAME_SIZE:
                result = parse_telemetry(payload)
                if result:
                    data_queue.put(result)
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


# --- GUI ---
def main():
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    plt.style.use('dark_background')
    plt.rcParams.update({
        'figure.facecolor': BG_COLOR,
        'axes.facecolor': BG_COLOR,
        'axes.edgecolor': GRID_COLOR,
        'axes.labelcolor': TEXT_COLOR,
        'text.color': TEXT_COLOR,
        'xtick.color': DIM_TEXT,
        'ytick.color': DIM_TEXT,
        'grid.color': GRID_COLOR,
    })

    fig = plt.figure(figsize=(18, 10))
    fig.patch.set_facecolor(BG_COLOR)

    fig.suptitle('Flight Telemetry Dashboard', fontsize=14,
                 color=TEXT_COLOR, fontweight='bold', y=0.99)

    # =========================================================================
    # 3D Quadcopter — full-screen background
    # =========================================================================
    ax_quad = fig.add_axes([0.10, 0.05, 0.85, 0.92], projection='3d')
    ax_quad.set_facecolor(BG_COLOR)
    ax_quad.view_init(elev=25, azim=-90)
    ax_quad.set_xlim(-2.5, 2.5)
    ax_quad.set_ylim(-2.5, 2.5)
    ax_quad.set_zlim(-1.5, 1.5)
    ax_quad.set_xticks([])
    ax_quad.set_yticks([])
    ax_quad.set_zticks([])
    ax_quad.xaxis.pane.fill = False
    ax_quad.yaxis.pane.fill = False
    ax_quad.zaxis.pane.fill = False
    ax_quad.xaxis.pane.set_edgecolor('none')
    ax_quad.yaxis.pane.set_edgecolor('none')
    ax_quad.zaxis.pane.set_edgecolor('none')
    ax_quad.xaxis.line.set_color('none')
    ax_quad.yaxis.line.set_color('none')
    ax_quad.zaxis.line.set_color('none')
    ax_quad.grid(False)

    # Body-frame motor positions in display coords (X=right, Y=forward, Z=up)
    ARM_LEN = 1.4
    MOTOR_POS_3D = np.array([
        [-ARM_LEN,  ARM_LEN, 0.0],  # m1 FL
        [ ARM_LEN,  ARM_LEN, 0.0],  # m2 FR
        [ ARM_LEN, -ARM_LEN, 0.0],  # m3 BR
        [-ARM_LEN, -ARM_LEN, 0.0],  # m4 BL
    ])
    MOTOR_LABELS_3D = ['M1', 'M2', 'M3', 'M4']
    NOSE_TIP  = np.array([0.0, ARM_LEN * 0.7, 0.0])
    NOSE_BASE = np.array([0.0, ARM_LEN * 0.25, 0.0])

    # Unit circle for prop disc visualisation
    N_CIRCLE = 32
    _theta = np.linspace(0, 2 * np.pi, N_CIRCLE, endpoint=True)
    UNIT_CIRCLE = np.column_stack(
        [np.cos(_theta), np.sin(_theta), np.zeros(N_CIRCLE)])

    # Ground reference (fixed at origin)
    ground_z = -0.8
    _gr_r = 2.0
    _gr_theta = np.linspace(0, 2 * np.pi, 64, endpoint=True)
    ax_quad.plot3D(
        _gr_r * np.cos(_gr_theta), _gr_r * np.sin(_gr_theta),
        np.full(64, ground_z), color=GRID_COLOR, lw=0.8, alpha=0.4)
    ax_quad.plot3D(
        [-_gr_r, _gr_r], [0, 0], [ground_z, ground_z],
        color=GRID_COLOR, lw=0.5, alpha=0.3)
    ax_quad.plot3D(
        [0, 0], [-_gr_r, _gr_r], [ground_z, ground_z],
        color=GRID_COLOR, lw=0.5, alpha=0.3)
    ax_quad.plot3D(
        [0, 0], [0, _gr_r * 0.7], [ground_z, ground_z],
        color=ACCENT_RED, lw=1, alpha=0.4)
    ax_quad.text(0, _gr_r * 0.8, ground_z, 'N', color=ACCENT_RED,
                 fontsize=9, ha='center', va='center', alpha=0.5)

    def make_rotation(roll_deg, pitch_deg, yaw_deg):
        """Build 3D rotation matrix for display frame (X=right, Y=fwd, Z=up)
        from NED Euler angles in degrees."""
        r = np.radians(roll_deg)
        p = np.radians(pitch_deg)
        y = np.radians(yaw_deg)
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        R_ned = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr            ]
        ])
        T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
        return T @ R_ned @ T

    # Arm lines (center -> motor)
    arm_lines = []
    for i in range(4):
        line, = ax_quad.plot3D([0, 0], [0, 0], [0, 0],
                               color='#888888', lw=3)
        arm_lines.append(line)

    # Nose indicator
    nose_line, = ax_quad.plot3D([0, 0], [0, 0], [0, 0],
                                color=ACCENT_RED, lw=3)

    # Prop disc lines — L1 (m1-m4) and L2 (m5-m8)
    prop_lines_l1 = []
    prop_lines_l2 = []
    for i in range(4):
        l1, = ax_quad.plot3D([], [], [], color=BAR_L1_COLOR,
                             lw=2.0, alpha=0.8)
        l2, = ax_quad.plot3D([], [], [], color=BAR_L2_COLOR,
                             lw=1.5, alpha=0.6)
        prop_lines_l1.append(l1)
        prop_lines_l2.append(l2)

    # Motor label texts (3D positioned)
    motor_label_texts = []
    for i in range(4):
        txt = ax_quad.text(0, 0, 0, MOTOR_LABELS_3D[i], fontsize=8,
                           ha='center', va='bottom', color=TEXT_COLOR,
                           fontweight='bold')
        motor_label_texts.append(txt)

    # =========================================================================
    # Left data panel — instrument readout strip
    # =========================================================================
    ax_data = fig.add_axes([0.005, 0.05, 0.105, 0.92])
    ax_data.set_facecolor(BG_COLOR)
    ax_data.patch.set_alpha(0.6)
    ax_data.set_xlim(0, 1)
    ax_data.set_ylim(0, 1)
    ax_data.set_xticks([])
    ax_data.set_yticks([])
    for spine in ax_data.spines.values():
        spine.set_edgecolor(GRID_COLOR)
        spine.set_alpha(0.3)

    _mono = dict(fontfamily='monospace', transform=ax_data.transAxes)
    _hdr  = dict(fontsize=8, color=DIM_TEXT, fontweight='bold', ha='left', **_mono)
    _val  = dict(fontsize=10, fontweight='bold', ha='left', **_mono)

    # Section: Attitude
    ax_data.text(0.08, 0.96, 'ATTITUDE', **_hdr)
    ax_data.plot([0.05, 0.95], [0.955, 0.955], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_roll  = ax_data.text(0.08, 0.92, 'R:   0.0\u00b0', color=ACCENT_BLUE, **_val)
    data_pitch = ax_data.text(0.08, 0.89, 'P:   0.0\u00b0', color=ACCENT_GREEN, **_val)
    data_yaw   = ax_data.text(0.08, 0.86, 'Y:   0.0\u00b0', color=ACCENT_ORANGE, **_val)

    # Section: Position
    ax_data.text(0.08, 0.81, 'POSITION', **_hdr)
    ax_data.plot([0.05, 0.95], [0.805, 0.805], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_px = ax_data.text(0.08, 0.77, 'X:  0.00m', color=ACCENT_CYAN, **_val)
    data_py = ax_data.text(0.08, 0.74, 'Y:  0.00m', color=ACCENT_CYAN, **_val)
    data_pz = ax_data.text(0.08, 0.71, 'Z:  0.00m', color=ACCENT_YELLOW, **_val)

    # Section: Velocity
    ax_data.text(0.08, 0.66, 'VELOCITY', **_hdr)
    ax_data.plot([0.05, 0.95], [0.655, 0.655], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_vx = ax_data.text(0.08, 0.62, 'Vx: 0.00', color=ACCENT_BLUE, **_val)
    data_vy = ax_data.text(0.08, 0.59, 'Vy: 0.00', color=ACCENT_GREEN, **_val)
    data_vz = ax_data.text(0.08, 0.56, 'Vz: 0.00', color=ACCENT_ORANGE, **_val)

    # Section: Motors (L1/L2 per position)
    ax_data.text(0.08, 0.51, 'MOTORS', **_hdr)
    ax_data.plot([0.05, 0.95], [0.505, 0.505], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_m = []
    motor_names = ['FL', 'FR', 'BR', 'BL']
    for i in range(4):
        txt = ax_data.text(0.08, 0.47 - i * 0.03,
                           f'{motor_names[i]}: 0/0', color=TEXT_COLOR,
                           fontsize=9, **_mono)
        data_m.append(txt)

    # Section: PID
    ax_data.text(0.08, 0.33, 'PID OUT', **_hdr)
    ax_data.plot([0.05, 0.95], [0.325, 0.325], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_pid_r = ax_data.text(0.08, 0.29, 'R:  0.0', color=ACCENT_BLUE, **_val)
    data_pid_p = ax_data.text(0.08, 0.26, 'P:  0.0', color=ACCENT_GREEN, **_val)
    data_pid_y = ax_data.text(0.08, 0.23, 'Y:  0.0', color=ACCENT_ORANGE, **_val)

    # =========================================================================
    # Overlay: Position XY mini-map (top-right)
    # =========================================================================
    ax_xy = fig.add_axes([0.80, 0.62, 0.18, 0.32])
    ax_xy.patch.set_alpha(0.5)
    ax_xy.set_title('Position XY', color=DIM_TEXT, fontsize=9, pad=4)
    ax_xy.set_xlabel('E (m)', fontsize=7)
    ax_xy.set_ylabel('N (m)', fontsize=7)
    ax_xy.tick_params(labelsize=7)
    ax_xy.grid(True, alpha=0.2)
    ax_xy.set_xlim(-3, 3)
    ax_xy.set_ylim(-3, 3)
    for spine in ax_xy.spines.values():
        spine.set_edgecolor(GRID_COLOR)
        spine.set_alpha(0.5)
    pos_trail, = ax_xy.plot([], [], '-', color=ACCENT_CYAN, linewidth=1.2, alpha=0.7)
    pos_dot, = ax_xy.plot([], [], 'o', color=ACCENT_CYAN, markersize=6)
    xy_history = {'x': [], 'y': []}
    xy_zoom = [None]  # None = auto-scale, float = manual half-size

    def on_scroll_xy(event):
        if event.inaxes != ax_xy:
            return
        xlim = ax_xy.get_xlim()
        cur_half = (xlim[1] - xlim[0]) / 2
        factor = 0.8 if event.button == 'up' else 1.25
        new_half = max(0.5, min(50.0, cur_half * factor))
        xy_zoom[0] = new_half

    fig.canvas.mpl_connect('scroll_event', on_scroll_xy)

    # =========================================================================
    # Overlay: Velocity chart (right, below Position XY)
    # =========================================================================
    ax_vel = fig.add_axes([0.80, 0.34, 0.18, 0.22])
    ax_vel.patch.set_alpha(0.5)
    ax_vel.set_title('Velocity', color=DIM_TEXT, fontsize=9, pad=4)
    ax_vel.set_xlabel('t (s)', fontsize=7)
    ax_vel.set_ylabel('m/s', fontsize=7)
    ax_vel.tick_params(labelsize=7)
    ax_vel.grid(True, alpha=0.2)
    for spine in ax_vel.spines.values():
        spine.set_edgecolor(GRID_COLOR)
        spine.set_alpha(0.5)
    vel_labels = ['Vx', 'Vy', 'Vz']
    vel_colors = [ACCENT_BLUE, ACCENT_GREEN, ACCENT_ORANGE]
    vel_lines = []
    for label, color in zip(vel_labels, vel_colors):
        line, = ax_vel.plot([], [], '-', color=color, linewidth=1.0, label=label)
        vel_lines.append(line)
    ax_vel.legend(loc='upper left', fontsize=6, framealpha=0.3)
    vel_history = {'t': [], 'vx': [], 'vy': [], 'vz': []}
    vel_t0 = [None]

    # =========================================================================
    # Overlay: Altitude chart (right, below Velocity)
    # =========================================================================
    ax_alt = fig.add_axes([0.80, 0.08, 0.18, 0.22])
    ax_alt.patch.set_alpha(0.5)
    ax_alt.set_title('Altitude', color=DIM_TEXT, fontsize=9, pad=4)
    ax_alt.set_xlabel('t (s)', fontsize=7)
    ax_alt.set_ylabel('Z (m)', fontsize=7)
    ax_alt.tick_params(labelsize=7)
    ax_alt.grid(True, alpha=0.2)
    for spine in ax_alt.spines.values():
        spine.set_edgecolor(GRID_COLOR)
        spine.set_alpha(0.5)
    alt_line, = ax_alt.plot([], [], '-', color=ACCENT_YELLOW, linewidth=1.2)
    alt_history = {'t': [], 'z': []}
    alt_t0 = [None]

    # =========================================================================
    # Status bar — bottom edge
    # =========================================================================
    chip_id_text = fig.text(0.96, 0.035, 'Chip ID: ---', fontsize=7,
                             ha='right', color=DIM_TEXT)

    state_text = fig.text(0.16, 0.015, 'State: ---', fontsize=9,
                          color=TEXT_COLOR, fontweight='bold')

    health_texts = []
    for i, name in enumerate(HEALTH_BITS):
        x = 0.32 + i * 0.05
        txt = fig.text(x, 0.015, f'\u25cf{name}', fontsize=7, color=DIM_TEXT)
        health_texts.append(txt)

    fps_text = fig.text(0.96, 0.015, '', fontsize=8, ha='right',
                        color=DIM_TEXT)

    # =========================================================================
    # Toggle Log Button
    # =========================================================================
    ax_toggle = fig.add_axes([0.005, 0.005, 0.08, 0.04])
    btn_toggle = Button(ax_toggle, 'Start Log',
                        color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_toggle.label.set_color(TEXT_COLOR)
    btn_toggle.label.set_fontsize(8)

    def on_toggle(event):
        global g_logging_active
        if g_serial and g_serial.is_open:
            if g_logging_active:
                send_log_class_command(g_serial, LOG_CLASS_NONE)
                g_logging_active = False
                btn_toggle.label.set_text('Start Log')
                btn_toggle.color = BTN_GREEN
                btn_toggle.hovercolor = BTN_GREEN_HOV
                ax_toggle.set_facecolor(BTN_GREEN)
            else:
                send_log_class_command(g_serial, LOG_CLASS_FLIGHT_TELEMETRY)
                g_logging_active = True
                btn_toggle.label.set_text('Stop Log')
                btn_toggle.color = BTN_RED
                btn_toggle.hovercolor = BTN_RED_HOV
                ax_toggle.set_facecolor(BTN_RED)

    btn_toggle.on_clicked(on_toggle)

    # =========================================================================
    # Reset FC Button
    # =========================================================================
    ax_reset = fig.add_axes([0.09, 0.005, 0.08, 0.04])
    btn_reset = Button(ax_reset, 'Reset FC',
                       color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_reset.label.set_color(TEXT_COLOR)
    btn_reset.label.set_fontsize(8)

    def on_reset(event):
        global g_logging_active
        if g_serial and g_serial.is_open:
            send_reset_command(g_serial)
            g_logging_active = False
            btn_toggle.label.set_text('Start Log')
            btn_toggle.color = BTN_GREEN
            btn_toggle.hovercolor = BTN_GREEN_HOV
            ax_toggle.set_facecolor(BTN_GREEN)

            # FC reboots after reset — wait then re-init
            def _post_reset():
                time.sleep(2.0)
                if g_serial and g_serial.is_open:
                    g_serial.reset_input_buffer()
                    send_log_class_command(g_serial, LOG_CLASS_HEART_BEAT)

            threading.Thread(target=_post_reset, daemon=True).start()

    btn_reset.on_clicked(on_reset)

    # =========================================================================
    # Animation loop
    # =========================================================================
    frame_count = [0]
    last_fps_time = [time.time()]

    def update(frame_num):
        updated = []
        latest = None

        while not data_queue.empty():
            try:
                latest = data_queue.get_nowait()
            except queue.Empty:
                break

        if latest is None:
            return updated

        d = latest
        roll, pitch, yaw = d['att']
        motors = d['motors']

        # =================================================================
        # 3D Quadcopter
        # =================================================================
        R = make_rotation(roll, pitch, yaw)
        motor_pts = (R @ MOTOR_POS_3D.T).T
        nose_tip_r  = R @ NOSE_TIP
        nose_base_r = R @ NOSE_BASE

        for i in range(4):
            arm_lines[i].set_data_3d(
                [0, motor_pts[i, 0]],
                [0, motor_pts[i, 1]],
                [0, motor_pts[i, 2]])

        nose_line.set_data_3d(
            [nose_base_r[0], nose_tip_r[0]],
            [nose_base_r[1], nose_tip_r[1]],
            [nose_base_r[2], nose_tip_r[2]])

        for i in range(4):
            mx, my, mz = motor_pts[i]

            r1 = 0.15 + np.clip(motors[i] / MAX_SPEED, 0, 1) * 0.45
            circle_body = MOTOR_POS_3D[i] + r1 * UNIT_CIRCLE
            circle_w = (R @ circle_body.T).T
            prop_lines_l1[i].set_data_3d(
                circle_w[:, 0], circle_w[:, 1], circle_w[:, 2])
            prop_lines_l1[i].set_alpha(
                0.3 + np.clip(motors[i] / MAX_SPEED, 0, 1) * 0.6)

            r2 = 0.10 + np.clip(motors[i+4] / MAX_SPEED, 0, 1) * 0.35
            offset = np.array([0.0, 0.0, 0.08])
            circle_body2 = MOTOR_POS_3D[i] + offset + r2 * UNIT_CIRCLE
            circle_w2 = (R @ circle_body2.T).T
            prop_lines_l2[i].set_data_3d(
                circle_w2[:, 0], circle_w2[:, 1], circle_w2[:, 2])
            prop_lines_l2[i].set_alpha(
                0.2 + np.clip(motors[i+4] / MAX_SPEED, 0, 1) * 0.5)

            motor_label_texts[i].set_position((mx, my))
            motor_label_texts[i].set_3d_properties(mz + 0.35, zdir='z')

        # =================================================================
        # Data panel values
        # =================================================================
        data_roll.set_text(f'R:{roll:+6.1f}\u00b0')
        data_pitch.set_text(f'P:{pitch:+6.1f}\u00b0')
        data_yaw.set_text(f'Y:{yaw:+6.1f}\u00b0')

        px_ned, py_ned, pz_ned = d['pos']
        data_px.set_text(f'X:{px_ned:+6.2f}m')
        data_py.set_text(f'Y:{py_ned:+6.2f}m')
        data_pz.set_text(f'Z:{pz_ned:+6.2f}m')

        data_vx.set_text(f'Vx:{d["vel"][0]:+5.2f}')
        data_vy.set_text(f'Vy:{d["vel"][1]:+5.2f}')
        data_vz.set_text(f'Vz:{d["vel"][2]:+5.2f}')

        for i in range(4):
            data_m[i].set_text(f'{motor_names[i]}:{motors[i]:5d}/{motors[i+4]:5d}')

        data_pid_r.set_text(f'R:{d["pid"][0]:+6.1f}')
        data_pid_p.set_text(f'P:{d["pid"][1]:+6.1f}')
        data_pid_y.set_text(f'Y:{d["pid"][2]:+6.1f}')

        # =================================================================
        # Position XY
        # =================================================================
        if np.isfinite(py_ned) and np.isfinite(px_ned):
            xy_history['x'].append(py_ned)
            xy_history['y'].append(px_ned)
        if len(xy_history['x']) > HISTORY_LEN:
            xy_history['x'] = xy_history['x'][-HISTORY_LEN:]
            xy_history['y'] = xy_history['y'][-HISTORY_LEN:]
        pos_trail.set_data(xy_history['x'], xy_history['y'])
        pos_dot.set_data([py_ned], [px_ned])
        # Zoom: manual (scroll) or auto-fit
        if xy_zoom[0] is not None and np.isfinite(py_ned) and np.isfinite(px_ned):
            half = xy_zoom[0]
            ax_xy.set_xlim(py_ned - half, py_ned + half)
            ax_xy.set_ylim(px_ned - half, px_ned + half)
        elif len(xy_history['x']) > 1:
            xarr = np.array(xy_history['x'])
            yarr = np.array(xy_history['y'])
            xmin, xmax = xarr.min(), xarr.max()
            ymin, ymax = yarr.min(), yarr.max()
            span = max(xmax - xmin, ymax - ymin, 2.0)
            cx = (xmin + xmax) / 2
            cy = (ymin + ymax) / 2
            half = span / 2 + 1.0
            ax_xy.set_xlim(cx - half, cx + half)
            ax_xy.set_ylim(cy - half, cy + half)
        updated.extend([pos_trail, pos_dot])

        # =================================================================
        # Velocity chart
        # =================================================================
        now = time.time()
        if vel_t0[0] is None:
            vel_t0[0] = now
        vt = now - vel_t0[0]
        vel_history['t'].append(vt)
        vel_history['vx'].append(d['vel'][0])
        vel_history['vy'].append(d['vel'][1])
        vel_history['vz'].append(d['vel'][2])
        if len(vel_history['t']) > HISTORY_LEN:
            vel_history['t'] = vel_history['t'][-HISTORY_LEN:]
            vel_history['vx'] = vel_history['vx'][-HISTORY_LEN:]
            vel_history['vy'] = vel_history['vy'][-HISTORY_LEN:]
            vel_history['vz'] = vel_history['vz'][-HISTORY_LEN:]
        vel_lines[0].set_data(vel_history['t'], vel_history['vx'])
        vel_lines[1].set_data(vel_history['t'], vel_history['vy'])
        vel_lines[2].set_data(vel_history['t'], vel_history['vz'])
        if len(vel_history['t']) > 1:
            ax_vel.set_xlim(vel_history['t'][0], vel_history['t'][-1] + 0.1)
            all_v = np.array(vel_history['vx'] + vel_history['vy'] + vel_history['vz'])
            all_v = all_v[np.isfinite(all_v)]
            if len(all_v) > 0:
                vmin, vmax = all_v.min(), all_v.max()
                vm = max(0.5, (vmax - vmin) * 0.2)
                ax_vel.set_ylim(vmin - vm, vmax + vm)
        updated.extend(vel_lines)

        # =================================================================
        # Altitude chart
        # =================================================================
        if alt_t0[0] is None:
            alt_t0[0] = now
        t = now - alt_t0[0]
        alt_history['t'].append(t)
        alt_history['z'].append(d['pos'][2])
        if len(alt_history['t']) > HISTORY_LEN:
            alt_history['t'] = alt_history['t'][-HISTORY_LEN:]
            alt_history['z'] = alt_history['z'][-HISTORY_LEN:]
        alt_line.set_data(alt_history['t'], alt_history['z'])
        if len(alt_history['t']) > 1:
            ax_alt.set_xlim(alt_history['t'][0], alt_history['t'][-1] + 0.1)
            zarr = np.array(alt_history['z'])
            zarr = zarr[np.isfinite(zarr)]
            if len(zarr) > 0:
                zmin, zmax = zarr.min(), zarr.max()
                margin = max(0.5, (zmax - zmin) * 0.2)
                ax_alt.set_ylim(zmin - margin, zmax + margin)
        updated.append(alt_line)

        # =================================================================
        # State + Health
        # =================================================================
        state_name = STATE_NAMES.get(d['state'], f'UNKNOWN({d["state"]})')
        state_color = ACCENT_GREEN if d['state'] >= 3 else (
            ACCENT_YELLOW if d['state'] >= 1 else TEXT_COLOR)
        state_text.set_text(f'State: {state_name}')
        state_text.set_color(state_color)
        updated.append(state_text)

        for i, txt in enumerate(health_texts):
            ok = (d['health'] >> i) & 1
            txt.set_color(HEALTH_OK if ok else HEALTH_BAD)
        updated.extend(health_texts)

        # =================================================================
        # Chip ID
        # =================================================================
        if g_chip_id is not None:
            chip_id_text.set_text(f'Chip ID: {g_chip_id}')
            updated.append(chip_id_text)

        # =================================================================
        # FPS
        # =================================================================
        frame_count[0] += 1
        elapsed = now - last_fps_time[0]
        if elapsed >= 1.0:
            fps = frame_count[0] / elapsed
            fps_text.set_text(f'{fps:.0f} Hz')
            frame_count[0] = 0
            last_fps_time[0] = now
            updated.append(fps_text)

        return updated

    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
