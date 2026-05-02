import serial
import serial.tools.list_ports
import struct
import threading
import numpy as np
import sys
import matplotlib
matplotlib.use('macosx' if sys.platform == 'darwin' else 'TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import time
import math


def screen_fit_figsize(base_width, base_height, margin_px=90, dpi=100):
    try:
        import tkinter as tk
        root = tk.Tk(); root.withdraw()
        screen_h = root.winfo_screenheight()
        root.destroy()
    except Exception:
        return (base_width, base_height)
    scale = min(1.0, max(300, screen_h - margin_px) / (base_height * dpi))
    return (base_width * scale, base_height * scale)

"""
Flight Telemetry Dashboard — Bicopter

Connects to the FC via serial, sends LOG_CLASS_FLIGHT_TELEMETRY,
and displays a real-time dashboard with a tilting-nacelle bicopter.

Frame layout (66 bytes):
  0–11    Attitude: roll, pitch, yaw (3 × float, degrees)
  12–23   Position: x, y, z (3 × float, meters)
  24–35   Velocity: vx, vy, vz (3 × float, m/s)
  36–51   Motors: m1–m8 (8 × int16_t)
            [0]=left motor, [1]=right motor,
            [2..3]=unused, [4]=left servo (PWM µs), [5]=right servo (PWM µs),
            [4..7]=unused
  52–63   PID outputs: roll, pitch, yaw (3 × float)
  64      Flight state (uint8_t)
  65      Sensor health bitmask (uint8_t)

Usage:
  python3 flight_telemetry_bicopter_view.py
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

# --- Motor / Servo Parameters ---
MIN_SPEED    = 150
MAX_SPEED    = 1800
SERVO_MIN    = 1000
SERVO_MAX    = 2000
SERVO_CENTER = 1500

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
MOTOR_COLOR     = '#5599ff'
SERVO_COLOR     = '#ff9955'
BTN_GREEN       = '#2d5a2d'
BTN_GREEN_HOV   = '#3d7a3d'
BTN_RED         = '#5a2d2d'
BTN_RED_HOV     = '#7a3d3d'
BTN_COLOR       = '#333333'
BTN_HOVER       = '#555555'
HEALTH_OK       = '#55cc55'
HEALTH_BAD      = '#ff3333'

# Time series history
HISTORY_LEN = 250  # 10 seconds at 25 Hz — keeps line plots cheap to redraw

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
# Latest-only frame slot (see flight_telemetry_view.py for rationale).
g_latest = [None]
g_latest_lock = threading.Lock()
g_serial = None
g_logging_active = False
g_chip_id = None

# Counter of telemetry frames received from the FC.
g_rx_frame_count = [0]


def send_log_class_command(ser, log_class):
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

# --- Outlier limits (viewer-side hardening) ---
MAX_POS_M    = 10000.0   # 10 km from origin
MAX_VEL_MPS  = 100.0     # 100 m/s ground/vertical speed

_outlier_count = 0
_outlier_last_log = 0

def _is_outlier(pos, vel):
    for v in pos:
        if abs(v) > MAX_POS_M:
            return True
    for v in vel:
        if abs(v) > MAX_VEL_MPS:
            return True
    return False


def _robust_ylim(values, pad_frac=0.15, min_span=0.5):
    """Return (lo, hi) y-limits using 5th/95th percentile to ignore outliers."""
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return (-1.0, 1.0)
    if arr.size < 4:
        lo, hi = float(arr.min()), float(arr.max())
    else:
        lo = float(np.percentile(arr, 5))
        hi = float(np.percentile(arr, 95))
    span = hi - lo
    if span < min_span:
        mid = (lo + hi) * 0.5
        lo, hi = mid - min_span * 0.5, mid + min_span * 0.5
        span = hi - lo
    pad = span * pad_frac
    return (lo - pad, hi + pad)


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
        'att': att,
        'pos': pos,
        'vel': vel,
        'motors': motors,
        'pid': pid,
        'state': state,
        'health': health,
    }


def serial_reader():
    """Background thread: bulk-read DB frames and park the latest one.
    See flight_telemetry_view.py for the rationale (byte-by-byte reads at
    38400 baud cause kernel-buffer backlog when matplotlib stalls).
    """
    global g_serial, g_chip_id
    if not SERIAL_PORT:
        return
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(b'\x00' * 32)
        ser.flush()
        time.sleep(0.05)
        g_serial = ser
        print(f"  \u2713 Connected to {SERIAL_PORT}")
        send_chip_id_request(ser)
        send_log_class_command(ser, LOG_CLASS_HEART_BEAT)

        BACKLOG_FLUSH_BYTES = 4096
        last_backlog_log = 0.0

        rx = bytearray()

        while True:
            n_waiting = ser.in_waiting
            if n_waiting > BACKLOG_FLUSH_BYTES:
                ser.reset_input_buffer()
                rx.clear()
                now = time.time()
                if now - last_backlog_log >= 1.0:
                    print(f"  \u26a0 backlog {n_waiting} B in kernel buffer \u2014 flushed (viewer falling behind)")
                    last_backlog_log = now
                continue

            chunk = ser.read(n_waiting if n_waiting > 0 else 1)
            if not chunk:
                continue
            rx.extend(chunk)

            while True:
                i = rx.find(b'db')
                if i < 0:
                    if len(rx) > 1:
                        del rx[:-1]
                    break
                if i > 0:
                    del rx[:i]
                if len(rx) < 6:
                    break
                length = int.from_bytes(rx[4:6], 'little')
                if length > 1024:
                    del rx[:2]
                    continue
                frame_total = 6 + length + 2
                if len(rx) < frame_total:
                    break

                msg_id = rx[2]
                payload = bytes(rx[6:6 + length])
                del rx[:frame_total]

                if msg_id == SEND_LOG_ID and length == 8 and g_chip_id is None:
                    g_chip_id = payload[:8].hex().upper()
                    print(f"  \u2713 Chip ID: {g_chip_id}")
                elif msg_id == SEND_LOG_ID and length == TELEMETRY_FRAME_SIZE:
                    result = parse_telemetry(payload)
                    if not result:
                        continue
                    global _outlier_count, _outlier_last_log
                    if _is_outlier(result['pos'], result['vel']):
                        _outlier_count += 1
                        now = time.time()
                        if now - _outlier_last_log >= 1.0:
                            print(f"  \u26a0 outlier dropped: pos={result['pos']} vel={result['vel']}  ({_outlier_count} dropped)")
                            _outlier_count = 0
                            _outlier_last_log = now
                        continue
                    with g_latest_lock:
                        g_latest[0] = result
                    g_rx_frame_count[0] += 1
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


def servo_to_angle_deg(pwm_us):
    return (pwm_us - SERVO_CENTER) / (SERVO_MAX - SERVO_CENTER) * 45.0


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

    fig = plt.figure(figsize=screen_fit_figsize(18, 10))
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('Flight Telemetry Dashboard \u2014 Bicopter', fontsize=14,
                 color=TEXT_COLOR, fontweight='bold', y=0.99)

    # =========================================================================
    # 3D Bicopter — full-screen background
    # =========================================================================
    ax_bic = fig.add_axes([0.10, 0.10, 0.85, 0.87], projection='3d')
    ax_bic.set_facecolor(BG_COLOR)
    ax_bic.view_init(elev=0, azim=180)
    ax_bic.set_xlim(-2.5, 2.5)
    ax_bic.set_ylim(-2.5, 2.5)
    ax_bic.set_zlim(-1.5, 1.5)
    ax_bic.set_xticks([])
    ax_bic.set_yticks([])
    ax_bic.set_zticks([])
    ax_bic.xaxis.pane.fill = False
    ax_bic.yaxis.pane.fill = False
    ax_bic.zaxis.pane.fill = False
    ax_bic.xaxis.pane.set_edgecolor('none')
    ax_bic.yaxis.pane.set_edgecolor('none')
    ax_bic.zaxis.pane.set_edgecolor('none')
    ax_bic.xaxis.line.set_color('none')
    ax_bic.yaxis.line.set_color('none')
    ax_bic.zaxis.line.set_color('none')
    ax_bic.grid(False)

    # Body-frame geometry (display coords: X=right, Y=forward, Z=up)
    BEAM_HALF = 1.6     # half span of main beam
    FUSELAGE_LEN = 0.6  # fuselage half-length (forward/backward from center)
    NACELLE_LEN = 0.45  # nacelle arm length below pivot

    # Motor mount positions (beam tips)
    MOUNT_L = np.array([-BEAM_HALF, 0.0, 0.0])  # left
    MOUNT_R = np.array([ BEAM_HALF, 0.0, 0.0])   # right

    # Nose tip & base for direction indicator
    NOSE_TIP  = np.array([0.0, FUSELAGE_LEN * 1.6, 0.0])
    NOSE_BASE = np.array([0.0, FUSELAGE_LEN * 0.4, 0.0])

    # Unit circle for prop disc
    N_CIRCLE = 32
    _theta = np.linspace(0, 2 * np.pi, N_CIRCLE, endpoint=True)
    UNIT_CIRCLE = np.column_stack(
        [np.cos(_theta), np.sin(_theta), np.zeros(N_CIRCLE)])

    # Ground reference
    ground_z = -0.8
    _gr_r = 2.0
    _gr_theta = np.linspace(0, 2 * np.pi, 64, endpoint=True)
    ax_bic.plot3D(
        _gr_r * np.cos(_gr_theta), _gr_r * np.sin(_gr_theta),
        np.full(64, ground_z), color=GRID_COLOR, lw=0.8, alpha=0.4)
    ax_bic.plot3D(
        [-_gr_r, _gr_r], [0, 0], [ground_z, ground_z],
        color=GRID_COLOR, lw=0.5, alpha=0.3)
    ax_bic.plot3D(
        [0, 0], [-_gr_r, _gr_r], [ground_z, ground_z],
        color=GRID_COLOR, lw=0.5, alpha=0.3)
    ax_bic.plot3D(
        [0, 0], [0, _gr_r * 0.7], [ground_z, ground_z],
        color=ACCENT_RED, lw=1, alpha=0.4)
    ax_bic.text(0, _gr_r * 0.8, ground_z, 'N', color=ACCENT_RED,
                fontsize=9, ha='center', va='center', alpha=0.5)

    def make_rotation(roll_deg, pitch_deg, yaw_deg):
        """Build a 3D rotation matrix for the *display* frame
        (X=right/East, Y=forward/North, Z=up) from FC Euler angles.
        Body axes defined directly in display frame so:
          + roll  rolls right wing down
          + pitch tilts the nose up
          + yaw   rotates nose toward east (CW from above)
        """
        r = np.radians(roll_deg)
        p = np.radians(pitch_deg)
        y = np.radians(yaw_deg)
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        Rz = np.array([[ cy, -sy, 0],
                       [ sy,  cy, 0],
                       [  0,   0, 1]])
        Rx = np.array([[1,  0,   0 ],
                       [0, cp, -sp],
                       [0, sp,  cp]])
        Ry = np.array([[ cr, 0, sr],
                       [  0, 1,  0],
                       [-sr, 0, cr]])
        return Rz @ Rx @ Ry

    def nacelle_tilt_matrix(angle_deg):
        """Rotation about body X-axis (left-right beam axis) by tilt angle.
        Positive angle tilts nacelle forward (+Y in display frame)."""
        a = np.radians(angle_deg)
        return np.array([
            [1, 0,        0       ],
            [0, np.cos(a), np.sin(a)],
            [0, -np.sin(a), np.cos(a)]
        ])

    # --- Draw elements ---
    # Main beam (left mount -> right mount)
    beam_line, = ax_bic.plot3D([0, 0], [0, 0], [0, 0],
                               color='#888888', lw=4)

    # Fuselage (short fore-aft body)
    fuselage_line, = ax_bic.plot3D([0, 0], [0, 0], [0, 0],
                                   color='#666666', lw=3)

    # Nose indicator
    nose_line, = ax_bic.plot3D([0, 0], [0, 0], [0, 0],
                               color=ACCENT_RED, lw=3)

    # Nacelle arms (pivot -> motor)
    nacelle_l_line, = ax_bic.plot3D([0, 0], [0, 0], [0, 0],
                                    color=SERVO_COLOR, lw=3)
    nacelle_r_line, = ax_bic.plot3D([0, 0], [0, 0], [0, 0],
                                    color=SERVO_COLOR, lw=3)

    # Prop discs
    prop_l_line, = ax_bic.plot3D([], [], [], color=MOTOR_COLOR,
                                 lw=2.0, alpha=0.8)
    prop_r_line, = ax_bic.plot3D([], [], [], color=MOTOR_COLOR,
                                 lw=2.0, alpha=0.8)

    # Motor label texts
    label_m1 = ax_bic.text(0, 0, 0, 'M1', fontsize=8, ha='center',
                           va='bottom', color=TEXT_COLOR, fontweight='bold')
    label_m2 = ax_bic.text(0, 0, 0, 'M2', fontsize=8, ha='center',
                           va='bottom', color=TEXT_COLOR, fontweight='bold')

    # =========================================================================
    # Left data panel — instrument readout strip
    # =========================================================================
    ax_data = fig.add_axes([0.005, 0.10, 0.105, 0.87])
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
    data_px = ax_data.text(0.08, 0.77, 'N:  0.00m', color=ACCENT_CYAN, **_val)
    data_py = ax_data.text(0.08, 0.74, 'E:  0.00m', color=ACCENT_CYAN, **_val)
    data_pz = ax_data.text(0.08, 0.71, 'Alt: 0.00m', color=ACCENT_YELLOW, **_val)

    # Section: Velocity
    ax_data.text(0.08, 0.66, 'VELOCITY', **_hdr)
    ax_data.plot([0.05, 0.95], [0.655, 0.655], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_vx = ax_data.text(0.08, 0.62, 'Vn: 0.00', color=ACCENT_BLUE, **_val)
    data_vy = ax_data.text(0.08, 0.59, 'Ve: 0.00', color=ACCENT_GREEN, **_val)
    data_vz = ax_data.text(0.08, 0.56, 'Vu: 0.00', color=ACCENT_ORANGE, **_val)

    # Section: Motors + Servos
    ax_data.text(0.08, 0.51, 'MOTORS', **_hdr)
    ax_data.plot([0.05, 0.95], [0.505, 0.505], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_m1 = ax_data.text(0.08, 0.47, 'M1(L):    0', color=MOTOR_COLOR,
                           fontsize=9, **_mono)
    data_m2 = ax_data.text(0.08, 0.44, 'M2(R):    0', color=MOTOR_COLOR,
                           fontsize=9, **_mono)

    ax_data.text(0.08, 0.39, 'SERVOS', **_hdr)
    ax_data.plot([0.05, 0.95], [0.385, 0.385], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_s1 = ax_data.text(0.08, 0.35, 'S1(L):  0.0\u00b0', color=SERVO_COLOR,
                           fontsize=9, **_mono)
    data_s2 = ax_data.text(0.08, 0.32, 'S2(R):  0.0\u00b0', color=SERVO_COLOR,
                           fontsize=9, **_mono)

    # Section: PID
    ax_data.text(0.08, 0.27, 'PID OUT', **_hdr)
    ax_data.plot([0.05, 0.95], [0.265, 0.265], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_pid_r = ax_data.text(0.08, 0.23, 'R:  0.0', color=ACCENT_BLUE, **_val)
    data_pid_p = ax_data.text(0.08, 0.20, 'P:  0.0', color=ACCENT_GREEN, **_val)
    data_pid_y = ax_data.text(0.08, 0.17, 'Y:  0.0', color=ACCENT_ORANGE, **_val)

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
    xy_zoom = [None]

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
    vel_labels = ['Vn', 'Ve', 'Vu']
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
    ax_alt.set_ylabel('Alt (m)', fontsize=7)
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
    chip_id_text = fig.text(0.995, 0.078, 'Chip ID: ---', fontsize=7,
                             ha='right', color=DIM_TEXT)

    state_text = fig.text(0.005, 0.055, 'State: ---', fontsize=9,
                          color=TEXT_COLOR, fontweight='bold')

    health_texts = []
    for i, name in enumerate(HEALTH_BITS):
        x = 0.18 + i * 0.05
        txt = fig.text(x, 0.057, f'\u25cf{name}', fontsize=7, color=DIM_TEXT)
        health_texts.append(txt)

    fps_text = fig.text(0.995, 0.025, 'Data:   0 Hz | UI:  0 Hz', fontsize=8,
                        ha='right', color=DIM_TEXT)

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
    # 3D Face-View Buttons
    # =========================================================================
    _views_ft = {'Top': (90, 180), 'Front': (0, 0), 'Back': (0, 180),
                 'Left': (0, 90), 'Right': (0, -90)}
    _view_btns_ft = []
    for _i_ft, (_lbl_ft, (_e_ft, _a_ft)) in enumerate(_views_ft.items()):
        _bx_ft = fig.add_axes([0.18 + _i_ft * 0.06, 0.005, 0.055, 0.04])
        _b_ft = Button(_bx_ft, _lbl_ft, color=BTN_COLOR, hovercolor=BTN_HOVER)
        _b_ft.label.set_color(TEXT_COLOR)
        _b_ft.label.set_fontsize(7)
        _b_ft.on_clicked(lambda event, e=_e_ft, a=_a_ft:
                          (ax_bic.view_init(elev=e, azim=a), plt.draw()))
        _view_btns_ft.append(_b_ft)

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

            def _post_reset():
                time.sleep(2.0)
                if g_serial and g_serial.is_open:
                    g_serial.reset_input_buffer()
                    send_log_class_command(g_serial, LOG_CLASS_HEART_BEAT)

            threading.Thread(target=_post_reset, daemon=True).start()

    btn_reset.on_clicked(on_reset)

    # =========================================================================
    # Clear Button — wipes plot history without touching the FC
    # =========================================================================
    ax_clear = fig.add_axes([0.55, 0.005, 0.06, 0.04])
    btn_clear = Button(ax_clear, 'Clear',
                       color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_clear.label.set_color(TEXT_COLOR)
    btn_clear.label.set_fontsize(8)

    def on_clear(event):
        xy_history['x'].clear()
        xy_history['y'].clear()
        for k in vel_history:
            vel_history[k].clear()
        for k in alt_history:
            alt_history[k].clear()
        vel_t0[0] = None
        alt_t0[0] = None
        for ln in vel_lines:
            ln.set_data([], [])
        alt_line.set_data([], [])
        pos_trail.set_data([], [])
        pos_dot.set_data([], [])
        fig.canvas.draw_idle()

    btn_clear.on_clicked(on_clear)

    # =========================================================================
    # Animation loop
    # =========================================================================
    frame_count = [0]
    last_fps_time = [time.time()]

    def update(frame_num):
        updated = []

        # --- FPS / data-rate counter (runs every UI tick, even with no data) ---
        now_tick = time.time()
        frame_count[0] += 1
        elapsed_tick = now_tick - last_fps_time[0]
        if elapsed_tick >= 1.0:
            ui_hz = frame_count[0] / elapsed_tick
            rx_hz = g_rx_frame_count[0] / elapsed_tick
            g_rx_frame_count[0] = 0
            fps_text.set_text(f'Data:{rx_hz:4.0f} Hz | UI:{ui_hz:3.0f} Hz')
            frame_count[0] = 0
            last_fps_time[0] = now_tick
            updated.append(fps_text)

        with g_latest_lock:
            latest = g_latest[0]
            g_latest[0] = None

        if latest is None:
            return updated

        d = latest
        roll, pitch, yaw = d['att']
        motors = d['motors']

        # Bicopter outputs
        m1_speed = motors[0]  # left motor
        m2_speed = motors[1]  # right motor
        s1_pwm   = motors[4]  # left servo PWM µs
        s2_pwm   = motors[5]  # right servo PWM µs

        s1_angle = servo_to_angle_deg(s1_pwm)
        # S2 mounted opposite — negate for display
        s2_angle = -servo_to_angle_deg(s2_pwm)

        # =================================================================
        # 3D Bicopter
        # =================================================================
        R = make_rotation(roll, pitch, yaw)

        # Body points in world
        mount_l_w = R @ MOUNT_L
        mount_r_w = R @ MOUNT_R
        nose_tip_w  = R @ NOSE_TIP
        nose_base_w = R @ NOSE_BASE
        fuse_fwd = R @ np.array([0, FUSELAGE_LEN, 0])
        fuse_aft = R @ np.array([0, -FUSELAGE_LEN, 0])

        # Main beam
        beam_line.set_data_3d(
            [mount_l_w[0], mount_r_w[0]],
            [mount_l_w[1], mount_r_w[1]],
            [mount_l_w[2], mount_r_w[2]])

        # Fuselage
        fuselage_line.set_data_3d(
            [fuse_aft[0], fuse_fwd[0]],
            [fuse_aft[1], fuse_fwd[1]],
            [fuse_aft[2], fuse_fwd[2]])

        # Nose
        nose_line.set_data_3d(
            [nose_base_w[0], nose_tip_w[0]],
            [nose_base_w[1], nose_tip_w[1]],
            [nose_base_w[2], nose_tip_w[2]])

        # --- Tilting nacelles ---
        # Left nacelle: tilt about the body-frame beam axis (X)
        T_l = nacelle_tilt_matrix(s1_angle)
        nacelle_vec_l = T_l @ np.array([0, 0, NACELLE_LEN])
        motor_l_body = MOUNT_L + nacelle_vec_l
        motor_l_w = R @ motor_l_body

        nacelle_l_line.set_data_3d(
            [mount_l_w[0], motor_l_w[0]],
            [mount_l_w[1], motor_l_w[1]],
            [mount_l_w[2], motor_l_w[2]])

        # Right nacelle
        T_r = nacelle_tilt_matrix(s2_angle)
        nacelle_vec_r = T_r @ np.array([0, 0, NACELLE_LEN])
        motor_r_body = MOUNT_R + nacelle_vec_r
        motor_r_w = R @ motor_r_body

        nacelle_r_line.set_data_3d(
            [mount_r_w[0], motor_r_w[0]],
            [mount_r_w[1], motor_r_w[1]],
            [mount_r_w[2], motor_r_w[2]])

        # --- Prop discs (tilted with nacelle) ---
        r1 = 0.15 + np.clip(m1_speed / MAX_SPEED, 0, 1) * 0.55
        circle_local_l = (T_l @ (r1 * UNIT_CIRCLE).T).T + motor_l_body
        circle_w_l = (R @ circle_local_l.T).T
        prop_l_line.set_data_3d(circle_w_l[:, 0], circle_w_l[:, 1], circle_w_l[:, 2])
        prop_l_line.set_alpha(0.3 + np.clip(m1_speed / MAX_SPEED, 0, 1) * 0.6)

        # Right prop
        r2 = 0.15 + np.clip(m2_speed / MAX_SPEED, 0, 1) * 0.55
        circle_local_r = (T_r @ (r2 * UNIT_CIRCLE).T).T + motor_r_body
        circle_w_r = (R @ circle_local_r.T).T
        prop_r_line.set_data_3d(circle_w_r[:, 0], circle_w_r[:, 1], circle_w_r[:, 2])
        prop_r_line.set_alpha(0.3 + np.clip(m2_speed / MAX_SPEED, 0, 1) * 0.6)

        # Motor labels
        label_m1.set_position((motor_l_w[0], motor_l_w[1]))
        label_m1.set_3d_properties(motor_l_w[2] + 0.35, zdir='z')
        label_m2.set_position((motor_r_w[0], motor_r_w[1]))
        label_m2.set_3d_properties(motor_r_w[2] + 0.35, zdir='z')

        # =================================================================
        # Data panel values
        # =================================================================
        data_roll.set_text(f'R:{roll:+6.1f}\u00b0')
        data_pitch.set_text(f'P:{pitch:+6.1f}\u00b0')
        data_yaw.set_text(f'Y:{yaw:+6.1f}\u00b0')

        px_ned, py_ned, pz_ned = d['pos']
        alt_up = -pz_ned  # NED Z is down; show altitude positive-up
        data_px.set_text(f'N:{px_ned:+6.2f}m')
        data_py.set_text(f'E:{py_ned:+6.2f}m')
        data_pz.set_text(f'Alt:{alt_up:+6.2f}m')

        vn, ve, vd = d['vel']
        vu = -vd  # NED Vz is down; show vertical velocity positive-up
        data_vx.set_text(f'Vn:{vn:+5.2f}')
        data_vy.set_text(f'Ve:{ve:+5.2f}')
        data_vz.set_text(f'Vu:{vu:+5.2f}')

        data_m1.set_text(f'M1(L):{m1_speed:5d}')
        data_m2.set_text(f'M2(R):{m2_speed:5d}')
        data_s1.set_text(f'S1(L):{s1_angle:+5.1f}\u00b0')
        data_s2.set_text(f'S2(R):{s2_angle:+5.1f}\u00b0')

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
        vel_history['vx'].append(vn)
        vel_history['vy'].append(ve)
        vel_history['vz'].append(vu)
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
            all_v = vel_history['vx'] + vel_history['vy'] + vel_history['vz']
            ax_vel.set_ylim(*_robust_ylim(all_v, pad_frac=0.2, min_span=1.0))
        updated.extend(vel_lines)

        # =================================================================
        # Altitude chart
        # =================================================================
        if alt_t0[0] is None:
            alt_t0[0] = now
        t = now - alt_t0[0]
        alt_history['t'].append(t)
        alt_history['z'].append(alt_up)
        if len(alt_history['t']) > HISTORY_LEN:
            alt_history['t'] = alt_history['t'][-HISTORY_LEN:]
            alt_history['z'] = alt_history['z'][-HISTORY_LEN:]
        alt_line.set_data(alt_history['t'], alt_history['z'])
        if len(alt_history['t']) > 1:
            ax_alt.set_xlim(alt_history['t'][0], alt_history['t'][-1] + 0.1)
            ax_alt.set_ylim(*_robust_ylim(alt_history['z'], pad_frac=0.2, min_span=1.0))
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

        return updated

    from matplotlib.animation import FuncAnimation
    # FC publishes flight telemetry at 25 Hz; match the UI tick.
    ani = FuncAnimation(fig, update, interval=40, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
