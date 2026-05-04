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
# --- inlined view helpers (no external dependency) ---
FACE_VIEWS = {
    'Top':    (90, 180),
    'Bottom': (-90, 0),
    'Front':  (0, 0),
    'Back':   (0, 180),
    'Left':   (0, 90),
    'Right':  (0, -90),
}

def fit_to_screen_height(fig, margin_px=80):
    """Resize ``fig`` so its height matches the host screen height
    (preserving the original ``figsize`` aspect ratio).  No-op if the
    screen size cannot be determined."""
    import sys as _sys
    sw = sh = 0
    try:
        if _sys.platform == 'darwin':
            import ctypes as _c, ctypes.util as _cu
            _lib = _c.CDLL(_cu.find_library('ApplicationServices'))
            class _Sz(_c.Structure):
                _fields_ = [('w', _c.c_double), ('h', _c.c_double)]
            class _Pt(_c.Structure):
                _fields_ = [('x', _c.c_double), ('y', _c.c_double)]
            class _Rc(_c.Structure):
                _fields_ = [('o', _Pt), ('s', _Sz)]
            _lib.CGMainDisplayID.restype = _c.c_uint32
            _lib.CGDisplayBounds.restype = _Rc
            _lib.CGDisplayBounds.argtypes = [_c.c_uint32]
            r = _lib.CGDisplayBounds(_lib.CGMainDisplayID())
            sw, sh = int(r.s.w), int(r.s.h)
        else:
            import tkinter as _tk
            _r = _tk.Tk(); _r.withdraw()
            sw = _r.winfo_screenwidth(); sh = _r.winfo_screenheight()
            _r.destroy()
    except Exception:
        return
    if sh <= 0 or sw <= 0:
        return
    th = max(400, sh - margin_px)
    wi, hi = fig.get_size_inches()
    if hi <= 0:
        return
    asp = wi / hi
    tw = int(th * asp)
    if tw > sw - margin_px:
        tw = max(400, sw - margin_px)
        th = int(tw / asp)
    # Use rc logical DPI (not fig.get_dpi(), which doubles on HiDPI/retina
    # and would halve the resulting window).
    import matplotlib as _mpl
    _ldpi = _mpl.rcParams.get("figure.dpi", 100) or 100
    try:
        fig.set_size_inches(tw / _ldpi, th / _ldpi, forward=True)
    except Exception:
        pass

def add_3d_view_buttons(fig, ax, default='Back',
                        rect=(0.02, 0.05, 0.06, 0.035), gap=0.005,
                        color='#2a3140', hover='#3a4255',
                        text_color='#cfd6e4', fontsize=7):
    """Add a row of 6 face-view buttons (Top, Bottom, Front, Back, Left,
    Right) to ``fig`` controlling the 3D ``ax``.  Sets ``default`` view."""
    from matplotlib.widgets import Button as _Btn
    x0, y0, w, h = rect
    btns = []
    for i, (label, (elev, azim)) in enumerate(FACE_VIEWS.items()):
        bx = fig.add_axes([x0 + i * (w + gap), y0, w, h])
        b = _Btn(bx, label, color=color, hovercolor=hover)
        b.label.set_color(text_color)
        b.label.set_fontsize(fontsize)
        b.on_clicked(
            lambda event, e=elev, a=azim:
            (ax.view_init(elev=e, azim=a), fig.canvas.draw_idle()))
        btns.append(b)
    if default in FACE_VIEWS:
        e, a = FACE_VIEWS[default]
        ax.view_init(elev=e, azim=a)
    return btns
# --- end inlined view helpers ---
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation
import time

"""
Position Estimation Dashboard (2D + Altitude)

Real-time visualization of drone position estimation output.
  Center:       2D Position Map (XY top-down) with trail history
  Left panel:   Numeric data readout (position, velocity, distance)
  Right-Top:    Altitude (Z) time series
  Right-Bottom: Velocity time series (Vx, Vy, Vz)

Frame layout (LOG_CLASS_POSITION = 0x04, 24 bytes):
  float[0] = Pos X (Pitch/Forward, meters)
  float[1] = Pos Y (Roll/Right, meters)
  float[2] = Pos Z (Altitude, meters)
  float[3] = Vel X (Pitch, m/s)
  float[4] = Vel Y (Roll, m/s)
  float[5] = Vel Z (Vertical, m/s)

Usage:
  python3 position_estimation_2d_and_z.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 38400
SEND_LOG_ID = 0x00

LOG_CLASS_NONE      = 0x00
LOG_CLASS_HEART_BEAT = 0x09
LOG_CLASS_POSITION  = 0x04
DB_CMD_LOG_CLASS    = 0x03
DB_CMD_RESET       = 0x07
DB_CMD_CHIP_ID     = 0x09

POSITION_FRAME_SIZE = 24  # 6 floats
HISTORY_LEN = 300         # ~30s at 10 Hz

# --- UI Colors ---
BG_COLOR       = '#1e1e1e'
PANEL_COLOR    = '#252526'
TEXT_COLOR     = '#cccccc'
DIM_TEXT       = '#888888'
GRID_COLOR     = '#3c3c3c'
ACCENT_BLUE    = '#5599ff'
ACCENT_GREEN   = '#55cc55'
ACCENT_RED     = '#ff5555'
ACCENT_ORANGE  = '#ff9955'
ACCENT_YELLOW  = '#ffcc55'
ACCENT_CYAN    = '#55cccc'
BTN_GREEN      = '#2d5a2d'
BTN_GREEN_HOV  = '#3d7a3d'
BTN_RED        = '#5a2d2d'
BTN_RED_HOV    = '#7a3d3d'

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
    names = {0x00: 'NONE', 0x04: 'POSITION'}
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


def serial_reader():
    """Background thread: reads DB frames from FC, parses position payloads."""
    global g_serial, g_chip_id
    if not SERIAL_PORT:
        return
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(b'\x00' * 32)
        ser.flush()
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
            elif msg_id == SEND_LOG_ID and length == POSITION_FRAME_SIZE:
                vals = struct.unpack('<6f', payload)
                data_queue.put(vals)
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

    fig = plt.figure(figsize=(16, 9))
    fit_to_screen_height(fig)
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('Position Estimation \u2014 2D + Altitude', fontsize=14,
                 color=TEXT_COLOR, fontweight='bold', y=0.99)

    # =========================================================================
    # 2D Position Map (center)
    # =========================================================================
    ax_xy = fig.add_axes([0.12, 0.10, 0.48, 0.84])
    ax_xy.set_facecolor(BG_COLOR)
    ax_xy.set_aspect('equal')
    ax_xy.set_xlabel('Y \u2014 Roll / East (m)', fontsize=9)
    ax_xy.set_ylabel('X \u2014 Pitch / North (m)', fontsize=9)
    ax_xy.grid(True, alpha=0.2, linestyle=':')
    ax_xy.set_xlim(-3, 3)
    ax_xy.set_ylim(-3, 3)
    ax_xy.tick_params(labelsize=7)
    for spine in ax_xy.spines.values():
        spine.set_edgecolor(GRID_COLOR)

    # Concentric distance rings
    for r in [1, 2, 3, 5, 10]:
        circle = plt.Circle((0, 0), r, color=GRID_COLOR, fill=False,
                             linewidth=0.5, alpha=0.3, linestyle='--')
        ax_xy.add_patch(circle)

    # Origin crosshair
    ax_xy.axhline(0, color=GRID_COLOR, linewidth=0.5, alpha=0.4)
    ax_xy.axvline(0, color=GRID_COLOR, linewidth=0.5, alpha=0.4)

    # North indicator
    ax_xy.annotate('N', xy=(0, 0), xytext=(0, 2.7),
                   fontsize=9, color=ACCENT_RED, alpha=0.5,
                   ha='center', va='center')

    # Position trail + dot
    pos_trail, = ax_xy.plot([], [], '-', color=ACCENT_CYAN, linewidth=1.2, alpha=0.5)
    pos_dot, = ax_xy.plot([], [], 'o', color=ACCENT_CYAN, markersize=8, zorder=5)

    # Velocity arrow
    vel_arrow = ax_xy.quiver(0, 0, 0, 0, color=ACCENT_ORANGE, scale=1,
                             scale_units='xy', angles='xy', width=0.005,
                             zorder=4, alpha=0.8)

    xy_history = {'x': [], 'y': []}
    xy_zoom = [None]

    def on_scroll_xy(event):
        if event.inaxes != ax_xy:
            return
        xlim = ax_xy.get_xlim()
        cur_half = (xlim[1] - xlim[0]) / 2
        factor = 0.8 if event.button == 'up' else 1.25
        new_half = max(0.5, min(100.0, cur_half * factor))
        xy_zoom[0] = new_half

    fig.canvas.mpl_connect('scroll_event', on_scroll_xy)

    # =========================================================================
    # Left data panel
    # =========================================================================
    ax_data = fig.add_axes([0.005, 0.10, 0.105, 0.84])
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

    # Section: Position
    ax_data.text(0.08, 0.95, 'POSITION', **_hdr)
    ax_data.plot([0.05, 0.95], [0.945, 0.945], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_px = ax_data.text(0.08, 0.91, 'X:  0.00m', color=ACCENT_CYAN, **_val)
    data_py = ax_data.text(0.08, 0.87, 'Y:  0.00m', color=ACCENT_CYAN, **_val)
    data_pz = ax_data.text(0.08, 0.83, 'Z:  0.00m', color=ACCENT_YELLOW, **_val)

    # Section: Velocity
    ax_data.text(0.08, 0.76, 'VELOCITY', **_hdr)
    ax_data.plot([0.05, 0.95], [0.755, 0.755], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_vx = ax_data.text(0.08, 0.72, 'Vx: 0.00', color=ACCENT_BLUE, **_val)
    data_vy = ax_data.text(0.08, 0.68, 'Vy: 0.00', color=ACCENT_GREEN, **_val)
    data_vz = ax_data.text(0.08, 0.64, 'Vz: 0.00', color=ACCENT_ORANGE, **_val)

    # Section: Distance / Speed
    ax_data.text(0.08, 0.57, 'METRICS', **_hdr)
    ax_data.plot([0.05, 0.95], [0.565, 0.565], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_dist  = ax_data.text(0.08, 0.53, 'D:  0.00m', color=TEXT_COLOR, **_val)
    data_speed = ax_data.text(0.08, 0.49, 'S:  0.00',  color=TEXT_COLOR, **_val)

    # =========================================================================
    # Altitude chart (right top)
    # =========================================================================
    ax_alt = fig.add_axes([0.67, 0.55, 0.31, 0.38])
    ax_alt.set_facecolor(BG_COLOR)
    ax_alt.patch.set_alpha(0.8)
    ax_alt.set_title('Altitude (Z)', color=DIM_TEXT, fontsize=9, pad=4)
    ax_alt.set_xlabel('t (s)', fontsize=7)
    ax_alt.set_ylabel('m', fontsize=7)
    ax_alt.tick_params(labelsize=7)
    ax_alt.grid(True, alpha=0.2, linestyle=':')
    for spine in ax_alt.spines.values():
        spine.set_edgecolor(GRID_COLOR)
        spine.set_alpha(0.5)
    alt_line, = ax_alt.plot([], [], '-', color=ACCENT_YELLOW, linewidth=1.2)
    alt_val_text = ax_alt.text(0.98, 0.92, '', transform=ax_alt.transAxes,
                               fontsize=8, ha='right', va='top',
                               fontfamily='monospace', color=ACCENT_YELLOW)
    alt_history = {'t': [], 'z': []}
    alt_t0 = [None]

    # =========================================================================
    # Velocity chart (right bottom)
    # =========================================================================
    ax_vel = fig.add_axes([0.67, 0.10, 0.31, 0.38])
    ax_vel.set_facecolor(BG_COLOR)
    ax_vel.patch.set_alpha(0.8)
    ax_vel.set_title('Velocity', color=DIM_TEXT, fontsize=9, pad=4)
    ax_vel.set_xlabel('t (s)', fontsize=7)
    ax_vel.set_ylabel('m/s', fontsize=7)
    ax_vel.tick_params(labelsize=7)
    ax_vel.grid(True, alpha=0.2, linestyle=':')
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
    # Status bar
    # =========================================================================
    chip_id_text = fig.text(0.96, 0.035, 'Chip ID: ---', fontsize=7,
                            ha='right', color=DIM_TEXT)
    fps_text = fig.text(0.96, 0.015, '', fontsize=8, ha='right',
                        color=DIM_TEXT)

    # =========================================================================
    # Buttons
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
                send_log_class_command(g_serial, LOG_CLASS_POSITION)
                g_logging_active = True
                btn_toggle.label.set_text('Stop Log')
                btn_toggle.color = BTN_RED
                btn_toggle.hovercolor = BTN_RED_HOV
                ax_toggle.set_facecolor(BTN_RED)

    btn_toggle.on_clicked(on_toggle)

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

        px, py, pz, vx, vy, vz = latest

        # =================================================================
        # Data panel
        # =================================================================
        data_px.set_text(f'X:{px:+6.2f}m')
        data_py.set_text(f'Y:{py:+6.2f}m')
        data_pz.set_text(f'Z:{pz:+6.2f}m')

        data_vx.set_text(f'Vx:{vx:+5.2f}')
        data_vy.set_text(f'Vy:{vy:+5.2f}')
        data_vz.set_text(f'Vz:{vz:+5.2f}')

        dist = np.sqrt(px**2 + py**2)
        speed = np.sqrt(vx**2 + vy**2 + vz**2)
        data_dist.set_text(f'D:{dist:6.2f}m')
        data_speed.set_text(f'S:{speed:5.2f}')

        # =================================================================
        # 2D Position Map
        # =================================================================
        xy_history['x'].append(py)   # Y->East mapped to display X
        xy_history['y'].append(px)   # X->North mapped to display Y
        if len(xy_history['x']) > HISTORY_LEN:
            xy_history['x'] = xy_history['x'][-HISTORY_LEN:]
            xy_history['y'] = xy_history['y'][-HISTORY_LEN:]
        pos_trail.set_data(xy_history['x'], xy_history['y'])
        pos_dot.set_data([py], [px])

        # Update velocity arrow
        vel_arrow.set_offsets([[py, px]])
        vel_arrow.set_UVC(vy, vx)

        # Auto-zoom
        if xy_zoom[0] is not None:
            half = xy_zoom[0]
            ax_xy.set_xlim(py - half, py + half)
            ax_xy.set_ylim(px - half, px + half)
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
        # Altitude chart
        # =================================================================
        now = time.time()
        if alt_t0[0] is None:
            alt_t0[0] = now
        at = now - alt_t0[0]
        alt_history['t'].append(at)
        alt_history['z'].append(pz)
        if len(alt_history['t']) > HISTORY_LEN:
            alt_history['t'] = alt_history['t'][-HISTORY_LEN:]
            alt_history['z'] = alt_history['z'][-HISTORY_LEN:]
        alt_line.set_data(alt_history['t'], alt_history['z'])
        alt_val_text.set_text(f'Z:{pz:+.3f}m')
        if len(alt_history['t']) > 1:
            ax_alt.set_xlim(alt_history['t'][0], alt_history['t'][-1] + 0.1)
            zarr = np.array(alt_history['z'])
            zmin, zmax = zarr.min(), zarr.max()
            margin = max(0.5, (zmax - zmin) * 0.2)
            ax_alt.set_ylim(zmin - margin, zmax + margin)
        updated.append(alt_line)

        # =================================================================
        # Velocity chart
        # =================================================================
        if vel_t0[0] is None:
            vel_t0[0] = now
        vt = now - vel_t0[0]
        vel_history['t'].append(vt)
        vel_history['vx'].append(vx)
        vel_history['vy'].append(vy)
        vel_history['vz'].append(vz)
        if len(vel_history['t']) > HISTORY_LEN:
            for k in vel_history:
                vel_history[k] = vel_history[k][-HISTORY_LEN:]
        vel_lines[0].set_data(vel_history['t'], vel_history['vx'])
        vel_lines[1].set_data(vel_history['t'], vel_history['vy'])
        vel_lines[2].set_data(vel_history['t'], vel_history['vz'])
        if len(vel_history['t']) > 1:
            ax_vel.set_xlim(vel_history['t'][0], vel_history['t'][-1] + 0.1)
            all_v = np.array(vel_history['vx'] + vel_history['vy'] + vel_history['vz'])
            vmin, vmax = all_v.min(), all_v.max()
            vm = max(0.5, (vmax - vmin) * 0.2)
            ax_vel.set_ylim(vmin - vm, vmax + vm)
        updated.extend(vel_lines)

        # =================================================================
        # Chip ID + FPS
        # =================================================================
        if g_chip_id is not None:
            chip_id_text.set_text(f'Chip ID: {g_chip_id}')
            updated.append(chip_id_text)

        frame_count[0] += 1
        elapsed = now - last_fps_time[0]
        if elapsed >= 1.0:
            fps = frame_count[0] / elapsed
            fps_text.set_text(f'{fps:.0f} Hz')
            frame_count[0] = 0
            last_fps_time[0] = now
            updated.append(fps_text)

        return updated

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
