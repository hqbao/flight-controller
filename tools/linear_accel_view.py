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
Linear Acceleration Time-Series Viewer

Diagnoses post-crash vibration rectification / DC accel bias by streaming
v_linear_acc (gravity-removed acceleration) and showing:
  - Live time-series of la.x / la.y / la.z
  - Running mean (critical: non-zero mean under vibration = rectification bias)
  - Running std and peak

Reuses existing firmware log classes — no firmware change required.
  LOG_CLASS_ATTITUDE       (0x03): body-frame v_linear_acc
  LOG_CLASS_ATTITUDE_EARTH (0x13): earth-frame v_linear_acc

Bench test workflow:
  1. Motors OFF, drone at rest: means should be near zero (|la| < 0.05 m/s^2)
  2. Motors ON at hover (drone restrained): any DC shift in the mean means
     vibration is biasing the accelerometer. la.z drifting positive while
     hovering is a classic sign of a bent prop / damaged motor bearing.

Frame (36 bytes, 9 floats): v_pred[3], v_true[3], v_linear_acc[3]
la is in units of g in the firmware — we scale to m/s^2 for display.

Usage:
  python3 linear_accel_view.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 38400
SEND_LOG_ID = 0x00

LOG_CLASS_NONE           = 0x00
LOG_CLASS_ATTITUDE       = 0x03
LOG_CLASS_HEART_BEAT     = 0x09
LOG_CLASS_ATTITUDE_EARTH = 0x13
DB_CMD_LOG_CLASS         = 0x03
DB_CMD_RESET             = 0x07

ATTITUDE_FRAME_SIZE = 36  # 9 floats
GRAVITY_MSS         = 9.80665
HISTORY_SECONDS     = 30.0
SAMPLE_HZ           = 10.0            # attitude_estimation loop_logger rate
HISTORY_POINTS      = int(HISTORY_SECONDS * SAMPLE_HZ)

# --- UI Colors ---
BG_COLOR      = '#1e1e1e'
PANEL_COLOR   = '#252526'
TEXT_COLOR    = '#cccccc'
DIM_TEXT      = '#888888'
GRID_COLOR    = '#3c3c3c'
BTN_COLOR     = '#333333'
BTN_HOVER     = '#444444'
BTN_GREEN     = '#2d5a2d'
BTN_GREEN_HOV = '#3d7a3d'
BTN_RED       = '#5a2d2d'
BTN_RED_HOV   = '#7a3d3d'
COLOR_X       = '#ff5555'  # red
COLOR_Y       = '#55dd55'  # green
COLOR_Z       = '#5599ff'  # blue
MEAN_COLOR    = '#ffcc44'

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
g_earth_view = False  # False = body frame, True = earth frame


def _build_frame(cmd_id, payload):
    msg_class = 0x00
    length = len(payload)
    header = struct.pack('<2sBBH', b'db', cmd_id, msg_class, length)
    cs = cmd_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
    for b in payload:
        cs += b
    cs &= 0xFFFF
    return header + payload + struct.pack('<H', cs)


def send_log_class_command(ser, log_class):
    ser.write(_build_frame(DB_CMD_LOG_CLASS, bytes([log_class])))
    ser.flush()
    names = {0x00: 'NONE', 0x03: 'ATTITUDE', 0x09: 'HEART_BEAT', 0x13: 'ATTITUDE_EARTH'}
    print(f"  \u2192 Log class: {names.get(log_class, f'0x{log_class:02X}')}")


def send_reset_command(ser):
    ser.write(_build_frame(DB_CMD_RESET, bytes([0x00])))
    ser.flush()
    print("  \u2192 Reset command sent")


def serial_reader():
    global g_serial
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

            _ = ser.read(1)  # class byte

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

            if msg_id == SEND_LOG_ID and length == ATTITUDE_FRAME_SIZE:
                vals = struct.unpack('<9f', payload)
                # la is at float[6..8], in units of g -> convert to m/s^2
                la = (vals[6] * GRAVITY_MSS,
                      vals[7] * GRAVITY_MSS,
                      vals[8] * GRAVITY_MSS)
                data_queue.put(la)
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


# --- GUI ---
def main():
    global g_logging_active, g_earth_view

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

    fig, (ax_x, ax_y, ax_z) = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    fit_to_screen_height(fig)
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('Linear Acceleration \u2014 Vibration Bias Diagnostic',
                 fontsize=13, color=TEXT_COLOR, fontweight='bold', y=0.98)
    plt.subplots_adjust(bottom=0.12, top=0.92, left=0.07, right=0.97, hspace=0.22)

    frame_label = fig.text(0.5, 0.945, 'Body Frame (X=Fwd  Y=Right  Z=Down-inverted)',
                           fontsize=10, ha='center', color=MEAN_COLOR,
                           fontweight='bold')

    # Ring buffers
    buf_x = np.zeros(HISTORY_POINTS, dtype=np.float32)
    buf_y = np.zeros(HISTORY_POINTS, dtype=np.float32)
    buf_z = np.zeros(HISTORY_POINTS, dtype=np.float32)
    t_axis = np.linspace(-HISTORY_SECONDS, 0, HISTORY_POINTS)
    fill_count = [0]

    def setup_axis(ax, color, label):
        ax.set_facecolor(BG_COLOR)
        ax.grid(True, alpha=0.25)
        ax.axhline(0, color=DIM_TEXT, lw=0.5, alpha=0.5)
        mean_line = ax.axhline(0, color=MEAN_COLOR, lw=1.0, ls='--', alpha=0.7)
        line, = ax.plot(t_axis, np.zeros(HISTORY_POINTS), color=color, lw=1.0)
        ax.set_ylabel(f'{label}\n(m/s\u00b2)', color=TEXT_COLOR)
        ax.set_ylim(-2.0, 2.0)
        stats = ax.text(0.005, 0.97, '', transform=ax.transAxes,
                        fontsize=9, fontfamily='monospace',
                        color=TEXT_COLOR, verticalalignment='top',
                        bbox=dict(facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                                  boxstyle='round,pad=0.3', alpha=0.8))
        return line, mean_line, stats

    line_x, mean_x, stats_x = setup_axis(ax_x, COLOR_X, 'la.x')
    line_y, mean_y, stats_y = setup_axis(ax_y, COLOR_Y, 'la.y')
    line_z, mean_z, stats_z = setup_axis(ax_z, COLOR_Z, 'la.z')
    ax_z.set_xlabel('Time (s, relative)', color=TEXT_COLOR)

    # --- Buttons ---
    ax_toggle = fig.add_axes([0.55, 0.02, 0.09, 0.04])
    btn_toggle = Button(ax_toggle, 'Start Log',
                        color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_toggle.label.set_color(TEXT_COLOR)
    btn_toggle.label.set_fontsize(9)

    def active_log_class():
        return LOG_CLASS_ATTITUDE_EARTH if g_earth_view else LOG_CLASS_ATTITUDE

    def on_toggle(event):
        global g_logging_active
        if not (g_serial and g_serial.is_open):
            return
        if g_logging_active:
            send_log_class_command(g_serial, LOG_CLASS_NONE)
            g_logging_active = False
            btn_toggle.label.set_text('Start Log')
            btn_toggle.color = BTN_GREEN
            btn_toggle.hovercolor = BTN_GREEN_HOV
            ax_toggle.set_facecolor(BTN_GREEN)
        else:
            send_log_class_command(g_serial, active_log_class())
            g_logging_active = True
            btn_toggle.label.set_text('Stop Log')
            btn_toggle.color = BTN_RED
            btn_toggle.hovercolor = BTN_RED_HOV
            ax_toggle.set_facecolor(BTN_RED)

    btn_toggle.on_clicked(on_toggle)

    ax_frame = fig.add_axes([0.65, 0.02, 0.09, 0.04])
    btn_frame = Button(ax_frame, 'Body/Earth', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_frame.label.set_color(TEXT_COLOR)
    btn_frame.label.set_fontsize(9)

    def on_frame(event):
        global g_earth_view
        g_earth_view = not g_earth_view
        if g_earth_view:
            frame_label.set_text('Earth Frame (X=North  Y=East  Z=Up)')
        else:
            frame_label.set_text('Body Frame (X=Fwd  Y=Right  Z=Down-inverted)')
        # Clear buffers on frame switch
        buf_x[:] = 0; buf_y[:] = 0; buf_z[:] = 0
        fill_count[0] = 0
        while not data_queue.empty():
            try:
                data_queue.get_nowait()
            except queue.Empty:
                break
        if g_logging_active and g_serial and g_serial.is_open:
            send_log_class_command(g_serial, active_log_class())

    btn_frame.on_clicked(on_frame)

    ax_reset = fig.add_axes([0.75, 0.02, 0.09, 0.04])
    btn_reset = Button(ax_reset, 'Reset FC',
                       color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_reset.label.set_color(TEXT_COLOR)
    btn_reset.label.set_fontsize(9)

    def on_reset(event):
        global g_logging_active
        if not (g_serial and g_serial.is_open):
            return
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

    ax_clear = fig.add_axes([0.85, 0.02, 0.09, 0.04])
    btn_clear = Button(ax_clear, 'Clear', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_clear.label.set_color(TEXT_COLOR)
    btn_clear.label.set_fontsize(9)

    def on_clear(event):
        buf_x[:] = 0; buf_y[:] = 0; buf_z[:] = 0
        fill_count[0] = 0

    btn_clear.on_clicked(on_clear)

    # --- Animation ---
    def update(_frame):
        # Drain queue into ring buffer
        consumed = 0
        while True:
            try:
                la = data_queue.get_nowait()
            except queue.Empty:
                break
            buf_x[:-1] = buf_x[1:]; buf_x[-1] = la[0]
            buf_y[:-1] = buf_y[1:]; buf_y[-1] = la[1]
            buf_z[:-1] = buf_z[1:]; buf_z[-1] = la[2]
            fill_count[0] = min(fill_count[0] + 1, HISTORY_POINTS)
            consumed += 1
            if consumed > 200:
                break  # don't block the UI

        line_x.set_ydata(buf_x)
        line_y.set_ydata(buf_y)
        line_z.set_ydata(buf_z)

        n = fill_count[0]
        if n > 2:
            # Use only the filled portion of the ring (last n samples)
            vx = buf_x[-n:]; vy = buf_y[-n:]; vz = buf_z[-n:]
            mx, my, mz = float(vx.mean()), float(vy.mean()), float(vz.mean())
            sx, sy, sz = float(vx.std()),  float(vy.std()),  float(vz.std())
            px, py, pz = float(np.abs(vx).max()), float(np.abs(vy).max()), float(np.abs(vz).max())

            mean_x.set_ydata([mx, mx])
            mean_y.set_ydata([my, my])
            mean_z.set_ydata([mz, mz])

            stats_x.set_text(f'mean {mx:+.3f}   std {sx:.3f}   |peak| {px:.2f}')
            stats_y.set_text(f'mean {my:+.3f}   std {sy:.3f}   |peak| {py:.2f}')
            stats_z.set_text(f'mean {mz:+.3f}   std {sz:.3f}   |peak| {pz:.2f}')

            # Autoscale Y when data exceeds default window
            for ax, v in [(ax_x, vx), (ax_y, vy), (ax_z, vz)]:
                lim = max(2.0, float(np.abs(v).max()) * 1.2)
                ax.set_ylim(-lim, lim)

        return line_x, line_y, line_z, mean_x, mean_y, mean_z

    ani = FuncAnimation(fig, update, interval=50, blit=False,
                        cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
