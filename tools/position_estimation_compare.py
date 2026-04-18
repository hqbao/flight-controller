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
from matplotlib.animation import FuncAnimation
import time

"""
Position Estimation Fusion Comparison — Fusion5 vs Fusion4

Real-time overlay of Fusion5 (solid) and Fusion4 (dashed) in a 2x3 grid:
  Row 1: Pos X, Pos Y, Pos Z
  Row 2: Vel X, Vel Y, Vel Z

Frame layout (LOG_CLASS_POSITION_COMPARE = 0x1C, 48 bytes = 12 floats):
  float[0]  = F5 Pos X     float[6]  = F4 Pos X
  float[1]  = F5 Pos Y     float[7]  = F4 Pos Y
  float[2]  = F5 Pos Z     float[8]  = F4 Pos Z
  float[3]  = F5 Vel X     float[9]  = F4 Vel X
  float[4]  = F5 Vel Y     float[10] = F4 Vel Y
  float[5]  = F5 Vel Z     float[11] = F4 Vel Z

Usage:
  python3 position_estimation_compare.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 19200
SEND_LOG_ID = 0x00

LOG_CLASS_NONE      = 0x00
LOG_CLASS_HEART_BEAT = 0x09
LOG_CLASS_POSITION_COMPARE = 0x1C
DB_CMD_LOG_CLASS    = 0x03
DB_CMD_RESET       = 0x07
DB_CMD_CHIP_ID     = 0x09

FRAME_SIZE = 48  # 12 floats
HISTORY_LEN = 300  # ~30s at 10 Hz

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
    msg_id = DB_CMD_LOG_CLASS
    msg_class = 0x00
    length = 1
    payload = bytes([log_class])
    header = struct.pack('<2sBBH', b'db', msg_id, msg_class, length)
    checksum = (msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF) + log_class) & 0xFFFF
    frame = header + payload + struct.pack('<H', checksum)
    ser.write(frame)
    ser.flush()
    names = {0x00: 'NONE', 0x1C: 'POSITION_COMPARE'}
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


def serial_reader():
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
            elif msg_id == SEND_LOG_ID and length == FRAME_SIZE:
                vals = struct.unpack('<12f', payload)
                data_queue.put(vals)
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


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

    fig, axes = plt.subplots(2, 3, figsize=(18, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('Position Estimation \u2014 Fusion5 (solid) vs Fusion4 (dashed)',
                 fontsize=14, color=TEXT_COLOR, fontweight='bold', y=0.99)
    fig.subplots_adjust(left=0.05, right=0.97, top=0.93, bottom=0.10,
                        hspace=0.35, wspace=0.25)

    ax_px, ax_py, ax_pz = axes[0]
    ax_vx, ax_vy, ax_vz = axes[1]

    # Chart config: (ax, title, ylabel, xlabel)
    chart_cfg = [
        (ax_px, 'Pos X (Forward)', 'Position (m)', None),
        (ax_py, 'Pos Y (Right)',   'Position (m)', None),
        (ax_pz, 'Pos Z (Alt)',     'Position (m)', None),
        (ax_vx, 'Vel X (Forward)', 'Velocity (m/s)', 'Time (s)'),
        (ax_vy, 'Vel Y (Right)',   'Velocity (m/s)', 'Time (s)'),
        (ax_vz, 'Vel Z (Vert)',    'Velocity (m/s)', 'Time (s)'),
    ]

    for ax, title, ylabel, xlabel in chart_cfg:
        ax.set_title(title, color=DIM_TEXT, fontsize=10, pad=6)
        ax.set_ylabel(ylabel, fontsize=8)
        if xlabel:
            ax.set_xlabel(xlabel, fontsize=8)
        ax.grid(True, alpha=0.2, linestyle=':')
        ax.tick_params(labelsize=7)

    # Lines: solid=F5, dashed=F4
    line_f5_px, = ax_px.plot([], [], '-',  color=ACCENT_BLUE,   lw=1.2, label='F5')
    line_f4_px, = ax_px.plot([], [], '--', color=ACCENT_RED,    lw=1.2, label='F4')
    line_f5_py, = ax_py.plot([], [], '-',  color=ACCENT_GREEN,  lw=1.2, label='F5')
    line_f4_py, = ax_py.plot([], [], '--', color=ACCENT_ORANGE, lw=1.2, label='F4')
    line_f5_pz, = ax_pz.plot([], [], '-',  color=ACCENT_YELLOW, lw=1.2, label='F5')
    line_f4_pz, = ax_pz.plot([], [], '--', color=ACCENT_CYAN,   lw=1.2, label='F4')

    line_f5_vx, = ax_vx.plot([], [], '-',  color=ACCENT_BLUE,   lw=1.2, label='F5')
    line_f4_vx, = ax_vx.plot([], [], '--', color=ACCENT_RED,    lw=1.2, label='F4')
    line_f5_vy, = ax_vy.plot([], [], '-',  color=ACCENT_GREEN,  lw=1.2, label='F5')
    line_f4_vy, = ax_vy.plot([], [], '--', color=ACCENT_ORANGE, lw=1.2, label='F4')
    line_f5_vz, = ax_vz.plot([], [], '-',  color=ACCENT_YELLOW, lw=1.2, label='F5')
    line_f4_vz, = ax_vz.plot([], [], '--', color=ACCENT_CYAN,   lw=1.2, label='F4')

    for ax in [ax_px, ax_py, ax_pz, ax_vx, ax_vy, ax_vz]:
        ax.legend(loc='upper left', fontsize=7, framealpha=0.3)

    # Value text overlays
    texts = {}
    for ax, key in [(ax_px, 'px'), (ax_py, 'py'), (ax_pz, 'pz'),
                     (ax_vx, 'vx'), (ax_vy, 'vy'), (ax_vz, 'vz')]:
        texts[key] = ax.text(0.98, 0.95, '', transform=ax.transAxes,
                             fontsize=8, ha='right', va='top',
                             fontfamily='monospace', color=TEXT_COLOR)

    # History buffers
    keys = ['t',
            'f5_px', 'f5_py', 'f5_pz', 'f5_vx', 'f5_vy', 'f5_vz',
            'f4_px', 'f4_py', 'f4_pz', 'f4_vx', 'f4_vy', 'f4_vz']
    h = {k: [] for k in keys}
    t0 = [None]

    # Status bar
    chip_id_text = fig.text(0.96, 0.035, 'Chip ID: ---', fontsize=7,
                            ha='right', color=DIM_TEXT)
    fps_text = fig.text(0.96, 0.015, '', fontsize=8, ha='right', color=DIM_TEXT)

    # Buttons
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
                send_log_class_command(g_serial, LOG_CLASS_POSITION_COMPARE)
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

    # Animation loop
    frame_count = [0]
    last_fps_time = [time.time()]

    # Lines grouped for easy update: (f5_line, f4_line, f5_key, f4_key, ax, text_key)
    line_groups = [
        (line_f5_px, line_f4_px, 'f5_px', 'f4_px', ax_px, 'px'),
        (line_f5_py, line_f4_py, 'f5_py', 'f4_py', ax_py, 'py'),
        (line_f5_pz, line_f4_pz, 'f5_pz', 'f4_pz', ax_pz, 'pz'),
        (line_f5_vx, line_f4_vx, 'f5_vx', 'f4_vx', ax_vx, 'vx'),
        (line_f5_vy, line_f4_vy, 'f5_vy', 'f4_vy', ax_vy, 'vy'),
        (line_f5_vz, line_f4_vz, 'f5_vz', 'f4_vz', ax_vz, 'vz'),
    ]

    def update(frame_num):
        latest = None
        while not data_queue.empty():
            try:
                latest = data_queue.get_nowait()
            except queue.Empty:
                break
        if latest is None:
            return []

        (f5_px, f5_py, f5_pz, f5_vx, f5_vy, f5_vz,
         f4_px, f4_py, f4_pz, f4_vx, f4_vy, f4_vz) = latest

        now = time.time()
        if t0[0] is None:
            t0[0] = now
        t = now - t0[0]

        h['t'].append(t)
        h['f5_px'].append(f5_px); h['f5_py'].append(f5_py); h['f5_pz'].append(f5_pz)
        h['f5_vx'].append(f5_vx); h['f5_vy'].append(f5_vy); h['f5_vz'].append(f5_vz)
        h['f4_px'].append(f4_px); h['f4_py'].append(f4_py); h['f4_pz'].append(f4_pz)
        h['f4_vx'].append(f4_vx); h['f4_vy'].append(f4_vy); h['f4_vz'].append(f4_vz)

        if len(h['t']) > HISTORY_LEN:
            for k in h:
                h[k] = h[k][-HISTORY_LEN:]

        ta = np.array(h['t'])
        updated = []

        for ln5, ln4, k5, k4, ax, tk in line_groups:
            d5 = np.array(h[k5])
            d4 = np.array(h[k4])
            ln5.set_data(ta, d5)
            ln4.set_data(ta, d4)
            texts[tk].set_text(f'F5:{d5[-1]:+.3f}  F4:{d4[-1]:+.3f}')
            if len(ta) > 1:
                ax.set_xlim(ta[0], ta[-1] + 0.1)
                all_d = np.concatenate([d5, d4])
                dmin, dmax = all_d.min(), all_d.max()
                m = max(0.1, (dmax - dmin) * 0.2)
                ax.set_ylim(dmin - m, dmax + m)
            updated.extend([ln5, ln4])

        if g_chip_id is not None:
            chip_id_text.set_text(f'Chip ID: {g_chip_id}')

        frame_count[0] += 1
        elapsed = now - last_fps_time[0]
        if elapsed >= 1.0:
            fps = frame_count[0] / elapsed
            fps_text.set_text(f'{fps:.0f} Hz')
            frame_count[0] = 0
            last_fps_time[0] = now

        return updated

    ani = FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
