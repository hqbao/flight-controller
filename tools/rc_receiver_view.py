import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib
import sys
matplotlib.use('macosx' if sys.platform == 'darwin' else 'TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation
import time

"""
RC Receiver Debug Tool (rc_receiver_view.py)

Streams parsed RC receiver data from the flight controller and displays
real-time time-series of roll, pitch, yaw, alt commands plus state/mode
and message count to detect data corruption or dropouts.

Log class: LOG_CLASS_RC_RECEIVER (0x1B)
Frame: 7 floats (28 bytes) at 25 Hz:
  [roll] [pitch] [yaw] [alt] [state] [mode] [msg_count]

Usage:
  python3 tools/rc_receiver_view.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 38400
SEND_LOG_ID = 0x00

LOG_CLASS_NONE = 0x00
LOG_CLASS_HEART_BEAT = 0x09
LOG_CLASS_RC_RECEIVER = 0x1B
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET = 0x07

HISTORY_SEC = 15
UPDATE_HZ = 25
HISTORY_LEN = HISTORY_SEC * UPDATE_HZ

# --- Dark Theme ---
BG_COLOR = '#1e1e1e'
PANEL_COLOR = '#252526'
TEXT_COLOR = '#cccccc'
DIM_TEXT = '#888888'
GRID_COLOR = '#3c3c3c'
ROLL_COLOR = '#ff5555'
PITCH_COLOR = '#55ff55'
YAW_COLOR = '#5599ff'
ALT_COLOR = '#ffaa55'

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
    names = {0x00: 'NONE', 0x1B: 'RC_RECEIVER'}
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
        time.sleep(0.05)
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
            if not ((b1[0] == 0x64 and b2[0] == 0x62) or (b1[0] == 0x62 and b2[0] == 0x64)):
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

            if msg_id == SEND_LOG_ID and length == 28:  # 7 floats
                vals = struct.unpack('<7f', payload)
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

    fig = plt.figure(figsize=(14, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('RC Receiver Debug', fontsize=16,
                 color=TEXT_COLOR, fontweight='bold', y=0.97)

    # 3 rows: roll/pitch, yaw/alt, info panel
    ax_rp = fig.add_axes([0.08, 0.66, 0.60, 0.26])
    ax_ya = fig.add_axes([0.08, 0.36, 0.60, 0.26])
    ax_info = fig.add_axes([0.74, 0.36, 0.24, 0.56])

    # Bottom: message count time-series
    ax_cnt = fig.add_axes([0.08, 0.06, 0.60, 0.22])

    # --- Roll/Pitch chart ---
    ax_rp.set_ylabel('Degrees')
    ax_rp.set_title('Roll / Pitch', color=DIM_TEXT, fontsize=11)
    ax_rp.set_xlim(0, HISTORY_LEN)
    ax_rp.set_ylim(-50, 50)
    ax_rp.grid(True, alpha=0.3)
    ax_rp.axhline(y=0, color=GRID_COLOR, lw=1)
    line_roll, = ax_rp.plot([], [], color=ROLL_COLOR, lw=1.5, label='Roll')
    line_pitch, = ax_rp.plot([], [], color=PITCH_COLOR, lw=1.5, label='Pitch')
    ax_rp.legend(loc='upper right', fontsize=8, facecolor=PANEL_COLOR, edgecolor=GRID_COLOR)

    # --- Yaw/Alt chart ---
    ax_ya.set_ylabel('Degrees')
    ax_ya.set_title('Yaw / Alt', color=DIM_TEXT, fontsize=11)
    ax_ya.set_xlim(0, HISTORY_LEN)
    ax_ya.set_ylim(-50, 50)
    ax_ya.grid(True, alpha=0.3)
    ax_ya.axhline(y=0, color=GRID_COLOR, lw=1)
    line_yaw, = ax_ya.plot([], [], color=YAW_COLOR, lw=1.5, label='Yaw')
    line_alt, = ax_ya.plot([], [], color=ALT_COLOR, lw=1.5, label='Alt')
    ax_ya.legend(loc='upper right', fontsize=8, facecolor=PANEL_COLOR, edgecolor=GRID_COLOR)

    # --- Message count chart ---
    ax_cnt.set_ylabel('Msg Count')
    ax_cnt.set_xlabel('Samples')
    ax_cnt.set_title('Message Count (should increase steadily)', color=DIM_TEXT, fontsize=11)
    ax_cnt.set_xlim(0, HISTORY_LEN)
    ax_cnt.grid(True, alpha=0.3)
    line_cnt, = ax_cnt.plot([], [], color='#aaaaaa', lw=1.5)

    # --- Info panel ---
    ax_info.set_facecolor(PANEL_COLOR)
    ax_info.set_xticks([])
    ax_info.set_yticks([])
    ax_info.set_title('Live Values', color=DIM_TEXT, fontsize=11)
    for spine in ax_info.spines.values():
        spine.set_edgecolor(GRID_COLOR)

    info_text = ax_info.text(0.1, 0.85, '', fontsize=12, fontfamily='monospace',
                             color=TEXT_COLOR, va='top', transform=ax_info.transAxes)

    # --- Data buffers ---
    hist_roll = np.zeros(HISTORY_LEN)
    hist_pitch = np.zeros(HISTORY_LEN)
    hist_yaw = np.zeros(HISTORY_LEN)
    hist_alt = np.zeros(HISTORY_LEN)
    hist_cnt = np.zeros(HISTORY_LEN)
    x_data = np.arange(HISTORY_LEN)
    last_state = [0.0]
    last_mode = [0.0]
    last_cnt = [0.0]
    frame_count = [0]
    last_recv_cnt = [0.0]

    def update(frame):
        changed = False
        while not data_queue.empty():
            try:
                vals = data_queue.get_nowait()
            except queue.Empty:
                break
            changed = True

            roll, pitch, yaw, alt, state, mode, cnt = vals

            np.roll(hist_roll, -1)
            hist_roll[:-1] = hist_roll[1:]
            hist_roll[-1] = roll

            np.roll(hist_pitch, -1)
            hist_pitch[:-1] = hist_pitch[1:]
            hist_pitch[-1] = pitch

            np.roll(hist_yaw, -1)
            hist_yaw[:-1] = hist_yaw[1:]
            hist_yaw[-1] = yaw

            np.roll(hist_alt, -1)
            hist_alt[:-1] = hist_alt[1:]
            hist_alt[-1] = alt

            np.roll(hist_cnt, -1)
            hist_cnt[:-1] = hist_cnt[1:]
            hist_cnt[-1] = cnt

            last_state[0] = state
            last_mode[0] = mode
            last_cnt[0] = cnt
            frame_count[0] += 1

        if changed:
            line_roll.set_data(x_data, hist_roll)
            line_pitch.set_data(x_data, hist_pitch)
            line_yaw.set_data(x_data, hist_yaw)
            line_alt.set_data(x_data, hist_alt)
            line_cnt.set_data(x_data, hist_cnt)

            # Auto-scale message count axis
            cnt_min = np.min(hist_cnt[hist_cnt > 0]) if np.any(hist_cnt > 0) else 0
            cnt_max = np.max(hist_cnt) if np.max(hist_cnt) > 0 else 1
            ax_cnt.set_ylim(cnt_min - 1, cnt_max + 1)

            # Auto-scale roll/pitch
            rp_max = max(abs(np.min(hist_roll[-100:])), abs(np.max(hist_roll[-100:])),
                         abs(np.min(hist_pitch[-100:])), abs(np.max(hist_pitch[-100:])), 5)
            ax_rp.set_ylim(-rp_max * 1.2, rp_max * 1.2)

            # Auto-scale yaw/alt
            ya_max = max(abs(np.min(hist_yaw[-100:])), abs(np.max(hist_yaw[-100:])),
                         abs(np.min(hist_alt[-100:])), abs(np.max(hist_alt[-100:])), 5)
            ax_ya.set_ylim(-ya_max * 1.2, ya_max * 1.2)

            s = int(last_state[0])
            m = int(last_mode[0])

            info_str = (
                f"Roll:  {hist_roll[-1]:+7.1f}°\n"
                f"Pitch: {hist_pitch[-1]:+7.1f}°\n"
                f"Yaw:   {hist_yaw[-1]:+7.1f}°\n"
                f"Alt:   {hist_alt[-1]:+7.1f}°\n"
                f"\n"
                f"State: {s}\n"
                f"Mode:  {m}\n"
                f"\n"
                f"Msg#:  {int(last_cnt[0])}\n"
                f"Frames: {frame_count[0]}"
            )
            info_text.set_text(info_str)

        return line_roll, line_pitch, line_yaw, line_alt, line_cnt, info_text

    # --- Buttons ---
    ax_start = fig.add_axes([0.74, 0.15, 0.11, 0.05])
    ax_stop = fig.add_axes([0.86, 0.15, 0.11, 0.05])
    ax_reset = fig.add_axes([0.74, 0.08, 0.11, 0.05])
    ax_clear = fig.add_axes([0.86, 0.08, 0.11, 0.05])

    btn_start = Button(ax_start, 'Start Log', color='#2d5a2d', hovercolor='#3d7a3d')
    btn_stop = Button(ax_stop, 'Stop Log', color='#5a2d2d', hovercolor='#7a3d3d')
    btn_reset = Button(ax_reset, 'Reset FC', color='#5a2d2d', hovercolor='#7a3d3d')
    btn_clear = Button(ax_clear, 'Clear', color='#3c3c3c', hovercolor='#555555')

    def on_start(event):
        global g_logging_active
        if g_serial and g_serial.is_open:
            send_log_class_command(g_serial, LOG_CLASS_RC_RECEIVER)
            g_logging_active = True

    def on_stop(event):
        global g_logging_active
        if g_serial and g_serial.is_open:
            send_log_class_command(g_serial, LOG_CLASS_NONE)
            g_logging_active = False

    def on_reset(event):
        if g_serial and g_serial.is_open:
            send_reset_command(g_serial)

            def _post_reset():
                time.sleep(2.0)
                if g_serial and g_serial.is_open:
                    g_serial.reset_input_buffer()
                    send_log_class_command(g_serial, LOG_CLASS_HEART_BEAT)

            threading.Thread(target=_post_reset, daemon=True).start()

    def on_clear(event):
        hist_roll[:] = 0
        hist_pitch[:] = 0
        hist_yaw[:] = 0
        hist_alt[:] = 0
        hist_cnt[:] = 0
        frame_count[0] = 0

    btn_start.on_clicked(on_start)
    btn_stop.on_clicked(on_stop)
    btn_reset.on_clicked(on_reset)
    btn_clear.on_clicked(on_clear)

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
