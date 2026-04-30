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
FFT Dual Spectrum Visualization Tool (fft_spectrum_dual_view.py)

Receives RAW + POST-NOTCH gyro FFT spectrums from the flight controller
and displays them stacked (raw on top, filtered on bottom) so notch filter
effectiveness can be verified live.

Frame layout (LOG_CLASS_FFT_SPECTRUM_DUAL_X/Y/Z, 0x14-0x16):
  [axis(1)] [raw_bins(103)] [filt_bins(103)]
  [raw_peak1(float)] [raw_peak2(float)]
  [filt_peak1(float)] [filt_peak2(float)]
  = 1 + 103 + 103 + 16 = 223 bytes per frame, 10 Hz (~58% of 38400 baud)

Usage:
  python3 tools/fft_spectrum_dual_view.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 38400
SEND_LOG_ID = 0x00

LOG_CLASS_NONE = 0x00
LOG_CLASS_HEART_BEAT = 0x09
LOG_CLASS_FFT_SPECTRUM_DUAL_X = 0x14
LOG_CLASS_FFT_SPECTRUM_DUAL_Y = 0x15
LOG_CLASS_FFT_SPECTRUM_DUAL_Z = 0x16
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET = 0x07

# FFT parameters (must match fft.c)
FFT_SIZE = 256
SAMPLE_HZ = 1000.0
FREQ_BIN_HZ = SAMPLE_HZ / FFT_SIZE        # ~3.906 Hz
SPECTRUM_MAX_HZ = 400.0
SPECTRUM_BINS = int(SPECTRUM_MAX_HZ / FREQ_BIN_HZ) + 1  # 103
NUM_PEAKS = 3

DUAL_FRAME_SIZE = 1 + 2 * SPECTRUM_BINS + 2 * NUM_PEAKS * 4  # 223 bytes
NUM_AXES = 3

SPECTROGRAM_COLS = 200
UPDATE_RATE_HZ = 10.0  # one axis at a time, 10 Hz

AXIS_MAP = {
    0: (LOG_CLASS_FFT_SPECTRUM_DUAL_X, 'X'),
    1: (LOG_CLASS_FFT_SPECTRUM_DUAL_Y, 'Y'),
    2: (LOG_CLASS_FFT_SPECTRUM_DUAL_Z, 'Z'),
}

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
PEAK_COLORS   = ['#ff5555', '#55ff55', '#5599ff']

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
# Each item: (axis, raw_bins, filt_bins, raw_peaks, filt_peaks)
frame_queue = queue.Queue()
g_serial = None
g_logging_active = False
g_selected_axis = [0]


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
    names = {0x00: 'NONE',
             0x14: 'DUAL_X', 0x15: 'DUAL_Y', 0x16: 'DUAL_Z'}
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

            if msg_id != SEND_LOG_ID:
                continue

            if length == DUAL_FRAME_SIZE:
                axis = payload[0]
                off = 1
                raw_bins = np.frombuffer(
                    payload[off:off + SPECTRUM_BINS], dtype=np.uint8).copy()
                off += SPECTRUM_BINS
                filt_bins = np.frombuffer(
                    payload[off:off + SPECTRUM_BINS], dtype=np.uint8).copy()
                off += SPECTRUM_BINS
                raw_peaks = struct.unpack(f'<{NUM_PEAKS}f',
                                          payload[off:off + NUM_PEAKS * 4])
                off += NUM_PEAKS * 4
                filt_peaks = struct.unpack(f'<{NUM_PEAKS}f',
                                           payload[off:off + NUM_PEAKS * 4])
                frame_queue.put((axis, raw_bins, filt_bins, raw_peaks, filt_peaks))
            else:
                print(f"  ? Unknown frame: len={length}")
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


def main():
    global g_logging_active

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

    # 2 rows (raw on top, filtered on bottom), 2 columns (spectrogram + snapshot bar)
    fig, axes = plt.subplots(
        2, 2, figsize=screen_fit_figsize(16, 9),
        gridspec_kw={'width_ratios': [4, 1], 'hspace': 0.18, 'wspace': 0.08},
        sharey='row')
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('FFT Dual Spectrum \u2014 Raw vs Post-Notch (10 Hz)',
                 fontsize=13, color=TEXT_COLOR, fontweight='bold', y=0.98)
    plt.subplots_adjust(bottom=0.10, top=0.93, left=0.06, right=0.97)

    ax_spec_raw, ax_snap_raw = axes[0]
    ax_spec_filt, ax_snap_filt = axes[1]

    freq_axis = np.arange(SPECTRUM_BINS) * FREQ_BIN_HZ
    time_axis = np.arange(SPECTROGRAM_COLS) / UPDATE_RATE_HZ

    # --- Spectrograms ---
    spec_raw = np.zeros((SPECTRUM_BINS, SPECTROGRAM_COLS), dtype=np.float32)
    spec_filt = np.zeros((SPECTRUM_BINS, SPECTROGRAM_COLS), dtype=np.float32)

    img_raw = ax_spec_raw.imshow(
        spec_raw, aspect='auto', origin='lower',
        extent=[0, time_axis[-1], 0, SPECTRUM_MAX_HZ],
        cmap='inferno', vmin=0, vmax=255, interpolation='bilinear')
    img_filt = ax_spec_filt.imshow(
        spec_filt, aspect='auto', origin='lower',
        extent=[0, time_axis[-1], 0, SPECTRUM_MAX_HZ],
        cmap='inferno', vmin=0, vmax=255, interpolation='bilinear')

    ax_spec_raw.set_ylabel('Frequency (Hz)', color=TEXT_COLOR)
    ax_spec_filt.set_ylabel('Frequency (Hz)', color=TEXT_COLOR)
    ax_spec_filt.set_xlabel('Time (s)', color=TEXT_COLOR)

    title_raw = ax_spec_raw.set_title(
        'RAW gyro \u2014 Axis: X', fontsize=11, color=TEXT_COLOR, loc='left')
    title_filt = ax_spec_filt.set_title(
        'POST-NOTCH gyro \u2014 Axis: X', fontsize=11, color='#55ff55', loc='left')

    # Peak overlay lines
    peak_lines_raw = []
    peak_lines_filt = []
    for p in range(NUM_PEAKS):
        l1, = ax_spec_raw.plot([], [], color=PEAK_COLORS[p], linewidth=1.5,
                               linestyle='--', alpha=0.8, label=f'Peak {p+1}')
        l2, = ax_spec_filt.plot([], [], color=PEAK_COLORS[p], linewidth=1.5,
                                linestyle='--', alpha=0.8, label=f'Peak {p+1}')
        peak_lines_raw.append(l1)
        peak_lines_filt.append(l2)
    ax_spec_raw.legend(loc='upper right', fontsize=7, framealpha=0.3,
                       facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                       labelcolor=TEXT_COLOR)
    ax_spec_filt.legend(loc='upper right', fontsize=7, framealpha=0.3,
                        facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                        labelcolor=TEXT_COLOR)

    # Per-axis history of (raw_peaks, filt_peaks)
    peak_history_raw = [[[] for _ in range(NUM_PEAKS)] for _ in range(NUM_AXES)]
    peak_history_filt = [[[] for _ in range(NUM_PEAKS)] for _ in range(NUM_AXES)]
    col_count = [0]

    # --- Snapshot bar charts ---
    bars_raw = ax_snap_raw.barh(freq_axis, np.zeros(SPECTRUM_BINS),
                                height=FREQ_BIN_HZ * 0.8, color='#ff9944', alpha=0.8)
    bars_filt = ax_snap_filt.barh(freq_axis, np.zeros(SPECTRUM_BINS),
                                  height=FREQ_BIN_HZ * 0.8, color='#44aaff', alpha=0.8)
    for ax_snap, ttl in ((ax_snap_raw, 'Raw'), (ax_snap_filt, 'Filtered')):
        ax_snap.set_xlim(0, 260)
        ax_snap.set_ylim(0, SPECTRUM_MAX_HZ)
        ax_snap.set_yticklabels([])
        ax_snap.set_title(ttl, fontsize=10, color=DIM_TEXT)
    ax_snap_filt.set_xlabel('Magnitude', color=TEXT_COLOR, fontsize=9)

    peak_label_raw = ax_snap_raw.text(
        0.05, 0.97, '', transform=ax_snap_raw.transAxes,
        fontsize=9, fontfamily='monospace',
        color=TEXT_COLOR, fontweight='bold', verticalalignment='top')
    peak_label_filt = ax_snap_filt.text(
        0.05, 0.97, '', transform=ax_snap_filt.transAxes,
        fontsize=9, fontfamily='monospace',
        color=TEXT_COLOR, fontweight='bold', verticalalignment='top')

    # --- Axis selector buttons ---
    axis_btns = []
    for i, (_, label) in AXIS_MAP.items():
        bx = fig.add_axes([0.06 + i * 0.05, 0.02, 0.045, 0.04])
        b = Button(bx, label, color=BTN_COLOR, hovercolor=BTN_HOVER)
        b.label.set_color(TEXT_COLOR)
        b.label.set_fontsize(9)

        def on_axis(event, idx=i):
            g_selected_axis[0] = idx
            lc = AXIS_MAP[idx][0]
            lbl = AXIS_MAP[idx][1]
            title_raw.set_text(f'RAW gyro \u2014 Axis: {lbl}')
            title_filt.set_text(f'POST-NOTCH gyro \u2014 Axis: {lbl}')
            if g_logging_active and g_serial and g_serial.is_open:
                send_log_class_command(g_serial, lc)
            while not frame_queue.empty():
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    break
            spec_raw[:, :] = 0
            spec_filt[:, :] = 0
            col_count[0] = 0
            for a in range(NUM_AXES):
                for p in range(NUM_PEAKS):
                    peak_history_raw[a][p].clear()
                    peak_history_filt[a][p].clear()

        b.on_clicked(on_axis)
        axis_btns.append(b)

    # --- Control buttons ---
    ax_toggle = fig.add_axes([0.72, 0.02, 0.08, 0.04])
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
                lc = AXIS_MAP[g_selected_axis[0]][0]
                send_log_class_command(g_serial, lc)
                g_logging_active = True
                btn_toggle.label.set_text('Stop Log')
                btn_toggle.color = BTN_RED
                btn_toggle.hovercolor = BTN_RED_HOV
                ax_toggle.set_facecolor(BTN_RED)

    btn_toggle.on_clicked(on_toggle)

    ax_reset = fig.add_axes([0.81, 0.02, 0.08, 0.04])
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

    ax_clear = fig.add_axes([0.90, 0.02, 0.07, 0.04])
    btn_clear = Button(ax_clear, 'Clear', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_clear.label.set_color(TEXT_COLOR)
    btn_clear.label.set_fontsize(8)

    def on_clear(event):
        spec_raw[:, :] = 0
        spec_filt[:, :] = 0
        col_count[0] = 0
        for a in range(NUM_AXES):
            for p in range(NUM_PEAKS):
                peak_history_raw[a][p].clear()
                peak_history_filt[a][p].clear()

    btn_clear.on_clicked(on_clear)

    fps_text = fig.text(0.50, 0.02, '', fontsize=7, color=DIM_TEXT, ha='center')
    frame_count = [0]
    last_fps_time = [time.time()]

    def update(frame_num):
        latest = None
        sel = g_selected_axis[0]
        while not frame_queue.empty():
            try:
                item = frame_queue.get_nowait()
                if item[0] == sel:
                    latest = item
            except queue.Empty:
                break

        if latest is not None:
            axis, raw_bins, filt_bins, raw_peaks, filt_peaks = latest

            spec_raw[:, :-1] = spec_raw[:, 1:]
            spec_raw[:, -1] = raw_bins[:SPECTRUM_BINS].astype(np.float32)
            spec_filt[:, :-1] = spec_filt[:, 1:]
            spec_filt[:, -1] = filt_bins[:SPECTRUM_BINS].astype(np.float32)
            col_count[0] += 1

            t_end = col_count[0] / UPDATE_RATE_HZ
            t_start = max(0, t_end - SPECTROGRAM_COLS / UPDATE_RATE_HZ)
            img_raw.set_data(spec_raw)
            img_raw.set_extent([t_start, t_end, 0, SPECTRUM_MAX_HZ])
            img_filt.set_data(spec_filt)
            img_filt.set_extent([t_start, t_end, 0, SPECTRUM_MAX_HZ])

            for bar, val in zip(bars_raw, raw_bins[:SPECTRUM_BINS]):
                bar.set_width(float(val))
            for bar, val in zip(bars_filt, filt_bins[:SPECTRUM_BINS]):
                bar.set_width(float(val))

            for p in range(NUM_PEAKS):
                peak_history_raw[axis][p].append(raw_peaks[p])
                peak_history_filt[axis][p].append(filt_peaks[p])
                while len(peak_history_raw[axis][p]) > SPECTROGRAM_COLS:
                    peak_history_raw[axis][p].pop(0)
                while len(peak_history_filt[axis][p]) > SPECTROGRAM_COLS:
                    peak_history_filt[axis][p].pop(0)

        # Refresh peak overlay + labels for currently selected axis
        sel = g_selected_axis[0]
        t_end = col_count[0] / UPDATE_RATE_HZ
        t_start = max(0, t_end - SPECTROGRAM_COLS / UPDATE_RATE_HZ)
        raw_label_parts = []
        filt_label_parts = []
        for p in range(NUM_PEAKS):
            hr = peak_history_raw[sel][p]
            hf = peak_history_filt[sel][p]
            if len(hr) > 0:
                tr = np.linspace(t_start, t_end, len(hr))
                peak_lines_raw[p].set_data(tr, hr)
                raw_label_parts.append(f'P{p+1}: {hr[-1]:.1f} Hz')
            else:
                peak_lines_raw[p].set_data([], [])
                raw_label_parts.append(f'P{p+1}: ---')
            if len(hf) > 0:
                tf = np.linspace(t_start, t_end, len(hf))
                peak_lines_filt[p].set_data(tf, hf)
                v = hf[-1]
                if v > 0.0:
                    filt_label_parts.append(f'P{p+1}: {v:.1f} Hz')
                else:
                    filt_label_parts.append(f'P{p+1}: ---')
            else:
                peak_lines_filt[p].set_data([], [])
                filt_label_parts.append(f'P{p+1}: ---')
        peak_label_raw.set_text('\n'.join(raw_label_parts))
        peak_label_filt.set_text('\n'.join(filt_label_parts))

        frame_count[0] += 1
        now = time.time()
        elapsed = now - last_fps_time[0]
        if elapsed >= 1.0:
            fps = frame_count[0] / elapsed
            fps_text.set_text(f'{fps:.0f} fps')
            frame_count[0] = 0
            last_fps_time[0] = now

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()

    if g_serial and g_serial.is_open:
        send_log_class_command(g_serial, LOG_CLASS_NONE)


if __name__ == '__main__':
    main()
