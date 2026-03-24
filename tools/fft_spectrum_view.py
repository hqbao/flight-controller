import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation
import time

"""
FFT Spectrum & Peaks Visualization Tool (fft_spectrum_view.py)

Receives on-board computed FFT spectrum and dynamic notch peak frequencies
from the flight controller and displays a real-time scrolling spectrogram.

Spectrum mode (LOG_CLASS_FFT_SPECTRUM_X/Y/Z, 0x18-0x1A):
  Combined frame: [axis(1)] [52 uint8 dB-scaled bins, 0-200 Hz] [peak1(float)] [peak2(float)]
  = 61 bytes per frame, ~3.3 Hz per axis, peaks overlaid as colored traces.

Peaks-only mode (LOG_CLASS_FFT_PEAKS, 0x17):
  Frame: 6 floats = 24 bytes (3 axes × 2 peaks), 10 Hz.

Usage:
  python3 tools/fft_spectrum_view.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
SEND_LOG_ID = 0x00

LOG_CLASS_NONE = 0x00
LOG_CLASS_FFT_PEAKS = 0x17
LOG_CLASS_FFT_SPECTRUM_X = 0x18
LOG_CLASS_FFT_SPECTRUM_Y = 0x19
LOG_CLASS_FFT_SPECTRUM_Z = 0x1A
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET = 0x07

# FFT parameters (must match fft.c)
FFT_SIZE = 256
SAMPLE_HZ = 1000.0
FREQ_BIN_HZ = SAMPLE_HZ / FFT_SIZE        # ~3.906 Hz
SPECTRUM_MAX_HZ = 200.0
SPECTRUM_BINS = int(SPECTRUM_MAX_HZ / FREQ_BIN_HZ) + 1  # 52
NUM_PEAKS = 2

SPECTRUM_FRAME_SIZE = 1 + SPECTRUM_BINS + NUM_PEAKS * 4  # 61 bytes: axis + bins + 2 floats
PEAKS_FRAME_SIZE = 24                      # 6 floats (standalone peaks mode)
NUM_AXES = 3

SPECTROGRAM_COLS = 200   # ~60 seconds at 3.3 Hz/axis
UPDATE_RATE_HZ = 3.3     # ~10 Hz / 3 axes

# Axis map: index → (log_class, label)
AXIS_MAP = {
    0: (LOG_CLASS_FFT_SPECTRUM_X, 'X'),
    1: (LOG_CLASS_FFT_SPECTRUM_Y, 'Y'),
    2: (LOG_CLASS_FFT_SPECTRUM_Z, 'Z'),
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
PEAK_COLORS   = ['#ff5555', '#55ff55']

# --- Auto-detect serial port ---
ports = serial.tools.list_ports.comports()
print("Scanning for serial ports...")
for port, desc, hwid in sorted(ports):
    if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB']):
        SERIAL_PORT = port
        print(f"  \u2713 Auto-selected: {port} ({desc})")
        break
    else:
        print(f"  \u00b7 Skipped: {port} ({desc})")

if not SERIAL_PORT:
    print("  \u2717 No compatible serial port found.")

# --- Global State ---
spectrum_queue = queue.Queue()
peaks_queue = queue.Queue()
g_serial = None
g_logging_active = False
g_selected_axis = [0]  # default X


def send_log_class_command(ser, log_class):
    msg_id = DB_CMD_LOG_CLASS
    msg_class = 0x00
    length = 1
    payload = bytes([log_class])
    header = struct.pack('<2sBBH', b'db', msg_id, msg_class, length)
    checksum = (msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF) + log_class) & 0xFFFF
    frame = header + payload + struct.pack('<H', checksum)
    ser.write(frame)
    ser.write(frame)
    ser.flush()
    names = {0x00: 'NONE', 0x17: 'FFT_PEAKS',
             0x18: 'SPECTRUM_X', 0x19: 'SPECTRUM_Y', 0x1A: 'SPECTRUM_Z'}
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

            if length == SPECTRUM_FRAME_SIZE:
                axis = payload[0]
                bins = np.frombuffer(payload[1:1+SPECTRUM_BINS], dtype=np.uint8)
                spectrum_queue.put((axis, bins.copy()))
                # Extract appended peak frequencies
                peak_data = payload[1+SPECTRUM_BINS:]
                if len(peak_data) == NUM_PEAKS * 4:
                    pf = struct.unpack(f'<{NUM_PEAKS}f', peak_data)
                    peaks_queue.put((axis, pf))
            elif length == PEAKS_FRAME_SIZE:
                vals = struct.unpack('<6f', payload)
                peaks_queue.put((-1, vals))  # -1 = all-axes mode
            else:
                print(f"  ? Unknown frame: len={length}")
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


# --- GUI ---
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

    fig, (ax_spec, ax_snap) = plt.subplots(
        1, 2, figsize=(16, 7),
        gridspec_kw={'width_ratios': [4, 1]})
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('FFT Spectrum \u2014 On-Board (10 Hz)',
                 fontsize=13, color=TEXT_COLOR, fontweight='bold', y=0.98)
    plt.subplots_adjust(bottom=0.13, top=0.92, left=0.06, right=0.97, wspace=0.08)

    # Frequency axis (0 to 200 Hz)
    freq_axis = np.arange(SPECTRUM_BINS) * FREQ_BIN_HZ

    # --- Spectrogram (left) ---
    spec_data = np.zeros((SPECTRUM_BINS, SPECTROGRAM_COLS), dtype=np.float32)
    time_axis = np.arange(SPECTROGRAM_COLS) / UPDATE_RATE_HZ
    img = ax_spec.imshow(
        spec_data, aspect='auto', origin='lower',
        extent=[0, time_axis[-1], 0, SPECTRUM_MAX_HZ],
        cmap='inferno', vmin=0, vmax=255, interpolation='bilinear')
    ax_spec.set_xlabel('Time (s)', color=TEXT_COLOR)
    ax_spec.set_ylabel('Frequency (Hz)', color=TEXT_COLOR)
    title_text = ax_spec.set_title('Axis: X', fontsize=11, color=TEXT_COLOR, loc='left')

    # Peak overlay lines on spectrogram
    peak_lines = []
    for p in range(NUM_PEAKS):
        line, = ax_spec.plot([], [], color=PEAK_COLORS[p], linewidth=1.5,
                             linestyle='--', alpha=0.8, label=f'Peak {p+1}')
        peak_lines.append(line)
    ax_spec.legend(loc='upper right', fontsize=7, framealpha=0.3,
                   facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                   labelcolor=TEXT_COLOR)

    # Track peak history for overlay
    peak_history = [[[] for _ in range(NUM_PEAKS)] for _ in range(NUM_AXES)]
    col_count = [0]

    # --- Snapshot bar chart (right) ---
    bars = ax_snap.barh(freq_axis, np.zeros(SPECTRUM_BINS),
                        height=FREQ_BIN_HZ * 0.8, color='#ff9944', alpha=0.8)
    ax_snap.set_xlim(0, 260)
    ax_snap.set_ylim(0, SPECTRUM_MAX_HZ)
    ax_snap.set_xlabel('Magnitude', color=TEXT_COLOR, fontsize=9)
    ax_snap.set_yticklabels([])
    ax_snap.set_title('Latest', fontsize=10, color=DIM_TEXT)

    # Peak frequency labels
    peak_label = ax_snap.text(0.05, 0.97, '', transform=ax_snap.transAxes,
                              fontsize=9, fontfamily='monospace',
                              color=TEXT_COLOR, fontweight='bold',
                              verticalalignment='top')

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
            title_text.set_text(f'Axis: {lbl}')
            if g_logging_active and g_serial and g_serial.is_open:
                send_log_class_command(g_serial, lc)
            # Clear spectrogram
            spec_data[:, :] = 0
            col_count[0] = 0
            for a in range(NUM_AXES):
                for p in range(NUM_PEAKS):
                    peak_history[a][p].clear()

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
                ax_toggle.set_facecolor(BTN_GREEN)
            else:
                lc = AXIS_MAP[g_selected_axis[0]][0]
                send_log_class_command(g_serial, lc)
                g_logging_active = True
                btn_toggle.label.set_text('Stop Log')
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
            ax_toggle.set_facecolor(BTN_GREEN)

    btn_reset.on_clicked(on_reset)

    ax_clear = fig.add_axes([0.90, 0.02, 0.07, 0.04])
    btn_clear = Button(ax_clear, 'Clear', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_clear.label.set_color(TEXT_COLOR)
    btn_clear.label.set_fontsize(8)

    def on_clear(event):
        spec_data[:, :] = 0
        col_count[0] = 0
        for a in range(NUM_AXES):
            for p in range(NUM_PEAKS):
                peak_history[a][p].clear()

    btn_clear.on_clicked(on_clear)

    # FPS label
    fps_text = fig.text(0.50, 0.02, '', fontsize=7, color=DIM_TEXT, ha='center')
    frame_count = [0]
    last_fps_time = [time.time()]

    # --- Animation ---
    def update(frame_num):
        changed = False

        # Drain spectrum queue
        latest_spec = None
        while not spectrum_queue.empty():
            try:
                latest_spec = spectrum_queue.get_nowait()
            except queue.Empty:
                break

        if latest_spec is not None:
            axis, bins = latest_spec
            # Shift spectrogram left, append new column
            spec_data[:, :-1] = spec_data[:, 1:]
            spec_data[:, -1] = bins[:SPECTRUM_BINS].astype(np.float32)
            col_count[0] += 1

            # Update spectrogram image
            t_end = col_count[0] / UPDATE_RATE_HZ
            t_start = max(0, t_end - SPECTROGRAM_COLS / UPDATE_RATE_HZ)
            img.set_data(spec_data)
            img.set_extent([t_start, t_end, 0, SPECTRUM_MAX_HZ])

            # Update snapshot bars
            for bar, val in zip(bars, bins[:SPECTRUM_BINS]):
                bar.set_width(float(val))

            changed = True

        # Drain peaks queue (for overlay)
        latest_peaks = None
        while not peaks_queue.empty():
            try:
                latest_peaks = peaks_queue.get_nowait()
            except queue.Empty:
                break

        if latest_peaks is not None:
            axis_idx, pf = latest_peaks
            sel = g_selected_axis[0]
            if axis_idx == -1:
                # Standalone peaks mode (all axes): pf is 6 floats
                for p in range(NUM_PEAKS):
                    freq = pf[sel * NUM_PEAKS + p]
                    peak_history[sel][p].append(freq)
                    while len(peak_history[sel][p]) > SPECTROGRAM_COLS:
                        peak_history[sel][p].pop(0)
            elif axis_idx == sel:
                # Combined spectrum+peaks mode: pf is 2 floats for this axis
                for p in range(NUM_PEAKS):
                    peak_history[sel][p].append(pf[p])
                    while len(peak_history[sel][p]) > SPECTROGRAM_COLS:
                        peak_history[sel][p].pop(0)

        # Update peak overlay lines
        sel = g_selected_axis[0]
        t_end = col_count[0] / UPDATE_RATE_HZ
        t_start = max(0, t_end - SPECTROGRAM_COLS / UPDATE_RATE_HZ)
        label_parts = []
        for p in range(NUM_PEAKS):
            h = peak_history[sel][p]
            if len(h) > 0:
                n = len(h)
                t_vals = np.linspace(t_start, t_end, n)
                peak_lines[p].set_data(t_vals, h)
                label_parts.append(f'P{p+1}: {h[-1]:.1f} Hz')
            else:
                peak_lines[p].set_data([], [])
                label_parts.append(f'P{p+1}: ---')
        peak_label.set_text('\n'.join(label_parts))

        # FPS
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

