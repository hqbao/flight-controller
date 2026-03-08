import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation
from matplotlib.colors import LogNorm
from collections import deque
import time

"""
FFT Spectrogram (Waterfall) Tool

Displays a scrolling spectrogram of gyro Z-axis vibration over time.
  X-axis: Time (seconds)
  Y-axis: Frequency (Hz)
  Color:  Magnitude (°/s)

Same data stream as fft_view.py — batched 50 x int16 at 250 Hz.
Useful for observing how vibration frequencies shift with motor RPM,
throttle changes, or maneuvers.

Setup:
  1. Build & flash firmware
  2. Run: python3 tools/fft_spectrogram.py
  3. Click "Start Log" to begin streaming
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
SEND_LOG_ID = 0x00

# Log class constants (match messages.h)
LOG_CLASS_NONE      = 0x00
LOG_CLASS_IMU_GYRO  = 0x05
DB_CMD_LOG_CLASS    = 0x03

# FFT Parameters
SAMPLE_RATE = 250       # Hz (1 kHz / 4 decimation)
FFT_SIZE = 256          # Samples per FFT window (shorter for better time resolution)
FFT_HOP = 50            # Samples between consecutive FFTs (= 1 batch = 200ms)
BATCH_SIZE = 50         # int16 samples per DB frame
GYRO_FFT_SCALE = 10.0   # int16 -> deg/s

# Spectrogram display
SPECTROGRAM_COLS = 150  # Number of time columns to display (~30 seconds at 5 fps)
FREQ_MAX = 125          # Max frequency to display (Nyquist)

# Auto-detect serial port
ports = serial.tools.list_ports.comports()
found_port = False
print("Scanning for serial ports...")
for port, desc, hwid in sorted(ports):
    if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB']):
        SERIAL_PORT = port
        found_port = True
        print(f"Auto-selected Port: {port} ({desc})")
        break
    else:
        print(f"Skipped: {port} ({desc})")

if not found_port:
    print('----------------------------------------------------')
    print('ERROR: No compatible serial port found.')
    print('Please connect the Flight Controller and try again.')
    print('----------------------------------------------------')


# --- Global State ---
data_queue = queue.Queue(maxsize=5000)
g_serial = None
sample_buf = deque(maxlen=FFT_SIZE * 4)  # Rolling sample buffer

# Spectrogram matrix: rows = freq bins, cols = time slices
n_freq_bins = FFT_SIZE // 2 + 1
spectrogram = np.zeros((n_freq_bins, SPECTROGRAM_COLS))
spec_time = 0.0  # Elapsed time counter


# --- Log Class Command ---
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
    print(f"  \u2192 Log class set to 0x{log_class:02X}")


# --- Serial Reader ---
def serial_reader():
    global SERIAL_PORT, g_serial
    if not SERIAL_PORT:
        return

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            g_serial = ser
            print(f"Connected to {SERIAL_PORT} @ {BAUD_RATE} baud")
            while True:
                b1 = ser.read(1)
                if not b1:
                    continue
                if b1[0] != 0x64 and b1[0] != 0x62:
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

                expected_size = BATCH_SIZE * 2
                if msg_id == SEND_LOG_ID and length == expected_size:
                    samples = struct.unpack(f'<{BATCH_SIZE}h', payload)
                    data_queue.put(samples)

    except Exception as e:
        print(f"Serial error: {e}")


# --- GUI ---
def main():
    global g_serial, spectrogram, spec_time

    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    fig, (ax_t, ax_s) = plt.subplots(2, 1, figsize=(14, 8),
                                      gridspec_kw={'height_ratios': [1, 3]})
    fig.suptitle(f'Gyro Z Spectrogram  (Fs={SAMPLE_RATE} Hz, FFT={FFT_SIZE}, Hop={FFT_HOP})',
                 fontsize=13)
    plt.subplots_adjust(bottom=0.12)

    # --- Time domain plot (top) ---
    time_data = np.zeros(FFT_SIZE)
    line_t, = ax_t.plot(np.arange(FFT_SIZE), time_data, 'b-', linewidth=0.5)
    ax_t.set_ylabel('Gyro Z (°/s)')
    ax_t.set_xlim(0, FFT_SIZE)
    ax_t.set_ylim(-30, 30)
    ax_t.grid(True, alpha=0.3)
    ax_t.set_xlabel('Sample')
    ax_t.set_title('Time Domain (latest window)')

    # --- Spectrogram plot (bottom) ---
    freq_axis = np.fft.rfftfreq(FFT_SIZE, d=1.0 / SAMPLE_RATE)
    time_extent = SPECTROGRAM_COLS * FFT_HOP / SAMPLE_RATE  # seconds

    im = ax_s.imshow(spectrogram,
                     aspect='auto',
                     origin='lower',
                     cmap='inferno',
                     extent=[0, time_extent, 0, FREQ_MAX],
                     vmin=0, vmax=2.0,
                     interpolation='bilinear')
    ax_s.set_ylabel('Frequency (Hz)')
    ax_s.set_xlabel('Time (s)')
    ax_s.set_title('Spectrogram')
    cbar = fig.colorbar(im, ax=ax_s, label='Magnitude (°/s)', pad=0.02)

    # Peak text
    peak_txt = ax_s.text(0.98, 0.95, '', transform=ax_s.transAxes,
                         ha='right', va='top', fontsize=10, color='white',
                         bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))

    # Sample count
    count_txt = ax_t.text(0.02, 0.95, '', transform=ax_t.transAxes,
                          ha='left', va='top', fontsize=10,
                          bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))

    # --- Buttons ---
    btn_log_ax = plt.axes([0.05, 0.02, 0.12, 0.04])
    btn_log = Button(btn_log_ax, 'Start Log', color='#335533', hovercolor='#557755')

    def start_log(event):
        if g_serial and g_serial.is_open:
            send_log_class_command(g_serial, LOG_CLASS_IMU_GYRO)
        else:
            print('Serial not connected')

    btn_log.on_clicked(start_log)

    btn_stop_ax = plt.axes([0.18, 0.02, 0.12, 0.04])
    btn_stop = Button(btn_stop_ax, 'Stop Log', color='#553333', hovercolor='#775555')

    def stop_log(event):
        if g_serial and g_serial.is_open:
            send_log_class_command(g_serial, LOG_CLASS_NONE)
        else:
            print('Serial not connected')

    btn_stop.on_clicked(stop_log)

    btn_clear_ax = plt.axes([0.31, 0.02, 0.12, 0.04])
    btn_clear = Button(btn_clear_ax, 'Clear', color='#444444', hovercolor='#666666')

    def clear_data(event):
        global spectrogram
        sample_buf.clear()
        spectrogram = np.zeros((n_freq_bins, SPECTROGRAM_COLS))
        while not data_queue.empty():
            try:
                data_queue.get_nowait()
            except queue.Empty:
                break

    btn_clear.on_clicked(clear_data)

    plt.tight_layout(rect=[0, 0.06, 1, 0.95])

    # Track how many new samples since last FFT
    pending_samples = [0]

    def update(frame):
        global spectrogram

        # Drain queue into sample buffer
        got_data = False
        while not data_queue.empty():
            try:
                batch = data_queue.get_nowait()
                for raw in batch:
                    sample_buf.append(raw / GYRO_FFT_SCALE)
                pending_samples[0] += len(batch)
                got_data = True
            except queue.Empty:
                break

        n = len(sample_buf)
        count_txt.set_text(f'Buffer: {n}/{FFT_SIZE}')

        if n < FFT_SIZE:
            return im, line_t, peak_txt, count_txt

        # Compute new FFT columns for each hop worth of new data
        while pending_samples[0] >= FFT_HOP:
            pending_samples[0] -= FFT_HOP

            # Take latest FFT_SIZE samples
            signal = np.array(list(sample_buf))[-FFT_SIZE:]

            # Hanning window + FFT
            windowed = signal * np.hanning(FFT_SIZE)
            mag = np.abs(np.fft.rfft(windowed)) / FFT_SIZE * 2
            mag[0] /= 2

            # Scroll spectrogram left, add new column on right
            spectrogram = np.roll(spectrogram, -1, axis=1)
            spectrogram[:, -1] = mag

        # Update time domain (latest window)
        signal = np.array(list(sample_buf))[-FFT_SIZE:]
        line_t.set_ydata(signal)
        mx = max(np.max(np.abs(signal)), 5)
        ax_t.set_ylim(-mx * 1.3, mx * 1.3)

        # Update spectrogram image
        max_mag = np.max(spectrogram)
        if max_mag > 0.1:
            im.set_clim(vmin=0, vmax=max_mag * 0.8)
        im.set_data(spectrogram)

        # Peak frequency from latest column
        latest_col = spectrogram[:, -1]
        if len(latest_col) > 1:
            pk = np.argmax(latest_col[1:]) + 1
            peak_txt.set_text(f'Peak: {freq_axis[pk]:.1f} Hz\nMag: {latest_col[pk]:.2f} °/s')

        return im, line_t, peak_txt, count_txt

    ani = FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════════════════╗
║              GYRO FFT SPECTROGRAM TOOL                        ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  Scrolling spectrogram of gyro Z vibration over time.         ║
║  X = Time, Y = Frequency, Color = Magnitude                  ║
║                                                               ║
║  Data: 250 Hz gyro Z via UART (9600 baud)                     ║
║  FFT: 256-point, 50-sample hop (~200ms per column)            ║
║  Display: ~30 seconds of history, 0-125 Hz                    ║
║                                                               ║
║  USAGE:                                                       ║
║    1. Connect flight controller via USB                       ║
║    2. Click "Start Log" to begin streaming                    ║
║    3. Watch frequency patterns evolve over time               ║
║    4. Click "Stop Log" when done                              ║
║                                                               ║
║  LOOK FOR:                                                    ║
║    ✓ Horizontal lines = constant vibration frequency          ║
║    ✓ Rising lines = frequency increasing with RPM             ║
║    ✓ Bright spots = resonance at specific throttle            ║
║    ✓ Broadband = bearing noise or loose hardware              ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
""")
    main()
