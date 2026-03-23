import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation
from collections import deque
import time

"""
Gyro FFT Visualization Tool

Streams gyro data from the flight controller and displays
real-time time-domain and FFT frequency spectrum for vibration analysis.
Supports X, Y, Z axis selection via buttons.

Data format: Batched 50 x int16 samples at 250 Hz (1 kHz decimated by 4).
Each int16 = gyro Z in 0.1 deg/s units (divide by 10 to get deg/s).

Use this to identify:
  - Motor vibration frequencies (1x, 2x motor RPM)
  - Frame resonance
  - Prop imbalance
  - PID oscillation

Setup:
  1. Build & flash firmware (LOG_CLASS_IMU_GYRO is runtime-selectable)
  2. Run: python3 tools/gyro_fft_view.py
  3. Click "Start Log" to begin streaming
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
SEND_LOG_ID = 0x00

# Log class constants (match messages.h)
LOG_CLASS_NONE      = 0x00
LOG_CLASS_IMU_ACCEL_RAW = 0x01
LOG_CLASS_COMPASS   = 0x02
LOG_CLASS_ATTITUDE  = 0x03
LOG_CLASS_POSITION  = 0x04
LOG_CLASS_FFT_GYRO_Z = 0x05
LOG_CLASS_POSITION_OPTFLOW = 0x06
LOG_CLASS_ATTITUDE_MAG = 0x07
LOG_CLASS_FFT_GYRO_X = 0x0E
LOG_CLASS_FFT_GYRO_Y = 0x0F
LOG_CLASS_FFT_GYRO_FILTERED_X = 0x14
LOG_CLASS_FFT_GYRO_FILTERED_Y = 0x15
LOG_CLASS_FFT_GYRO_FILTERED_Z = 0x16
DB_CMD_LOG_CLASS    = 0x03
DB_CMD_RESET        = 0x07

# Axis selection: maps axis index to (log_class, label)
AXIS_MAP = {
    0: (LOG_CLASS_FFT_GYRO_X, 'X'),
    1: (LOG_CLASS_FFT_GYRO_Y, 'Y'),
    2: (LOG_CLASS_FFT_GYRO_Z, 'Z'),
    3: (LOG_CLASS_FFT_GYRO_FILTERED_X, 'fX'),
    4: (LOG_CLASS_FFT_GYRO_FILTERED_Y, 'fY'),
    5: (LOG_CLASS_FFT_GYRO_FILTERED_Z, 'fZ'),
}

# FFT Parameters
SAMPLE_RATE = 250       # Hz (1 kHz / 4 decimation)
FFT_SIZE = 512          # ~2 seconds of data
BATCH_SIZE = 50         # int16 samples per DB frame
GYRO_FFT_SCALE = 10.0   # int16 -> deg/s: divide by this

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


# --- Dark Theme Colors ---
BG_COLOR       = '#1e1e1e'
PANEL_COLOR    = '#252526'
TEXT_COLOR     = '#cccccc'
DIM_TEXT       = '#888888'
GRID_COLOR     = '#3c3c3c'
BTN_COLOR      = '#333333'
BTN_HOVER      = '#444444'
BTN_GREEN      = '#2d5a2d'
BTN_GREEN_HOV  = '#3d7a3d'
BTN_RED        = '#5a2d2d'
BTN_RED_HOV    = '#7a3d3d'


# --- Global State ---
data_queue = queue.Queue(maxsize=5000)
g_serial = None
ring = deque(maxlen=FFT_SIZE)


# --- Log Class Command ---
def send_log_class_command(ser, log_class):
    """Send DB frame to set active log class on the flight controller.
    Sent twice so the wireless bridge delivers enough bytes (>=16) to
    trigger the FC's DMA half-transfer callback."""
    msg_id = DB_CMD_LOG_CLASS
    msg_class = 0x00
    length = 1
    payload = bytes([log_class])
    header = struct.pack('<2sBBH', b'db', msg_id, msg_class, length)
    checksum = (msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF) + log_class) & 0xFFFF
    frame = header + payload + struct.pack('<H', checksum)
    # Send twice: bridge strips padding, so two 9-byte frames = 18 bytes on FC UART
    ser.write(frame)
    ser.write(frame)
    ser.flush()
    print(f"  \u2192 Log class set to 0x{log_class:02X}")


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
    ser.write(frame)
    ser.flush()
    print("  \u2192 Reset command sent")


# --- Serial Reader ---
def serial_reader():
    global SERIAL_PORT, g_serial
    if not SERIAL_PORT:
        return

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            g_serial = ser
            print(f"Connected to {SERIAL_PORT} @ {BAUD_RATE} baud")
            time.sleep(0.2)
            ser.reset_input_buffer()
            ser.write(b'\x00' * 32)
            ser.flush()
            while True:
                # Sync to 'db' header
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

                # Skip checksum (2 bytes consumed on next header scan)

                expected_size = BATCH_SIZE * 2  # 50 x int16 = 100 bytes
                if msg_id == SEND_LOG_ID and length == expected_size:
                    # Unpack 50 int16 values
                    samples = struct.unpack(f'<{BATCH_SIZE}h', payload)
                    data_queue.put(samples)

    except Exception as e:
        print(f"Serial error: {e}")


# --- GUI ---
def main():
    global g_serial

    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()

    # --- Dark Theme ---
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

    fig, (ax_t, ax_f) = plt.subplots(2, 1, figsize=(14, 8))
    fig.patch.set_facecolor(BG_COLOR)
    g_selected_axis = [2]  # default Z
    label = AXIS_MAP[g_selected_axis[0]][1]
    filtered_tag = ' (Filtered)' if g_selected_axis[0] >= 3 else ''
    fig.suptitle(f'Gyro {label} FFT Analysis{filtered_tag}  (Fs={SAMPLE_RATE} Hz, N={FFT_SIZE}, Nyquist={SAMPLE_RATE//2} Hz)',
                 fontsize=13, color=TEXT_COLOR)
    plt.subplots_adjust(bottom=0.15)

    for a in (ax_t, ax_f):
        a.set_facecolor(BG_COLOR)
        a.tick_params(colors=DIM_TEXT)
        for spine in a.spines.values():
            spine.set_color(GRID_COLOR)

    # Time domain plot
    line_t, = ax_t.plot([], [], color='#5599ff', linewidth=0.5)
    ax_t.set_ylabel(f'Gyro {AXIS_MAP[g_selected_axis[0]][1]} (°/s)', color=TEXT_COLOR)
    ax_t.set_xlim(0, FFT_SIZE)
    ax_t.set_ylim(-30, 30)
    ax_t.grid(True, alpha=0.3, color=GRID_COLOR)
    ax_t.set_xlabel('Sample', color=TEXT_COLOR)
    ax_t.set_title('Time Domain', color=TEXT_COLOR)

    # FFT plot
    freq_axis = np.fft.rfftfreq(FFT_SIZE, d=1.0 / SAMPLE_RATE)
    line_f, = ax_f.plot([], [], color='#ff5555', linewidth=1)
    ax_f.set_ylabel('Magnitude (°/s)', color=TEXT_COLOR)
    ax_f.set_xlim(0, SAMPLE_RATE / 2)
    ax_f.set_ylim(0, 5)
    ax_f.grid(True, alpha=0.3, color=GRID_COLOR)
    ax_f.set_xlabel('Frequency (Hz)', color=TEXT_COLOR)
    ax_f.set_title('Frequency Spectrum', color=TEXT_COLOR)

    # Peak frequency annotation
    peak_txt = ax_f.text(0.95, 0.95, '', transform=ax_f.transAxes,
                         ha='right', va='top', fontsize=11, color=TEXT_COLOR,
                         bbox=dict(boxstyle='round', facecolor=PANEL_COLOR, alpha=0.9))

    count_txt = ax_t.text(0.02, 0.95, '', transform=ax_t.transAxes,
                          ha='left', va='top', fontsize=10, color=TEXT_COLOR,
                          bbox=dict(boxstyle='round', facecolor=PANEL_COLOR, alpha=0.9))

    # --- Buttons ---
    # Start/Stop Log toggle
    g_logging_active = False
    g_last_toggle_time = 0
    btn_log_ax = plt.axes([0.05, 0.03, 0.12, 0.05])
    btn_log = Button(btn_log_ax, 'Start Log', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_log.label.set_color(TEXT_COLOR)
    btn_log.label.set_fontsize(8)

    def toggle_log(event):
        nonlocal g_logging_active, g_last_toggle_time
        now = time.time()
        if now - g_last_toggle_time < 0.5:
            return
        g_last_toggle_time = now
        if g_serial and g_serial.is_open:
            if g_logging_active:
                send_log_class_command(g_serial, LOG_CLASS_NONE)
                g_logging_active = False
                btn_log.label.set_text('Start Log')
                btn_log.color = BTN_GREEN
                btn_log.hovercolor = BTN_GREEN_HOV
            else:
                log_cls = AXIS_MAP[g_selected_axis[0]][0]
                send_log_class_command(g_serial, log_cls)
                g_logging_active = True
                btn_log.label.set_text('Stop Log')
                btn_log.color = BTN_RED
                btn_log.hovercolor = BTN_RED_HOV
        else:
            print('  \u2717 Serial not connected')

    btn_log.on_clicked(toggle_log)

    # Axis selector buttons
    axis_btns = []
    axis_btn_objs = []
    for i, (ax_idx, (_, label)) in enumerate(AXIS_MAP.items()):
        bx = plt.axes([0.25 + i * 0.06, 0.03, 0.05, 0.05])
        is_sel = (ax_idx == g_selected_axis[0])
        b = Button(bx, label, color=BTN_GREEN if is_sel else BTN_COLOR,
                   hovercolor=BTN_GREEN_HOV if is_sel else BTN_HOVER)
        b.label.set_color(TEXT_COLOR)
        b.label.set_fontsize(8)
        axis_btns.append(bx)
        axis_btn_objs.append((b, ax_idx))

    def select_axis(idx):
        def handler(event):
            g_selected_axis[0] = idx
            label = AXIS_MAP[idx][1]
            filtered_tag = ' (Filtered)' if idx >= 3 else ''
            fig.suptitle(f'Gyro {label} FFT Analysis{filtered_tag}  (Fs={SAMPLE_RATE} Hz, N={FFT_SIZE}, Nyquist={SAMPLE_RATE//2} Hz)',
                         fontsize=13, color=TEXT_COLOR)
            ax_t.set_ylabel(f'Gyro {label} (°/s)', color=TEXT_COLOR)
            # Update button colors
            for b, ai in axis_btn_objs:
                if ai == idx:
                    b.color = BTN_GREEN
                    b.hovercolor = BTN_GREEN_HOV
                else:
                    b.color = BTN_COLOR
                    b.hovercolor = BTN_HOVER
            # If logging, switch axis on FC
            if g_logging_active and g_serial and g_serial.is_open:
                send_log_class_command(g_serial, AXIS_MAP[idx][0])
            # Clear buffer on axis change
            ring.clear()
        return handler

    for b, ai in axis_btn_objs:
        b.on_clicked(select_axis(ai))

    # Clear button
    btn_clear_ax = plt.axes([0.63, 0.03, 0.10, 0.05])
    btn_clear = Button(btn_clear_ax, 'Clear', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_clear.label.set_color(TEXT_COLOR)
    btn_clear.label.set_fontsize(8)

    def clear_data(event):
        ring.clear()
        while not data_queue.empty():
            try:
                data_queue.get_nowait()
            except queue.Empty:
                break

    btn_clear.on_clicked(clear_data)

    # Reset FC button
    btn_reset_ax = plt.axes([0.75, 0.03, 0.10, 0.05])
    btn_reset = Button(btn_reset_ax, 'Reset FC', color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_reset.label.set_color(TEXT_COLOR)
    btn_reset.label.set_fontsize(8)

    def reset_fc(event):
        nonlocal g_logging_active
        if g_serial and g_serial.is_open:
            send_reset_command(g_serial)
            g_logging_active = False
            btn_log.label.set_text('Start Log')
            btn_log.color = BTN_GREEN
            btn_log.hovercolor = BTN_GREEN_HOV

    btn_reset.on_clicked(reset_fc)

    plt.tight_layout(rect=[0, 0.1, 1, 0.95])

    def update(frame):
        # Drain queue into ring buffer
        count = 0
        while not data_queue.empty() and count < 10:
            try:
                batch = data_queue.get_nowait()
                for raw in batch:
                    ring.append(raw / GYRO_FFT_SCALE)  # int16 -> deg/s
                count += 1
            except queue.Empty:
                break

        n = len(ring)
        count_txt.set_text(f'Samples: {n}/{FFT_SIZE}')

        if n < FFT_SIZE:
            return line_t, line_f, peak_txt, count_txt

        signal = np.array(ring)

        # Time domain
        line_t.set_data(np.arange(FFT_SIZE), signal)
        mx = max(np.max(np.abs(signal)), 5)
        ax_t.set_ylim(-mx * 1.3, mx * 1.3)

        # FFT with Hanning window
        windowed = signal * np.hanning(FFT_SIZE)
        mag = np.abs(np.fft.rfft(windowed)) / FFT_SIZE * 2
        mag[0] /= 2  # DC component

        line_f.set_data(freq_axis, mag)
        max_mag = np.max(mag[1:]) if len(mag) > 1 else 1
        ax_f.set_ylim(0, max(max_mag * 1.3, 0.5))

        # Peak frequency (ignore DC)
        if len(mag) > 1:
            pk = np.argmax(mag[1:]) + 1
            peak_txt.set_text(f'Peak: {freq_axis[pk]:.1f} Hz\nMag: {mag[pk]:.2f} °/s')

        return line_t, line_f, peak_txt, count_txt

    ani = FuncAnimation(fig, update, interval=150, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    print("""
╔═══════════════════════════════════════════════════════════════╗
║              GYRO FFT VISUALIZATION TOOL                      ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  Streams gyro X/Y/Z at 250 Hz via UART (9600 baud)            ║
║  Batched 50 x int16 per DB frame (5 fps, 540 B/s)            ║
║  FFT window: 512 samples (~2 sec), Nyquist: 125 Hz           ║
║                                                               ║
║  USAGE:                                                       ║
║    1. Connect flight controller via USB                       ║
║    2. Select axis (X / Y / Z buttons)                         ║
║    3. Click "Start Log" to begin streaming                    ║
║    4. View time-domain signal + frequency spectrum            ║
║    5. Click "Stop Log" when done                              ║
║                                                               ║
║  IDENTIFIES:                                                  ║
║    ✓ Motor vibration (1x, 2x RPM harmonics)                  ║
║    ✓ Frame/mount resonance                                    ║
║    ✓ Prop imbalance                                           ║
║    ✓ PID oscillation                                          ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
""")
    main()
