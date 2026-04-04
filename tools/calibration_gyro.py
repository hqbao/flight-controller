import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib
matplotlib.use('macosx')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import time
import os
import subprocess
from datetime import datetime

"""
Gyroscope Temperature Compensation Tool

Reads raw gyro data (LOG_CLASS_IMU_GYRO_RAW) from the flight controller,
records bias + temperature to CSV while the drone is stationary, then
loads the CSV to fit a degree-2 polynomial per axis and uploads the
coefficients to the FC for flash storage.

Calibration model:
  bias(T) = a*T² + b*T + c   per axis
  V_cal   = V_raw - bias(T)

Workflow:
  1. Connect board via USB, run this script.
  2. Click "Test Storage" to verify FC communication.
  3. Click "Start Log" — raw gyro data streams at 25 Hz.
  4. Keep drone perfectly still, click "Record".
  5. Let board warm up naturally. Click "Stop & Save" when done.
  6. Click "Load CSV" to fit polynomial from recorded data.
  7. Click "Upload to FC" — sends 9 coefficients to flash.
  8. Click "Verify" to confirm flash storage.
  9. Click "View Calibrated" to see compensated output.
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
SEND_LOG_ID = 0x00

# Log class constants (match messages.h)
LOG_CLASS_NONE           = 0x00
LOG_CLASS_IMU_GYRO_RAW   = 0x0B
LOG_CLASS_IMU_GYRO_CALIB = 0x0C
LOG_CLASS_STORAGE        = 0x10
DB_CMD_LOG_CLASS         = 0x03
DB_CMD_CALIBRATE_GYRO_TEMP = 0x08
DB_CMD_RESET             = 0x07
DB_CMD_CHIP_ID           = 0x09

# CSV polynomial fitting parameters
CAL_WINDOW       = 50     # Samples per averaging window (~2 sec at 25 Hz)
CAL_MIN_POINTS   = 10     # Minimum stationary calibration points
CAL_MIN_RANGE    = 3.0    # Minimum temperature range (°C)
MOTION_THRESHOLD = 32.0   # Max spread in LSB to count as stationary (~2 dps)

# History depth for live plots
HISTORY_LEN = 200

# --- Auto-detect serial port ---
ports = serial.tools.list_ports.comports()
found_port = False
print("Scanning for serial ports...")
for port, desc, hwid in sorted(ports):
    if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB']):
        SERIAL_PORT = port
        found_port = True
        print(f"  \u2713 Auto-selected: {port} ({desc})")
        break
    else:
        print(f"  \u00b7 Skipped: {port} ({desc})")

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
BTN_ORANGE     = '#5a4a2d'
BTN_ORANGE_HOV = '#7a6a3d'
COLOR_X        = '#ff5555'
COLOR_Y        = '#55dd55'
COLOR_Z        = '#5599ff'
COLOR_TEMP     = '#ffaa55'


# --- Global State ---
data_queue = queue.Queue()
verify_queue = queue.Queue()
g_serial = None
g_logging_active = False
g_last_toggle_time = 0
g_received_count = 0
g_view_calibrated = False

# Live data history
gyro_x_hist = []
gyro_y_hist = []
gyro_z_hist = []
temp_hist = []

# Recording
g_recording = False
g_record_data = []
g_record_start_time = 0

# Chip ID (requested at connect)
g_chip_id = None  # hex string e.g. 'A1B2C3D4E5F6'
chip_id_queue = queue.Queue()

# Storage readback (serial reader thread)
g_storage_page0 = None

# Test storage round-trip
g_test_storage_state = 'IDLE'  # IDLE, UPLOADING, VERIFYING
g_test_storage_time = 0
TEST_STORAGE_COEFFS = [0.001001, 0.002002, 0.003003,
                       0.004004, 0.005005, 0.006006,
                       0.007007, 0.008008, 0.009009]

# Calibration result (set by Load CSV)
g_cal_state  = 'IDLE'    # IDLE, DONE, FAILED
g_cal_points = []         # list of (temp, gx_mean, gy_mean, gz_mean)
g_cal_coeffs = None       # np.array (3, 3) — [axis][a, b, c]
g_cal_r2     = [0, 0, 0]  # R² per axis
g_cal_plot_dirty = False   # trigger scatter+fit redraw


# --- DB Frame Helpers ---



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


def send_chip_id_request(ser):
    """Send DB frame to request the 64-bit unique chip ID."""
    msg_id = DB_CMD_CHIP_ID
    msg_class = 0x00
    length = 1
    payload = bytes([0x00])
    header = struct.pack('<2sBBH', b'db', msg_id, msg_class, length)
    checksum = (msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF) + 0x00) & 0xFFFF
    frame = header + payload + struct.pack('<H', checksum)
    ser.write(frame)
    ser.flush()
    print("  \u2192 Chip ID requested")


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


def send_temp_calibration_upload(ser, coeffs):
    """Upload 9 polynomial coefficients (3 axes × [a,b,c]) to FC flash."""
    msg_id = DB_CMD_CALIBRATE_GYRO_TEMP
    msg_class = 0x00
    flat = []
    for axis in range(3):
        flat.extend([coeffs[axis][0], coeffs[axis][1], coeffs[axis][2]])
    payload = struct.pack('<9f', *flat)
    length = len(payload)
    header = struct.pack('<2sBBH', b'db', msg_id, msg_class, length)
    checksum = msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
    for b in payload:
        checksum += b
    checksum &= 0xFFFF
    frame = header + payload + struct.pack('<H', checksum)
    ser.write(frame)
    ser.flush()
    print(f"  \u2192 Temp compensation uploaded (9 coefficients)")


# --- Serial Reader Thread ---
def serial_reader():
    global SERIAL_PORT, g_storage_page0
    if not SERIAL_PORT:
        return

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            global g_serial
            g_serial = ser
            print(f"Connected to {SERIAL_PORT}")
            time.sleep(0.2)
            ser.reset_input_buffer()
            ser.write(b'\x00' * 32)
            ser.flush()

            # Auto-request chip ID at connect
            time.sleep(0.1)
            send_chip_id_request(ser)

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
                ser.read(2)  # consume checksum

                # Chip ID response: 8 bytes (64-bit unique ID)
                if msg_id == SEND_LOG_ID and length == 8:
                    chip_id_hex = payload.hex().upper()
                    chip_id_queue.put(chip_id_hex)

                # Gyro data: 4 floats (16 bytes) — gx, gy, gz, temp
                elif msg_id == SEND_LOG_ID and length == 16:
                    data_queue.put(struct.unpack('<4f', payload))

                # Legacy: 3 floats (12 bytes) — no temperature
                elif msg_id == SEND_LOG_ID and length == 12:
                    data_queue.put(struct.unpack('<3f', payload) + (None,))

                # Storage page 0: 30 floats (120 bytes)
                elif msg_id == SEND_LOG_ID and length == 120:
                    g_storage_page0 = struct.unpack('<30f', payload)

                # Storage page 1: 18 floats (72 bytes)
                elif msg_id == SEND_LOG_ID and length == 72:
                    page1 = struct.unpack('<18f', payload)
                    if g_storage_page0 is not None:
                        verify_queue.put(g_storage_page0 + page1)
                        g_storage_page0 = None

    except Exception as e:
        print(f"Serial error: {e}")


# --- GUI ---
def main():
    global g_logging_active, g_received_count
    global gyro_x_hist, gyro_y_hist, gyro_z_hist, temp_hist
    global g_recording, g_record_data, g_record_start_time
    global g_cal_state, g_cal_points, g_cal_coeffs, g_cal_r2, g_cal_plot_dirty
    global g_test_storage_state, g_test_storage_time

    threading.Thread(target=serial_reader, daemon=True).start()

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

    fig = plt.figure(figsize=(14, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('Gyroscope Temperature Compensation', fontsize=14, color=TEXT_COLOR)
    plt.subplots_adjust(left=0.08, right=0.72, bottom=0.18, top=0.92, hspace=0.35)

    # --- Subplot: Gyro History ---
    ax_gyro = fig.add_subplot(2, 1, 1)
    ax_gyro.set_title('Raw Gyro (LSB)', fontsize=11, color=TEXT_COLOR)
    ax_gyro.set_ylabel('Raw Value (LSB)', fontsize=9)
    ax_gyro.grid(True, alpha=0.3)
    line_gx, = ax_gyro.plot([], [], color=COLOR_X, linewidth=1.5, label='Gyro X')
    line_gy, = ax_gyro.plot([], [], color=COLOR_Y, linewidth=1.5, label='Gyro Y')
    line_gz, = ax_gyro.plot([], [], color=COLOR_Z, linewidth=1.5, label='Gyro Z')
    ax_gyro.legend(loc='upper left', fontsize=8,
                   facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # --- Subplot: Temperature History ---
    ax_temp = fig.add_subplot(2, 1, 2)
    ax_temp.set_title('Die Temperature (\u00b0C)', fontsize=11, color=TEXT_COLOR)
    ax_temp.set_ylabel('Temperature (\u00b0C)', fontsize=9)
    ax_temp.set_xlabel('Sample', fontsize=9)
    ax_temp.grid(True, alpha=0.3)
    line_temp, = ax_temp.plot([], [], color=COLOR_TEMP, linewidth=1.5, label='Temp')
    ax_temp.legend(loc='upper left', fontsize=8,
                   facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # Default Y ranges (user can scroll-zoom)
    ax_gyro.set_ylim(-100, 100)
    ax_temp.set_ylim(0, 100)

    # --- Scroll-to-Zoom ---
    def on_scroll(event):
        if event.inaxes not in (ax_gyro, ax_temp):
            return
        scale = 0.8 if event.button == 'up' else 1.25
        ylo, yhi = event.inaxes.get_ylim()
        yc = event.ydata
        event.inaxes.set_ylim(yc - (yc - ylo) * scale, yc + (yhi - yc) * scale)
    fig.canvas.mpl_connect('scroll_event', on_scroll)

    # --- Info Panel ---
    text_ax = plt.axes([0.74, 0.18, 0.24, 0.74])
    text_ax.axis('off')
    text_ax.set_facecolor(PANEL_COLOR)
    info_text = text_ax.text(0.05, 0.95,
        "GYRO TEMP COMPENSATION\n\n"
        "1. Click 'Test Storage'\n"
        "   to verify FC link\n"
        "2. Click 'Start Log'\n"
        "3. Keep drone still\n"
        "4. Click 'Record'\n"
        "5. Let board warm up\n"
        "6. Click 'Stop & Save'\n"
        "7. Click 'Load CSV'\n"
        "8. Click 'Upload to FC'\n"
        "9. Click 'Verify'\n",
        fontsize=9, va='top', fontfamily='monospace', color=TEXT_COLOR)

    # --- Button Layout (2 rows) ---
    start_x = 0.08
    w = 0.10
    gap = 0.01
    btn_h = 0.04
    row1_y = 0.10
    row2_y = 0.04

    # Row 1: Start Log | Record | Load CSV | Upload to FC
    btn_log_ax = plt.axes([start_x, row1_y, w, btn_h])
    btn_log = Button(btn_log_ax, 'Start Log', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_log.label.set_color(TEXT_COLOR)
    btn_log.label.set_fontsize(8)

    def toggle_log(event):
        global g_logging_active, g_last_toggle_time, g_view_calibrated
        now = time.time()
        if now - g_last_toggle_time < 0.5:
            return
        g_last_toggle_time = now
        if not g_serial or not g_serial.is_open:
            print('  \u2717 Serial not connected')
            return
        if g_logging_active:
            send_log_class_command(g_serial, LOG_CLASS_NONE)
            g_logging_active = False
            btn_log.label.set_text('Start Log')
            btn_log.color = BTN_GREEN
            btn_log.hovercolor = BTN_GREEN_HOV
        else:
            send_log_class_command(g_serial, LOG_CLASS_IMU_GYRO_RAW)
            g_logging_active = True
            btn_log.label.set_text('Stop Log')
            btn_log.color = BTN_RED
            btn_log.hovercolor = BTN_RED_HOV
        g_view_calibrated = False
        ax_gyro.set_title('Raw Gyro (LSB)', fontsize=11, color=TEXT_COLOR)
        ax_gyro.set_ylabel('Raw Value (LSB)', fontsize=9)
        btn_viewcal.label.set_text('View Calibrated')
        btn_viewcal.color = BTN_COLOR
        btn_viewcal.hovercolor = BTN_HOVER
    btn_log.on_clicked(toggle_log)

    btn_record_ax = plt.axes([start_x + (w + gap), row1_y, w, btn_h])
    btn_record = Button(btn_record_ax, 'Record', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_record.label.set_color(TEXT_COLOR)
    btn_record.label.set_fontsize(8)

    g_last_record_time = 0
    def toggle_record(event):
        nonlocal g_last_record_time
        global g_recording, g_record_data, g_record_start_time
        now = time.time()
        if now - g_last_record_time < 0.5:
            return
        g_last_record_time = now
        if g_recording:
            g_recording = False
            btn_record.label.set_text('Record')
            btn_record.color = BTN_GREEN
            btn_record.hovercolor = BTN_GREEN_HOV
            if g_record_data:
                data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
                os.makedirs(data_dir, exist_ok=True)
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                chip_tag = f'_{g_chip_id}' if g_chip_id else ''
                filename = os.path.join(data_dir, f'gyro_temp{chip_tag}_{timestamp}.csv')
                with open(filename, 'w') as f:
                    f.write('time_s,gyro_x_lsb,gyro_y_lsb,gyro_z_lsb,temp_c\n')
                    for row in g_record_data:
                        f.write(f'{row[0]:.3f},{row[1]:.2f},{row[2]:.2f},{row[3]:.2f},{row[4]:.2f}\n')
                duration = g_record_data[-1][0] - g_record_data[0][0]
                print(f'  \u2713 Saved {len(g_record_data)} samples ({duration:.1f}s) to {filename}')
            else:
                print('  \u2717 No data recorded')
        else:
            g_recording = True
            g_record_data = []
            g_record_start_time = time.time()
            btn_record.label.set_text('Stop & Save')
            btn_record.color = BTN_RED
            btn_record.hovercolor = BTN_RED_HOV
            print('  \u23f3 Recording gyro + temperature data...')
    btn_record.on_clicked(toggle_record)

    btn_loadcsv_ax = plt.axes([start_x + 2 * (w + gap), row1_y, w, btn_h])
    btn_loadcsv = Button(btn_loadcsv_ax, 'Load CSV', color=BTN_ORANGE, hovercolor=BTN_ORANGE_HOV)
    btn_loadcsv.label.set_color(TEXT_COLOR)
    btn_loadcsv.label.set_fontsize(8)

    g_last_loadcsv_time = 0
    def load_csv(event):
        nonlocal g_last_loadcsv_time
        global g_cal_state, g_cal_points, g_cal_coeffs, g_cal_r2, g_cal_plot_dirty
        now = time.time()
        if now - g_last_loadcsv_time < 0.5:
            return
        g_last_loadcsv_time = now

        data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
        os.makedirs(data_dir, exist_ok=True)
        result = subprocess.run(
            ['osascript', '-e',
             'set theFile to POSIX path of (choose file of type {"csv"} '
             'with prompt "Load Gyro Temperature Recording" '
             'default location POSIX file "' + data_dir + '")'],
            capture_output=True, text=True
        )
        filepath = result.stdout.strip()
        if not filepath:
            return

        try:
            data = np.genfromtxt(filepath, delimiter=',', skip_header=1)
        except Exception as e:
            print(f'  \u2717 Failed to load CSV: {e}')
            return

        if data.ndim != 2 or data.shape[1] < 5:
            print('  \u2717 Invalid CSV format (need: time,gx,gy,gz,temp)')
            return

        # Windowed averaging with motion rejection
        points = []
        n_samples = len(data)
        for start in range(0, n_samples - CAL_WINDOW + 1, CAL_WINDOW):
            window = data[start:start + CAL_WINDOW]
            gyro_vals = window[:, 1:4]
            spreads = gyro_vals.max(axis=0) - gyro_vals.min(axis=0)
            if spreads.max() <= MOTION_THRESHOLD:
                points.append((
                    window[:, 4].mean(),
                    gyro_vals[:, 0].mean(),
                    gyro_vals[:, 1].mean(),
                    gyro_vals[:, 2].mean(),
                ))

        print(f'  CSV loaded: {n_samples} samples, {len(points)} stationary points')

        if len(points) < CAL_MIN_POINTS:
            g_cal_state = 'FAILED'
            print(f'  \u2717 Only {len(points)} points (need {CAL_MIN_POINTS})')
            return

        pts = np.array(points)
        temps = pts[:, 0]
        temp_range = temps.max() - temps.min()
        if temp_range < CAL_MIN_RANGE:
            g_cal_state = 'FAILED'
            print(f'  \u2717 Range {temp_range:.1f}\u00b0C < {CAL_MIN_RANGE}\u00b0C minimum')
            return

        # Fit degree-2 polynomial per axis: bias(T) = a*T² + b*T + c
        g_cal_coeffs = np.zeros((3, 3))
        g_cal_r2 = [0.0, 0.0, 0.0]
        axis_names = ['X', 'Y', 'Z']
        for i in range(3):
            bias_data = pts[:, i + 1]
            coeffs = np.polyfit(temps, bias_data, 2)
            g_cal_coeffs[i] = coeffs
            predicted = np.polyval(coeffs, temps)
            ss_res = np.sum((bias_data - predicted) ** 2)
            ss_tot = np.sum((bias_data - np.mean(bias_data)) ** 2)
            g_cal_r2[i] = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
            print(f'  {axis_names[i]}: a={coeffs[0]:.6f}, b={coeffs[1]:.4f}, c={coeffs[2]:.2f} (R\u00b2={g_cal_r2[i]:.4f})')

        g_cal_points = points
        g_cal_state = 'DONE'
        g_cal_plot_dirty = True
        print(f'  \u2713 Polynomial fitted: {len(points)} points, range {temp_range:.1f}\u00b0C')
        print(f'  \u2192 Ready to Upload to FC')
    btn_loadcsv.on_clicked(load_csv)

    btn_upload_ax = plt.axes([start_x + 3 * (w + gap), row1_y, w, btn_h])
    btn_upload = Button(btn_upload_ax, 'Upload to FC', color=BTN_ORANGE, hovercolor=BTN_ORANGE_HOV)
    btn_upload.label.set_color(TEXT_COLOR)
    btn_upload.label.set_fontsize(8)

    g_last_upload_time = 0
    def upload_calibration(event):
        nonlocal g_last_upload_time
        now = time.time()
        if now - g_last_upload_time < 0.5:
            return
        g_last_upload_time = now
        if not g_serial or not g_serial.is_open:
            print('  \u2717 Serial not connected')
            return
        if g_cal_state == 'DONE' and g_cal_coeffs is not None:
            send_temp_calibration_upload(g_serial, g_cal_coeffs)
            print('  \u2705 Temp comp uploaded. Click Verify to confirm flash.')
        else:
            print('  \u2717 Load CSV first')
    btn_upload.on_clicked(upload_calibration)

    # Row 2: Verify | View Cal | Test Stor | Chip ID | Clear | Reset FC
    btn_verify_ax = plt.axes([start_x, row2_y, w, btn_h])
    btn_verify = Button(btn_verify_ax, 'Verify', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_verify.label.set_color(TEXT_COLOR)
    btn_verify.label.set_fontsize(8)

    g_last_verify_time = 0
    def verify_flash(event):
        nonlocal g_last_verify_time
        now = time.time()
        if now - g_last_verify_time < 0.5:
            return
        g_last_verify_time = now
        if g_serial and g_serial.is_open:
            global g_storage_page0
            g_storage_page0 = None
            send_log_class_command(g_serial, LOG_CLASS_STORAGE)
            print('  \u23f3 Querying flash storage...')
        else:
            print('  \u2717 Serial not connected')
    btn_verify.on_clicked(verify_flash)

    btn_viewcal_ax = plt.axes([start_x + (w + gap), row2_y, w, btn_h])
    btn_viewcal = Button(btn_viewcal_ax, 'View Cal', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_viewcal.label.set_color(TEXT_COLOR)
    btn_viewcal.label.set_fontsize(8)

    g_last_viewcal_time = 0
    def toggle_view_calibrated(event):
        nonlocal g_last_viewcal_time
        global g_logging_active, g_view_calibrated
        now = time.time()
        if now - g_last_viewcal_time < 0.5:
            return
        g_last_viewcal_time = now
        if not g_serial or not g_serial.is_open:
            print('  \u2717 Serial not connected')
            return
        if g_view_calibrated:
            send_log_class_command(g_serial, LOG_CLASS_IMU_GYRO_RAW)
            g_view_calibrated = False
            g_logging_active = True
            btn_viewcal.label.set_text('View Calibrated')
            btn_viewcal.color = BTN_COLOR
            btn_viewcal.hovercolor = BTN_HOVER
            btn_log.label.set_text('Stop Log')
            btn_log.color = BTN_RED
            btn_log.hovercolor = BTN_RED_HOV
            ax_gyro.set_title('Raw Gyro (LSB)', fontsize=11, color=TEXT_COLOR)
            ax_gyro.set_ylabel('Raw Value (LSB)', fontsize=9)
        else:
            send_log_class_command(g_serial, LOG_CLASS_IMU_GYRO_CALIB)
            g_view_calibrated = True
            g_logging_active = True
            btn_viewcal.label.set_text('View Raw')
            btn_viewcal.color = BTN_RED
            btn_viewcal.hovercolor = BTN_RED_HOV
            btn_log.label.set_text('Stop Log')
            btn_log.color = BTN_RED
            btn_log.hovercolor = BTN_RED_HOV
            ax_gyro.set_title('Calibrated Gyro (\u00b0/s)', fontsize=11, color=TEXT_COLOR)
            ax_gyro.set_ylabel('Angular Rate (\u00b0/s)', fontsize=9)
    btn_viewcal.on_clicked(toggle_view_calibrated)

    btn_teststorage_ax = plt.axes([start_x + 2 * (w + gap), row2_y, w, btn_h])
    btn_teststorage = Button(btn_teststorage_ax, 'Test Stor', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_teststorage.label.set_color(TEXT_COLOR)
    btn_teststorage.label.set_fontsize(8)

    g_last_teststorage_time = 0
    def test_storage(event):
        nonlocal g_last_teststorage_time
        global g_test_storage_state, g_test_storage_time
        now = time.time()
        if now - g_last_teststorage_time < 1.0:
            return
        g_last_teststorage_time = now
        if not g_serial or not g_serial.is_open:
            print('  \u2717 Serial not connected')
            return
        if g_test_storage_state != 'IDLE':
            print('  \u2717 Test already in progress')
            return
        test_coeffs = np.array(TEST_STORAGE_COEFFS).reshape(3, 3)
        send_temp_calibration_upload(g_serial, test_coeffs)
        g_test_storage_state = 'UPLOADING'
        g_test_storage_time = time.time()
        info_text.set_text(
            "STORAGE TEST\n"
            f"{'=' * 22}\n\n"
            "Uploading test data...\n"
            "Waiting for flash write\n"
            "(~2.5 seconds)\n"
        )
        print('  \u23f3 Test storage: uploading test coefficients...')
    btn_teststorage.on_clicked(test_storage)

    btn_chipid_ax = plt.axes([start_x + 3 * (w + gap), row2_y, w, btn_h])
    btn_chipid = Button(btn_chipid_ax, 'Chip ID', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_chipid.label.set_color(TEXT_COLOR)
    btn_chipid.label.set_fontsize(8)

    g_last_chipid_time = 0
    def get_chip_id(event):
        nonlocal g_last_chipid_time
        now = time.time()
        if now - g_last_chipid_time < 0.5:
            return
        g_last_chipid_time = now
        if not g_serial or not g_serial.is_open:
            print('  \u2717 Serial not connected')
            return
        if g_chip_id:
            info_text.set_text(
                f"CHIP ID \u2713\n"
                f"{'=' * 26}\n\n"
                f"{g_chip_id}\n\n"
                f"ID stored for CSV naming."
            )
            plt.draw()
        else:
            send_chip_id_request(g_serial)
            info_text.set_text(
                f"CHIP ID\n"
                f"{'=' * 26}\n\n"
                f"Requesting..."
            )
            plt.draw()
    btn_chipid.on_clicked(get_chip_id)

    btn_clear_ax = plt.axes([start_x + 4 * (w + gap), row2_y, w, btn_h])
    btn_clear = Button(btn_clear_ax, 'Clear', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_clear.label.set_color(TEXT_COLOR)
    btn_clear.label.set_fontsize(8)

    def clear_history(event):
        global gyro_x_hist, gyro_y_hist, gyro_z_hist, temp_hist, g_received_count
        gyro_x_hist = []
        gyro_y_hist = []
        gyro_z_hist = []
        temp_hist = []
        g_received_count = 0
        ax_gyro.set_ylim(-100, 100)
        ax_temp.set_ylim(0, 100)
        print("  History cleared")
    btn_clear.on_clicked(clear_history)

    btn_resetfc_ax = plt.axes([start_x + 5 * (w + gap), row2_y, w, btn_h])
    btn_resetfc = Button(btn_resetfc_ax, 'Reset FC', color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_resetfc.label.set_color(TEXT_COLOR)
    btn_resetfc.label.set_fontsize(8)

    def reset_fc(event):
        global g_logging_active
        if g_serial and g_serial.is_open:
            send_reset_command(g_serial)
            g_logging_active = False
            btn_log.label.set_text('Start Log')
            btn_log.color = BTN_GREEN
            btn_log.hovercolor = BTN_GREEN_HOV
    btn_resetfc.on_clicked(reset_fc)

    # --- Animation Loop ---
    cur_temp = 0.0
    g_cal_view_active = False
    chip_id_request_time = time.time()  # track auto-request for retry

    while True:
        # Drain chip ID queue (silently store)
        chip_id_updated = False
        while not chip_id_queue.empty():
            global g_chip_id
            g_chip_id = chip_id_queue.get()
            chip_id_updated = True
            print(f'  \u2713 Chip ID: {g_chip_id}')

        # Retry chip ID request if no response after 2 seconds
        if g_chip_id is None and g_serial and g_serial.is_open:
            now = time.time()
            if now - chip_id_request_time > 2.0:
                chip_id_request_time = now
                send_chip_id_request(g_serial)

        # Drain data queue
        data_updated = False
        while not data_queue.empty():
            values = data_queue.get()
            g_received_count += 1
            data_updated = True

            gx, gy, gz, temp = values
            gyro_x_hist.append(gx)
            gyro_y_hist.append(gy)
            gyro_z_hist.append(gz)
            if temp is not None:
                temp_hist.append(temp)

            # Trim history
            if len(gyro_x_hist) > HISTORY_LEN:
                gyro_x_hist = gyro_x_hist[-HISTORY_LEN:]
                gyro_y_hist = gyro_y_hist[-HISTORY_LEN:]
                gyro_z_hist = gyro_z_hist[-HISTORY_LEN:]
            if len(temp_hist) > HISTORY_LEN:
                temp_hist = temp_hist[-HISTORY_LEN:]

            # Record if active
            if g_recording and temp is not None:
                elapsed = time.time() - g_record_start_time
                g_record_data.append((elapsed, gx, gy, gz, temp))

        # Update live plots
        if data_updated:
            # Restore time-series view if scatter plot was showing
            if g_cal_view_active:
                g_cal_view_active = False
                ax_gyro.cla()
                if g_view_calibrated:
                    ax_gyro.set_title('Calibrated Gyro (\u00b0/s)', fontsize=11, color=TEXT_COLOR)
                    ax_gyro.set_ylabel('Angular Rate (\u00b0/s)', fontsize=9)
                else:
                    ax_gyro.set_title('Raw Gyro (LSB)', fontsize=11, color=TEXT_COLOR)
                    ax_gyro.set_ylabel('Raw Value (LSB)', fontsize=9)
                ax_gyro.grid(True, alpha=0.3)
                line_gx, = ax_gyro.plot([], [], color=COLOR_X, linewidth=1.5, label='Gyro X')
                line_gy, = ax_gyro.plot([], [], color=COLOR_Y, linewidth=1.5, label='Gyro Y')
                line_gz, = ax_gyro.plot([], [], color=COLOR_Z, linewidth=1.5, label='Gyro Z')
                ax_gyro.legend(loc='upper left', fontsize=8,
                               facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)
                ax_temp.cla()
                ax_temp.set_title('Die Temperature (\u00b0C)', fontsize=11, color=TEXT_COLOR)
                ax_temp.set_ylabel('Temperature (\u00b0C)', fontsize=9)
                ax_temp.set_xlabel('Sample', fontsize=9)
                ax_temp.grid(True, alpha=0.3)
                line_temp, = ax_temp.plot([], [], color=COLOR_TEMP, linewidth=1.5, label='Temp')
                ax_temp.legend(loc='upper left', fontsize=8,
                               facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

            n = len(gyro_x_hist)
            xs = list(range(n))
            line_gx.set_data(xs, gyro_x_hist)
            line_gy.set_data(xs, gyro_y_hist)
            line_gz.set_data(xs, gyro_z_hist)
            ax_gyro.set_xlim(0, max(n, 10))

            nt = len(temp_hist)
            if nt > 0:
                line_temp.set_data(list(range(nt)), temp_hist)
                ax_temp.set_xlim(0, max(nt, 10))

            cur_temp = temp_hist[-1] if temp_hist else 0.0

        # Info panel (refresh on data only — chip ID stored silently)
        if data_updated:
            rec_str = ''
            if g_recording:
                rec_dur = time.time() - g_record_start_time
                rec_str = f"\nREC \u25cf {len(g_record_data)} ({rec_dur:.0f}s)\n"

            cal_str = ''
            if g_cal_state == 'DONE' and g_cal_coeffs is not None:
                cal_str = (
                    f"\n{'=' * 22}\n"
                    f"TEMP COMP (deg 2)\n"
                    f"{'=' * 22}\n"
                    f"Points: {len(g_cal_points)}\n"
                    f"R\u00b2: {g_cal_r2[0]:.3f} "
                    f"{g_cal_r2[1]:.3f} "
                    f"{g_cal_r2[2]:.3f}\n"
                )
            elif g_cal_state == 'FAILED':
                cal_str = f"\n{'=' * 22}\nCALIBRATION FAILED\n"

            chip_str = f"Chip: {g_chip_id}\n" if g_chip_id else ''

            info_text.set_text(
                f"GYRO TEMP COMPENSATION\n"
                f"{'=' * 22}\n\n"
                f"{chip_str}"
                f"Temp:  {cur_temp:.1f} \u00b0C\n"
                f"Frames: {g_received_count}\n"
                f"{rec_str}"
                f"{cal_str}"
            )

        # Show scatter + fitted curves after Load CSV
        if g_cal_plot_dirty and g_cal_state == 'DONE' and g_cal_coeffs is not None:
            g_cal_plot_dirty = False
            g_cal_view_active = True
            pts = np.array(g_cal_points)
            temps = pts[:, 0]

            ax_gyro.cla()
            ax_gyro.set_title('Bias vs Temperature (fitted)', fontsize=11, color=TEXT_COLOR)
            ax_gyro.set_ylabel('Bias (LSB)', fontsize=9)
            ax_gyro.set_xlabel('Temperature (\u00b0C)', fontsize=9)
            ax_gyro.grid(True, alpha=0.3)
            ax_gyro.scatter(temps, pts[:, 1], color=COLOR_X, s=8, alpha=0.5)
            ax_gyro.scatter(temps, pts[:, 2], color=COLOR_Y, s=8, alpha=0.5)
            ax_gyro.scatter(temps, pts[:, 3], color=COLOR_Z, s=8, alpha=0.5)
            t_fit = np.linspace(temps.min(), temps.max(), 200)
            for i, (color, label) in enumerate([(COLOR_X, 'X'), (COLOR_Y, 'Y'), (COLOR_Z, 'Z')]):
                y_fit = np.polyval(g_cal_coeffs[i], t_fit)
                ax_gyro.plot(t_fit, y_fit, color=color, linewidth=2,
                             label=f'{label} (R\u00b2={g_cal_r2[i]:.3f})')
            ax_gyro.legend(loc='upper left', fontsize=8,
                           facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

            ax_temp.cla()
            ax_temp.set_title('Sample Temperature Distribution', fontsize=11, color=TEXT_COLOR)
            ax_temp.set_xlabel('Temperature (\u00b0C)', fontsize=9)
            ax_temp.set_ylabel('Count', fontsize=9)
            ax_temp.grid(True, alpha=0.3)
            ax_temp.hist(temps, bins=30, color=COLOR_TEMP, alpha=0.7, edgecolor=GRID_COLOR)

            plt.draw()

        # Test storage: trigger readback after flash flush delay
        if g_test_storage_state == 'UPLOADING' and time.time() - g_test_storage_time > 2.5:
            g_test_storage_state = 'VERIFYING'
            g_test_storage_time = time.time()
            if g_serial and g_serial.is_open:
                send_log_class_command(g_serial, LOG_CLASS_STORAGE)
                print('  \u23f3 Test storage: reading back flash...')

        # Timeout for readback (5s)
        if g_test_storage_state == 'VERIFYING' and time.time() - g_test_storage_time > 5.0:
            g_test_storage_state = 'IDLE'
            print('  \u2717 Storage readback timed out (no response from FC)')

        # Handle flash verification data
        while not verify_queue.empty():
            params = verify_queue.get()
            if g_serial and g_serial.is_open:
                send_log_class_command(g_serial, LOG_CLASS_NONE)
            g_logging_active = False
            btn_log.label.set_text('Start Log')
            btn_log.color = BTN_GREEN
            btn_log.hovercolor = BTN_GREEN_HOV

            if g_test_storage_state == 'VERIFYING':
                g_test_storage_state = 'IDLE'
                has_temp = params[30] if len(params) > 30 else 0.0
                ok = abs(has_temp - 1.0) < 0.01
                if ok:
                    for i in range(9):
                        got = params[31 + i] if len(params) > 31 + i else 0.0
                        if abs(got - TEST_STORAGE_COEFFS[i]) > 0.0001:
                            ok = False
                            break
                if ok:
                    info_text.set_text(
                        f"STORAGE TEST PASSED \u2713\n"
                        f"{'=' * 22}\n\n"
                        f"Upload  \u2713\n"
                        f"Flash   \u2713\n"
                        f"Readback \u2713\n\n"
                        f"All 9 coefficients\n"
                        f"verified correctly.\n\n"
                        f"{'=' * 22}\n"
                        f"Ready to calibrate.\n"
                    )
                    print('  \u2713 Storage test PASSED')
                else:
                    info_text.set_text(
                        f"STORAGE TEST FAILED \u2717\n"
                        f"{'=' * 22}\n\n"
                        f"Readback mismatch.\n"
                        f"Flag: {has_temp:.1f}\n\n"
                        f"Check serial link\n"
                        f"and try again.\n"
                    )
                    print(f'  \u2717 Storage test FAILED (flag={has_temp:.1f})')
            else:
                has_temp = params[30] if len(params) > 30 else 0.0
                if has_temp > 0.0:
                    coeffs_str = ''
                    axes = ['X', 'Y', 'Z']
                    for i, ax in enumerate(axes):
                        a = params[31 + i * 3] if len(params) > 31 + i * 3 else 0.0
                        b = params[32 + i * 3] if len(params) > 32 + i * 3 else 0.0
                        c = params[33 + i * 3] if len(params) > 33 + i * 3 else 0.0
                        coeffs_str += f"  {ax}: {a:+.4f} {b:+.4f} {c:+.2f}\n"
                    info_text.set_text(
                        f"FLASH VERIFIED \u2713\n"
                        f"{'=' * 22}\n\n"
                        f"Temp Comp: YES\n\n"
                        f"Coefficients (a b c):\n"
                        f"{coeffs_str}\n"
                        f"bias(T) = aT\u00b2+bT+c\n\n"
                        f"{'=' * 22}\n"
                        f"Saved to flash.\n"
                        f"Active immediately."
                    )
                else:
                    info_text.set_text(
                        f"FLASH VERIFY FAILED \u2717\n"
                        f"{'=' * 22}\n\n"
                        f"Temp comp not found\n"
                        f"in flash storage.\n\n"
                        f"Try uploading again."
                    )
            plt.draw()

        plt.pause(0.05)
        if not plt.fignum_exists(fig.number):
            break


if __name__ == "__main__":
    main()
