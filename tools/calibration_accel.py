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
from matplotlib.animation import FuncAnimation
import time
import os
from datetime import datetime

"""
IMU Accelerometer Calibration Tool (6-Position Ellipsoid Fit)

Computes Bias (B) and Scale (S) matrix for accelerometer calibration
using axis-aligned ellipsoid fitting.  Temperature compensation is NOT
needed for accel -- vibration noise during flight is orders of magnitude
larger than the ICM-42688P thermal drift (~0.15 mg/C).

Calibration model:
    V_cal = S * (V_raw - B)
where B = ellipsoid center (3 floats, raw LSB) and S = diagonal 3x3
scale matrix that maps the ellipsoid to a sphere.  Output preserves
raw LSB magnitude (~16384 LSB/g at +/-2g).

Workflow:
    1. Connect FC via USB, click 'Start Log' -- yellow arrow = live reading.
    2. Place drone flat (level), click 'Capture' -- averages 100 samples.
    3. Rotate to next orientation: Left, Right, Nose Up, Nose Down, Inverted.
    4. Repeat 'Capture' for each (minimum 6 positions).
    5. Click 'Compute' -- fits ellipsoid, shows red (raw) + green (corrected).
    6. Click 'Upload' -- sends bias + scale to FC, saved to flash.
    7. Click 'Verify' -- reads back flash to confirm.
    8. Click 'View Cal' -- switches to calibrated accel stream for validation.

CSV persistence:
    - 'Save CSV' saves captured positions to tools/data/ (gitignored).
    - 'Load CSV' reloads positions for re-computation or cross-board comparison.
    - Chip ID is auto-requested and embedded in the CSV filename.
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
SEND_LOG_ID = 0x00

# Log class constants (match messages.h)
LOG_CLASS_NONE            = 0x00
LOG_CLASS_IMU_ACCEL_RAW   = 0x01
LOG_CLASS_IMU_ACCEL_CALIB = 0x0A
LOG_CLASS_STORAGE         = 0x10
DB_CMD_LOG_CLASS          = 0x03
DB_CMD_CALIBRATE_ACCEL    = 0x05
DB_CMD_RESET              = 0x07
DB_CMD_CHIP_ID            = 0x09
PLOT_LIMIT = 20000  # Initial axis limit (scroll to zoom)
SAMPLES_PER_POSITION = 100  # Samples averaged per captured position


# Auto-detect serial port
ports = serial.tools.list_ports.comports()
found_port = False
print("Scanning for ports...")
for port, desc, hwid in sorted(ports):
    if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB']):
        SERIAL_PORT = port
        found_port = True
        print(f"Auto-selected Port: {port} ({desc})")
        break

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


# --- Global State ---
data_queue = queue.Queue()
verify_queue = queue.Queue()
chip_id_queue = queue.Queue()
raw_data_points = []              # List of averaged points (one per position)
current_position_samples = []     # Temporary samples for the current position
is_capturing = False
g_serial = None
g_received_count = 0
g_last_raw = None
g_chip_id = None                  # hex string e.g. 'A1B2C3D4E5F67890'
calibration_result = (np.zeros(3), np.eye(3))

# Test storage round-trip
g_test_storage_state = 'IDLE'  # IDLE, UPLOADING, VERIFYING
g_test_storage_time = 0
TEST_STORAGE_BIAS  = [11.11, 22.22, 33.33]
TEST_STORAGE_SCALE = [[1.01, 0.02, 0.03],
                      [0.04, 1.05, 0.06],
                      [0.07, 0.08, 1.09]]

# --- 6 standard orientations for on-screen guidance ---
ORIENTATIONS = [
    "Flat (level, Z up)",
    "Inverted (Z down)",
    "Left side (Y up)",
    "Right side (Y down)",
    "Nose up (X up)",
    "Nose down (X down)",
]


# --- DB Protocol Helpers ---

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


def send_calibration_upload(ser, bias, scale):
    """Send accel calibration data (bias + scale matrix) to the flight controller.

    Payload: 12 float32 values = 48 bytes
      [bias_x, bias_y, bias_z, s00, s01, s02, s10, s11, s12, s20, s21, s22]
    """
    msg_id = DB_CMD_CALIBRATE_ACCEL
    msg_class = 0x00

    values = list(bias) + [scale[r][c] for r in range(3) for c in range(3)]
    payload = struct.pack('<12f', *values)
    length = len(payload)  # 48

    header = struct.pack('<2sBBH', b'db', msg_id, msg_class, length)

    checksum = msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
    for b in payload:
        checksum += b
    checksum &= 0xFFFF

    frame = header + payload + struct.pack('<H', checksum)

    ser.write(frame)
    ser.flush()
    print(f"  \u2192 Accel calibration uploaded ({len(values)} floats, {len(frame)} bytes)")


# --- Serial Reader ---

def serial_reader():
    global SERIAL_PORT
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

            # Auto-request chip ID on connect
            time.sleep(0.3)
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
                ser.read(2)  # consume checksum

                if msg_id == SEND_LOG_ID:
                    # Accel data: 4 floats (ax, ay, az, temp) = 16 bytes
                    if length == 16:
                        x, y, z, temp = struct.unpack('<4f', payload)
                        if abs(x) <= 40000 and abs(y) <= 40000 and abs(z) <= 40000:
                            data_queue.put((x, y, z))

                    # Chip ID response: 8 bytes (64-bit unique ID)
                    elif length == 8 and any(b != 0 for b in payload):
                        chip_id_hex = payload.hex().upper()
                        chip_id_queue.put(chip_id_hex)

                    # LOG_CLASS_STORAGE: 30 floats (120 bytes)
                    elif length == 120:
                        values = struct.unpack('<30f', payload)
                        verify_queue.put(values)

    except Exception as e:
        print(f"Serial error: {e}")


# --- Calibration Algorithm ---

def calibrate_accelerometer(data):
    """
    Axis-aligned ellipsoid fit for accelerometer calibration.

    Fits: A*x^2 + B*y^2 + C*z^2 + G*x + H*y + I*z = 1
    (6 parameters, no cross-coupling terms)

    This is well-conditioned with 6+ capture positions (the standard
    6-position procedure: flat, inverted, left, right, nose-up, nose-down).

    Returns:
        center: Bias vector (3,) in raw LSB
        S:      Diagonal scale matrix (3,3), preserves raw LSB magnitude
    """
    if len(data) < 6:
        return None

    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]

    D = np.array([x**2, y**2, z**2, x, y, z]).T
    ones = np.ones((len(data), 1))

    try:
        v, _, _, _ = np.linalg.lstsq(D, ones, rcond=None)
    except Exception:
        return None

    if v is None or np.isnan(v).any():
        return None

    v = v.flatten()

    A, B, C = v[0], v[1], v[2]
    G, H, I_coeff = v[3], v[4], v[5]

    if A <= 0 or B <= 0 or C <= 0:
        return None

    center = np.array([-G / (2 * A), -H / (2 * B), -I_coeff / (2 * C)])

    scale_const = 1 + G**2 / (4 * A) + H**2 / (4 * B) + I_coeff**2 / (4 * C)
    if scale_const <= 0:
        return None

    S = np.diag([
        np.sqrt(A / scale_const),
        np.sqrt(B / scale_const),
        np.sqrt(C / scale_const),
    ])

    # Scale S so calibrated output preserves raw LSB magnitude
    raw_centered = data - center
    avg_radius = np.mean(np.linalg.norm(raw_centered, axis=1))
    S = S * avg_radius

    return center, S


# --- CSV Persistence ---

def save_positions_csv(positions, chip_id):
    """Save captured position averages to CSV in tools/data/."""
    if not positions:
        print("  \u2717 No positions to save")
        return None

    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
    os.makedirs(data_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    chip_tag = f'_{chip_id}' if chip_id else ''
    filename = os.path.join(data_dir, f'accel_cal{chip_tag}_{timestamp}.csv')

    with open(filename, 'w') as f:
        f.write('position,accel_x_lsb,accel_y_lsb,accel_z_lsb\n')
        for i, pt in enumerate(positions):
            f.write(f'{i + 1},{pt[0]:.2f},{pt[1]:.2f},{pt[2]:.2f}\n')

    print(f'  \u2713 Saved {len(positions)} positions to {filename}')
    return filename


def load_positions_csv(filepath):
    """Load captured position averages from CSV. Returns list of (x,y,z) tuples."""
    positions = []
    try:
        with open(filepath, 'r') as f:
            header = f.readline()  # skip header
            for line in f:
                parts = line.strip().split(',')
                if len(parts) >= 4:
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    positions.append((x, y, z))
        print(f'  \u2713 Loaded {len(positions)} positions from {filepath}')
    except Exception as e:
        print(f'  \u2717 Failed to load CSV: {e}')
    return positions


# --- GUI ---

def main():
    global is_capturing, raw_data_points, current_position_samples, calibration_result
    global g_received_count, g_last_raw
    global g_test_storage_state, g_test_storage_time

    show_raw = True
    show_calib = True

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

    fig = plt.figure(figsize=(14, 8))
    fig.patch.set_facecolor(BG_COLOR)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor(BG_COLOR)
    ax.set_box_aspect((1, 1, 1))
    plt.subplots_adjust(bottom=0.20, right=0.75)
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.pane.fill = False
        axis.pane.set_edgecolor(GRID_COLOR)
        axis.label.set_color(TEXT_COLOR)
        axis._axinfo['tick']['color'] = TEXT_COLOR
        axis._axinfo['tick']['inward_factor'] = 0
    ax.tick_params(colors=DIM_TEXT)

    plot_raw, = ax.plot([], [], [], 'ro', markersize=10, label='Captured Positions')
    plot_corr, = ax.plot([], [], [], 'go', markersize=10, label='Corrected Positions')
    live_arrow = [None]
    plot_live_tip, = ax.plot([], [], [], 'yD', markersize=8, label='Live Reading')

    # --- Info Panel ---
    text_ax = plt.axes([0.76, 0.15, 0.23, 0.75])
    text_ax.axis('off')
    text_ax.set_facecolor(PANEL_COLOR)
    instructions = (
        "ACCEL 6-POSITION CALIBRATION\n"
        f"{'=' * 30}\n\n"
        "STEP 1: Click 'Start Log'\n"
        "  Yellow arrow = live reading\n\n"
        "STEP 2: Place drone in each\n"
        "  orientation, click 'Capture':\n"
        "  1) Flat (level, Z up)\n"
        "  2) Inverted (Z down)\n"
        "  3) Left side (Y up)\n"
        "  4) Right side (Y down)\n"
        "  5) Nose up (X up)\n"
        "  6) Nose down (X down)\n\n"
        "STEP 3: Click 'Compute'\n"
        "  Red=raw, Green=corrected\n\n"
        "STEP 4: Click 'Upload'\n"
        "  Saves to FC flash\n\n"
        "STEP 5: Click 'Verify'\n"
        "  Reads back flash data\n\n"
        "STEP 6: Click 'View Cal'\n"
        "  Check magnitude ~16384\n"
        "  in all orientations"
    )
    info_text = text_ax.text(0.05, 0.98, instructions, fontsize=8, va='top',
                             fontfamily='monospace', color=TEXT_COLOR)

    # --- Button Layout (uniform: w=0.10, h=0.04, fontsize=8, gap=0.01) ---
    start_x = 0.02
    w = 0.10
    gap = 0.01
    btn_h = 0.04

    # Row 1 (top): View buttons (Top, Bottom, Front, Back, Left, Right)
    row1_y = 0.14
    views = {
        'Top': (90, -90), 'Bottom': (-90, -90),
        'Front': (0, 90), 'Back': (0, -90),
        'Left': (0, 180), 'Right': (0, 0)
    }
    view_btns = []
    for i, (label, (elev, azim)) in enumerate(views.items()):
        b_ax = plt.axes([start_x + i * (w + gap), row1_y, w, btn_h])
        b = Button(b_ax, label, color=BTN_COLOR, hovercolor=BTN_HOVER)
        b.label.set_color(TEXT_COLOR)
        b.label.set_fontsize(8)
        b.on_clicked(lambda event, e=elev, a=azim: (ax.view_init(elev=e, azim=a), plt.draw()))
        view_btns.append(b)

    # Row 2 (middle): Start Log | View Cal | Hide Raw | Hide Corr | Test Stor | Chip ID | Reset FC
    row2_y = 0.09

    g_logging_active = False
    g_verify_active = False
    g_last_toggle_time = 0

    btn_log_ax = plt.axes([start_x, row2_y, w, btn_h])
    btn_log = Button(btn_log_ax, 'Start Log', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_log.label.set_color(TEXT_COLOR)
    btn_log.label.set_fontsize(8)

    btn_viewcal_ax = plt.axes([start_x + (w + gap), row2_y, w, btn_h])
    btn_viewcal = Button(btn_viewcal_ax, 'View Cal', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_viewcal.label.set_color(TEXT_COLOR)
    btn_viewcal.label.set_fontsize(8)

    btn_raw_ax = plt.axes([start_x + 2 * (w + gap), row2_y, w, btn_h])
    btn_raw = Button(btn_raw_ax, 'Hide Raw', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_raw.label.set_color(TEXT_COLOR)
    btn_raw.label.set_fontsize(8)

    btn_calib_ax = plt.axes([start_x + 3 * (w + gap), row2_y, w, btn_h])
    btn_calib = Button(btn_calib_ax, 'Hide Corr', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_calib.label.set_color(TEXT_COLOR)
    btn_calib.label.set_fontsize(8)

    btn_teststorage_ax = plt.axes([start_x + 4 * (w + gap), row2_y, w, btn_h])
    btn_teststorage = Button(btn_teststorage_ax, 'Test Stor', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_teststorage.label.set_color(TEXT_COLOR)
    btn_teststorage.label.set_fontsize(8)

    btn_chipid_ax = plt.axes([start_x + 5 * (w + gap), row2_y, w, btn_h])
    btn_chipid = Button(btn_chipid_ax, 'Chip ID', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_chipid.label.set_color(TEXT_COLOR)
    btn_chipid.label.set_fontsize(8)

    btn_resetfc_ax = plt.axes([start_x + 6 * (w + gap), row2_y, w, btn_h])
    btn_resetfc = Button(btn_resetfc_ax, 'Reset FC', color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_resetfc.label.set_color(TEXT_COLOR)
    btn_resetfc.label.set_fontsize(8)

    # Row 3 (bottom): Capture | Compute | Upload | Verify | Save CSV | Load CSV | Clear
    row3_y = 0.04

    btn_capture_ax = plt.axes([start_x, row3_y, w, btn_h])
    btn_capture = Button(btn_capture_ax, 'Capture', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_capture.label.set_color(TEXT_COLOR)
    btn_capture.label.set_fontsize(8)

    btn_calc_ax = plt.axes([start_x + (w + gap), row3_y, w, btn_h])
    btn_calc = Button(btn_calc_ax, 'Compute', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_calc.label.set_color(TEXT_COLOR)
    btn_calc.label.set_fontsize(8)

    btn_upload_ax = plt.axes([start_x + 2 * (w + gap), row3_y, w, btn_h])
    btn_upload = Button(btn_upload_ax, 'Upload', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_upload.label.set_color(TEXT_COLOR)
    btn_upload.label.set_fontsize(8)

    btn_vflash_ax = plt.axes([start_x + 3 * (w + gap), row3_y, w, btn_h])
    btn_vflash = Button(btn_vflash_ax, 'Verify', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_vflash.label.set_color(TEXT_COLOR)
    btn_vflash.label.set_fontsize(8)

    btn_savecsv_ax = plt.axes([start_x + 4 * (w + gap), row3_y, w, btn_h])
    btn_savecsv = Button(btn_savecsv_ax, 'Save CSV', color=BTN_ORANGE, hovercolor=BTN_ORANGE_HOV)
    btn_savecsv.label.set_color(TEXT_COLOR)
    btn_savecsv.label.set_fontsize(8)

    btn_loadcsv_ax = plt.axes([start_x + 5 * (w + gap), row3_y, w, btn_h])
    btn_loadcsv = Button(btn_loadcsv_ax, 'Load CSV', color=BTN_ORANGE, hovercolor=BTN_ORANGE_HOV)
    btn_loadcsv.label.set_color(TEXT_COLOR)
    btn_loadcsv.label.set_fontsize(8)

    btn_clear_ax = plt.axes([start_x + 6 * (w + gap), row3_y, w, btn_h])
    btn_clear = Button(btn_clear_ax, 'Clear', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_clear.label.set_color(TEXT_COLOR)
    btn_clear.label.set_fontsize(8)

    # --- Button Callbacks ---

    def toggle_log(event):
        nonlocal g_logging_active, g_verify_active, g_last_toggle_time
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
                send_log_class_command(g_serial, LOG_CLASS_IMU_ACCEL_RAW)
                g_logging_active = True
                g_verify_active = False
                btn_log.label.set_text('Stop Log')
                btn_log.color = BTN_RED
                btn_log.hovercolor = BTN_RED_HOV
                btn_viewcal.label.set_text('View Cal')
                btn_viewcal.color = BTN_COLOR
                btn_viewcal.hovercolor = BTN_HOVER
        else:
            print('  \u2717 Serial not connected')
    btn_log.on_clicked(toggle_log)

    def toggle_view_calibrated(event):
        nonlocal g_logging_active, g_verify_active, g_last_toggle_time
        now = time.time()
        if now - g_last_toggle_time < 0.5:
            return
        g_last_toggle_time = now
        if g_serial and g_serial.is_open:
            if g_verify_active:
                send_log_class_command(g_serial, LOG_CLASS_NONE)
                g_verify_active = False
                btn_viewcal.label.set_text('View Cal')
                btn_viewcal.color = BTN_COLOR
                btn_viewcal.hovercolor = BTN_HOVER
            else:
                send_log_class_command(g_serial, LOG_CLASS_IMU_ACCEL_CALIB)
                g_verify_active = True
                g_logging_active = False
                btn_viewcal.label.set_text('View Raw')
                btn_viewcal.color = BTN_RED
                btn_viewcal.hovercolor = BTN_RED_HOV
                btn_log.label.set_text('Start Log')
                btn_log.color = BTN_GREEN
                btn_log.hovercolor = BTN_GREEN_HOV
        else:
            print('  \u2717 Serial not connected')
    btn_viewcal.on_clicked(toggle_view_calibrated)

    def toggle_raw(event):
        nonlocal show_raw
        show_raw = not show_raw
        plot_raw.set_visible(show_raw)
        btn_raw.label.set_text('Hide Raw' if show_raw else 'Show Raw')
        plt.draw()
    btn_raw.on_clicked(toggle_raw)

    def toggle_calib(event):
        nonlocal show_calib
        show_calib = not show_calib
        plot_corr.set_visible(show_calib)
        btn_calib.label.set_text('Hide Corr' if show_calib else 'Show Corr')
        plt.draw()
    btn_calib.on_clicked(toggle_calib)

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

    def reset_fc(event):
        nonlocal g_logging_active, g_verify_active
        if g_serial and g_serial.is_open:
            send_reset_command(g_serial)
            g_logging_active = False
            g_verify_active = False
            btn_log.label.set_text('Start Log')
            btn_log.color = BTN_GREEN
            btn_log.hovercolor = BTN_GREEN_HOV
            btn_viewcal.label.set_text('View Cal')
            btn_viewcal.color = BTN_COLOR
            btn_viewcal.hovercolor = BTN_HOVER
    btn_resetfc.on_clicked(reset_fc)

    def capture_position(event):
        global is_capturing, current_position_samples
        if not is_capturing:
            is_capturing = True
            current_position_samples = []
            btn_capture.label.set_text('Wait...')
            n = len(raw_data_points) + 1
            orient = ORIENTATIONS[n - 1] if n <= len(ORIENTATIONS) else f"Position {n}"
            print(f"  \u23f3 Capturing {orient}...")
    btn_capture.on_clicked(capture_position)

    def compute_calibration(event):
        global calibration_result
        if len(raw_data_points) < 6:
            info_text.set_text(
                f"NEED MORE POSITIONS\n"
                f"{'=' * 30}\n\n"
                f"Have: {len(raw_data_points)} positions\n"
                f"Need: at least 6\n\n"
                f"Keep capturing..."
            )
            print("  \u2717 Need at least 6 positions.")
            plt.draw()
            return

        data_np = np.array(raw_data_points)
        res = calibrate_accelerometer(data_np)
        if res is not None:
            B, S = res
            calibration_result = (B, S)
            print("  \u2713 Calibration computed")

            B_str = f"[{B[0]:.1f}, {B[1]:.1f}, {B[2]:.1f}]"
            S_diag = f"[{S[0][0]:.4f}, {S[1][1]:.4f}, {S[2][2]:.4f}]"
            info_text.set_text(
                f"CALIBRATION COMPUTED \u2713\n"
                f"{'=' * 30}\n\n"
                f"Positions: {len(raw_data_points)}\n\n"
                f"Bias B (LSB):\n{B_str}\n\n"
                f"Scale diag:\n{S_diag}\n\n"
                f"Red = raw captured\n"
                f"Green = corrected\n\n"
                f"Click 'Upload' to send\n"
                f"to flight controller."
            )

            raw_centered = (data_np - B).T
            corrected = np.dot(S, raw_centered).T
            plot_corr.set_data(corrected[:, 0], corrected[:, 1])
            plot_corr.set_3d_properties(corrected[:, 2])
            plt.draw()
        else:
            info_text.set_text(
                f"CALIBRATION FAILED \u2717\n"
                f"{'=' * 30}\n\n"
                f"Ellipsoid fit failed.\n"
                f"Check point spread.\n\n"
                f"Ensure 6 distinct\n"
                f"orientations."
            )
            print("  \u2717 Calibration failed (not enough spread?).")
            plt.draw()
    btn_calc.on_clicked(compute_calibration)

    def upload_calibration(event):
        B, S = calibration_result
        if np.allclose(B, 0) and np.allclose(S, np.eye(3)):
            print("  \u2717 No calibration computed yet.")
            return
        if not g_serial or not g_serial.is_open:
            print("  \u2717 Serial not connected")
            return
        send_calibration_upload(g_serial, B, S)
        info_text.set_text(
            f"UPLOADED \u2713\n"
            f"{'=' * 30}\n\n"
            f"Calibration sent to FC.\n"
            f"Saved to flash.\n\n"
            f"Click 'Verify' to confirm\n"
            f"flash storage.\n\n"
            f"Click 'View Cal' to see\n"
            f"calibrated output."
        )
        plt.draw()
    btn_upload.on_clicked(upload_calibration)

    def verify_flash(event):
        if not g_serial or not g_serial.is_open:
            print('  \u2717 Serial not connected')
            return
        send_log_class_command(g_serial, LOG_CLASS_STORAGE)
        info_text.set_text('Querying flash...')
        plt.draw()
        print('  \u23f3 Querying flash storage...')
    btn_vflash.on_clicked(verify_flash)

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
        send_calibration_upload(g_serial, TEST_STORAGE_BIAS, np.array(TEST_STORAGE_SCALE))
        g_test_storage_state = 'UPLOADING'
        g_test_storage_time = time.time()
        info_text.set_text(
            "STORAGE TEST\n"
            f"{'=' * 30}\n\n"
            "Uploading test data...\n"
            "Waiting for flash write\n"
            "(~2.5 seconds)\n"
        )
        print('  \u23f3 Test storage: uploading test calibration...')
    btn_teststorage.on_clicked(test_storage)

    g_last_savecsv_time = 0

    def save_csv(event):
        nonlocal g_last_savecsv_time
        now = time.time()
        if now - g_last_savecsv_time < 0.5:
            return
        g_last_savecsv_time = now
        if not raw_data_points:
            print("  \u2717 No positions captured")
            return
        filename = save_positions_csv(raw_data_points, g_chip_id)
        if filename:
            info_text.set_text(
                f"CSV SAVED \u2713\n"
                f"{'=' * 30}\n\n"
                f"Positions: {len(raw_data_points)}\n"
                f"File: {os.path.basename(filename)}\n\n"
                f"Saved to tools/data/"
            )
            plt.draw()
    btn_savecsv.on_clicked(save_csv)

    g_last_loadcsv_time = 0

    def load_csv(event):
        nonlocal g_last_loadcsv_time
        global raw_data_points, calibration_result
        now = time.time()
        if now - g_last_loadcsv_time < 1.0:
            return
        g_last_loadcsv_time = now

        # Use macOS file dialog
        import subprocess
        data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
        os.makedirs(data_dir, exist_ok=True)
        script = (
            f'set theFile to choose file with prompt "Select Accel CSV" '
            f'default location POSIX file "{data_dir}" '
            f'of type {{"csv"}}\n'
            f'return POSIX path of theFile'
        )
        try:
            result = subprocess.run(['osascript', '-e', script],
                                    capture_output=True, text=True, timeout=60)
            filepath = result.stdout.strip()
            if not filepath:
                print('  \u2717 No file selected')
                return
        except Exception as e:
            print(f'  \u2717 File dialog failed: {e}')
            return

        positions = load_positions_csv(filepath)
        if not positions:
            print('  \u2717 No valid positions in CSV')
            return

        raw_data_points = list(positions)
        calibration_result = (np.zeros(3), np.eye(3))

        # Update plot
        data_np = np.array(raw_data_points)
        plot_raw.set_data(data_np[:, 0], data_np[:, 1])
        plot_raw.set_3d_properties(data_np[:, 2])
        plot_corr.set_data([], [])
        plot_corr.set_3d_properties([])

        info_text.set_text(
            f"CSV LOADED \u2713\n"
            f"{'=' * 30}\n\n"
            f"Positions: {len(raw_data_points)}\n"
            f"File: {os.path.basename(filepath)}\n\n"
            f"Click 'Compute' to fit\n"
            f"ellipsoid."
        )
        plt.draw()
    btn_loadcsv.on_clicked(load_csv)

    def clear_points(event):
        global raw_data_points, calibration_result
        raw_data_points = []
        calibration_result = (np.zeros(3), np.eye(3))
        plot_raw.set_data([], [])
        plot_raw.set_3d_properties([])
        plot_corr.set_data([], [])
        plot_corr.set_3d_properties([])
        info_text.set_text(
            f"CLEARED\n"
            f"{'=' * 30}\n\n"
            f"All points removed.\n"
            f"Ready to capture."
        )
        print("  Points cleared.")
        plt.draw()
    btn_clear.on_clicked(clear_points)

    # --- Axes setup ---
    ax.set_xlabel('X', color=TEXT_COLOR)
    ax.set_ylabel('Y', color=TEXT_COLOR)
    ax.set_zlabel('Z', color=TEXT_COLOR)
    ax.legend(loc='upper left', fontsize=8,
              facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    ax.set_xlim(-PLOT_LIMIT, PLOT_LIMIT)
    ax.set_ylim(-PLOT_LIMIT, PLOT_LIMIT)
    ax.set_zlim(-PLOT_LIMIT, PLOT_LIMIT)

    def on_scroll(event):
        if event.inaxes != ax:
            return
        factor = 0.85 if event.button == 'up' else 1.15
        ax.set_xlim(ax.get_xlim()[0] * factor, ax.get_xlim()[1] * factor)
        ax.set_ylim(ax.get_ylim()[0] * factor, ax.get_ylim()[1] * factor)
        ax.set_zlim(ax.get_zlim()[0] * factor, ax.get_zlim()[1] * factor)
        plt.draw()
    fig.canvas.mpl_connect('scroll_event', on_scroll)

    # --- Chip ID auto-retry state ---
    chip_id_request_time = time.time()

    # --- Animation ---
    def update(frame):
        nonlocal chip_id_request_time

        data_updated = False
        chip_id_updated = False

        # Process chip ID responses (silently store)
        while not chip_id_queue.empty():
            global g_chip_id
            g_chip_id = chip_id_queue.get()
            chip_id_updated = True
            print(f"  \u2713 Chip ID: {g_chip_id}")

        # Auto-retry chip ID every 2s if not yet received
        if g_chip_id is None and g_serial and g_serial.is_open:
            now = time.time()
            if now - chip_id_request_time > 2.0:
                chip_id_request_time = now
                try:
                    send_chip_id_request(g_serial)
                except Exception:
                    pass

        # Process accel data
        while not data_queue.empty():
            pt = data_queue.get()
            g_received_count += 1
            g_last_raw = pt
            data_updated = True

            if is_capturing:
                current_position_samples.append(pt)
                if len(current_position_samples) >= SAMPLES_PER_POSITION:
                    avg_pt = np.mean(current_position_samples, axis=0)
                    raw_data_points.append(avg_pt)

                    n = len(raw_data_points)
                    orient = ORIENTATIONS[n - 1] if n <= len(ORIENTATIONS) else f"Position {n}"
                    print(f"  \u2713 Position {n} captured ({orient}): [{avg_pt[0]:.0f}, {avg_pt[1]:.0f}, {avg_pt[2]:.0f}]")

                    is_capturing = False
                    current_position_samples = []
                    btn_capture.label.set_text('Capture')

                    data_np = np.array(raw_data_points)
                    plot_raw.set_data(data_np[:, 0], data_np[:, 1])
                    plot_raw.set_3d_properties(data_np[:, 2])

                    # Show position list and next orientation hint
                    pos_lines = []
                    for i, p in enumerate(raw_data_points):
                        label = ORIENTATIONS[i] if i < len(ORIENTATIONS) else f"Extra {i+1}"
                        pos_lines.append(f"  {i+1}. {label}")
                    pos_list = "\n".join(pos_lines)

                    if n < 6:
                        next_orient = ORIENTATIONS[n]
                        hint = (
                            f"\nNEXT: {next_orient}\n"
                            f"Place drone, click 'Capture'"
                        )
                    elif n == 6:
                        hint = (
                            f"\n6 POSITIONS CAPTURED \u2713\n"
                            f"Click 'Compute' to fit.\n"
                            f"(or capture more for\n"
                            f" better accuracy)"
                        )
                    else:
                        hint = (
                            f"\n{n} positions captured.\n"
                            f"Click 'Compute' to fit."
                        )

                    info_text.set_text(
                        f"CAPTURED POSITIONS ({n})\n"
                        f"{'=' * 30}\n"
                        f"{pos_list}\n"
                        f"{hint}"
                    )
                    plt.draw()

        # Update live arrow and status
        if data_updated and g_last_raw is not None:
            plot_live_tip.set_data([g_last_raw[0]], [g_last_raw[1]])
            plot_live_tip.set_3d_properties([g_last_raw[2]])
            if live_arrow[0] is not None:
                live_arrow[0].remove()
            live_arrow[0] = ax.quiver(0, 0, 0, g_last_raw[0], g_last_raw[1], g_last_raw[2],
                                      color='yellow', arrow_length_ratio=0.08, linewidth=2)

            if g_verify_active:
                mag = np.sqrt(g_last_raw[0]**2 + g_last_raw[1]**2 + g_last_raw[2]**2)
                chip_line = f"Chip: {g_chip_id}\n" if g_chip_id else ""
                info_text.set_text(
                    f"VERIFY CALIBRATION\n"
                    f"{'=' * 30}\n"
                    f"{chip_line}"
                    f"Frames: {g_received_count}\n\n"
                    f"Calib X: {g_last_raw[0]:.1f}\n"
                    f"Calib Y: {g_last_raw[1]:.1f}\n"
                    f"Calib Z: {g_last_raw[2]:.1f}\n\n"
                    f"Magnitude: {mag:.0f}\n"
                    f"Expected:  ~16384\n\n"
                    f"Rotate drone around.\n"
                    f"Magnitude should stay\n"
                    f"consistent (~16384) in\n"
                    f"all orientations."
                )
            elif not is_capturing and len(raw_data_points) == 0:
                chip_line = f"Chip: {g_chip_id}\n\n" if g_chip_id else "\n"
                info_text.set_text(
                    f"LIVE DATA\n"
                    f"{'=' * 30}\n"
                    f"{chip_line}"
                    f"Frames: {g_received_count}\n\n"
                    f"X: {g_last_raw[0]:.1f} LSB\n"
                    f"Y: {g_last_raw[1]:.1f} LSB\n"
                    f"Z: {g_last_raw[2]:.1f} LSB\n\n"
                    f"Place drone flat (level)\n"
                    f"and click 'Capture'."
                )
            elif is_capturing:
                n = len(raw_data_points) + 1
                orient = ORIENTATIONS[n - 1] if n <= len(ORIENTATIONS) else f"Position {n}"
                info_text.set_text(
                    f"CAPTURING {orient}\n"
                    f"{'=' * 30}\n\n"
                    f"{len(current_position_samples)}/{SAMPLES_PER_POSITION} samples\n\n"
                    f"X: {g_last_raw[0]:.1f} LSB\n"
                    f"Y: {g_last_raw[1]:.1f} LSB\n"
                    f"Z: {g_last_raw[2]:.1f} LSB\n\n"
                    f"KEEP DRONE STILL!"
                )

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

        # Check for flash verification data
        while not verify_queue.empty():
            params = verify_queue.get()
            if g_serial and g_serial.is_open:
                send_log_class_command(g_serial, LOG_CLASS_NONE)
            g_logging_active = False
            g_verify_active = False
            btn_log.label.set_text('Start Log')
            btn_log.color = BTN_GREEN
            btn_log.hovercolor = BTN_GREEN_HOV
            btn_viewcal.label.set_text('View Cal')
            btn_viewcal.color = BTN_COLOR
            btn_viewcal.hovercolor = BTN_HOVER

            if g_test_storage_state == 'VERIFYING':
                g_test_storage_state = 'IDLE'
                stored_cal = params[4] if len(params) > 4 else 0.0
                ok = abs(stored_cal - 1.0) < 0.01
                if ok:
                    expected = TEST_STORAGE_BIAS + [s for row in TEST_STORAGE_SCALE for s in row]
                    for i in range(12):
                        got = params[5 + i] if len(params) > 5 + i else 0.0
                        if abs(got - expected[i]) > 0.0001:
                            ok = False
                            break
                if ok:
                    info_text.set_text(
                        f"STORAGE TEST PASSED \u2713\n"
                        f"{'=' * 30}\n\n"
                        f"Upload  \u2713\n"
                        f"Flash   \u2713\n"
                        f"Readback \u2713\n\n"
                        f"All 12 values\n"
                        f"verified correctly.\n\n"
                        f"{'=' * 30}\n"
                        f"Ready to calibrate.\n"
                    )
                    print('  \u2713 Storage test PASSED')
                else:
                    info_text.set_text(
                        f"STORAGE TEST FAILED \u2717\n"
                        f"{'=' * 30}\n\n"
                        f"Readback mismatch.\n"
                        f"Flag: {stored_cal:.1f}\n\n"
                        f"Check serial link\n"
                        f"and try again.\n"
                    )
                    print(f'  \u2717 Storage test FAILED (flag={stored_cal:.1f})')
            else:
                # Normal verify: Accel params in storage indices 4-16
                stored_cal = params[4]
                stored_bias = [params[5], params[6], params[7]]
                stored_scale = [[params[8 + r * 3 + c] for c in range(3)] for r in range(3)]

                if stored_cal > 0.0:
                    info_text.set_text(
                        f"FLASH VERIFIED \u2705\n"
                        f"{'=' * 30}\n\n"
                        f"Accel Calibrated: YES\n\n"
                        f"Stored Bias:\n"
                        f"[{stored_bias[0]:.1f},\n"
                        f" {stored_bias[1]:.1f},\n"
                        f" {stored_bias[2]:.1f}]\n\n"
                        f"Stored Scale diag:\n"
                        f"[{stored_scale[0][0]:.4f},\n"
                        f" {stored_scale[1][1]:.4f},\n"
                        f" {stored_scale[2][2]:.4f}]\n\n"
                        f"Active immediately.\n"
                        f"Click 'View Cal' to\n"
                        f"check magnitude."
                    )
                else:
                    info_text.set_text(
                        f"FLASH NOT CALIBRATED \u274c\n"
                        f"{'=' * 30}\n\n"
                        f"No accel calibration\n"
                        f"found in flash.\n\n"
                        f"Run calibration and\n"
                        f"upload first."
                    )
            plt.draw()

        return []

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
