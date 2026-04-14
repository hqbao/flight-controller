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
Magnetometer Calibration Tool

Interactive tool to compute Hard Iron (Bias) and Soft Iron (Scale/Skew)
calibration for the compass using Ellipsoid Fitting.

Calibration model:
    V_cal = S * (V_raw - B)
where B = ellipsoid center (hard iron bias, 3 floats, uT) and S = 3x3
symmetric matrix (soft iron correction) that maps the ellipsoid to a sphere.

Workflow:
    1. Connect FC via USB, click 'Start Log' -- yellow arrow = live reading.
    2. Click 'Stream', rotate drone in ALL directions (figure-8 motion).
       Cover the full sphere -- the more coverage, the better the fit.
    3. Watch 'Corrected Data' (green) become a sphere while streaming.
       Calibration auto-runs every 1 second with >= 20 points.
    4. Click 'Stop' when coverage is complete.
    5. Click 'Upload' -- sends bias + scale to FC, saved to flash.
    6. Click 'Verify' -- reads back flash to confirm.
    7. Click 'View Cal' -- switches to calibrated compass stream for validation.
       Magnitude should stay consistent (~1.0) in all orientations.

CSV persistence:
    - 'Save CSV' saves collected raw points to tools/data/ (gitignored).
    - 'Load CSV' reloads points for re-computation or cross-board comparison.
    - Chip ID is auto-requested and embedded in the CSV filename.
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
SEND_LOG_ID = 0x00

# Log class constants (match messages.h)
LOG_CLASS_NONE          = 0x00
LOG_CLASS_COMPASS       = 0x02
LOG_CLASS_COMPASS_CALIB = 0x0D
LOG_CLASS_STORAGE       = 0x10
DB_CMD_LOG_CLASS        = 0x03
DB_CMD_CALIBRATE_MAG    = 0x06
DB_CMD_RESET            = 0x07
DB_CMD_CHIP_ID          = 0x09
PLOT_LIMIT = 500  # Initial axis limit (scroll to zoom)


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
raw_data_points = []
is_collecting = False
g_serial = None
g_received_count = 0
g_last_raw = None
g_chip_id = None                  # hex string e.g. 'A1B2C3D4E5F67890'
calibration_result = (np.zeros(3), np.eye(3))
last_calibration_time = 0

# Test storage round-trip
g_test_storage_state = 'IDLE'  # IDLE, UPLOADING, VERIFYING
g_test_storage_time = 0
TEST_STORAGE_OFFSET = [11.11, 22.22, 33.33]
TEST_STORAGE_SCALE  = [[1.01, 0.02, 0.03],
                       [0.04, 1.05, 0.06],
                       [0.07, 0.08, 1.09]]


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
    """Send mag calibration data (offset + scale matrix) to the flight controller.

    Payload: 12 float32 values = 48 bytes
      [offset_x, offset_y, offset_z, s00, s01, s02, s10, s11, s12, s20, s21, s22]
    """
    msg_id = DB_CMD_CALIBRATE_MAG
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
    print(f"  \u2192 Mag calibration uploaded ({len(values)} floats, {len(frame)} bytes)")


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
                    # Compass data: 3 floats (x, y, z) = 12 bytes
                    if length == 12:
                        x, y, z = struct.unpack('<3f', payload)
                        if abs(x) <= 500 and abs(y) <= 500 and abs(z) <= 500:
                            data_queue.put((x, y, z))

                    # Chip ID response: 8 bytes (64-bit unique ID)
                    elif length == 8 and any(b != 0 for b in payload):
                        chip_id_hex = payload.hex().upper()
                        chip_id_queue.put(chip_id_hex)

                    # LOG_CLASS_STORAGE page 1: 30 floats (120 bytes)
                    elif length == 120:
                        values = struct.unpack('<30f', payload)
                        verify_queue.put(('page1', values))
                        print(f"  \u2713 Received flash page 1 ({length} bytes)")

                    # LOG_CLASS_STORAGE page 2: 18 floats (72 bytes)
                    elif length == 72:
                        values = struct.unpack('<18f', payload)
                        verify_queue.put(('page2', values))
                        print(f"  \u2713 Received flash page 2 ({length} bytes)")

    except Exception as e:
        print(f"Serial error: {e}")


# --- Calibration Algorithm ---

def calibrate_magnetometer(data):
    """
    Full ellipsoid fit for magnetometer calibration.

    Fits: Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
    (9 parameters, with cross-coupling terms for soft iron)

    Returns:
        center: Hard iron bias vector (3,) in uT
        S:      Soft iron correction matrix (3,3) symmetric
    """
    if len(data) < 10:
        return np.zeros(3), np.eye(3)

    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]

    D = np.array([x**2, y**2, z**2, x*y, x*z, y*z, x, y, z]).T
    ones = np.ones((len(data), 1))

    try:
        v, _, _, _ = np.linalg.lstsq(D, ones, rcond=None)
    except Exception:
        return None

    if v is None or np.isnan(v).any():
        return None

    v = v.flatten()

    A, B, C = v[0], v[1], v[2]
    D_coeff, E, F = v[3]/2, v[4]/2, v[5]/2
    G, H, I_coeff = v[6]/2, v[7]/2, v[8]/2

    A_mat = np.array([
        [A, D_coeff, E],
        [D_coeff, B, F],
        [E, F, C]
    ])

    B_vec = np.array([2*G, 2*H, 2*I_coeff])

    try:
        A_inv = np.linalg.inv(A_mat)
        center = -0.5 * np.dot(A_inv, B_vec)
    except np.linalg.LinAlgError:
        return None

    scale = 1 + np.dot(center.T, np.dot(A_mat, center))
    if scale <= 0:
        return None

    A_mat_scaled = A_mat / scale

    vals, vecs = np.linalg.eigh(A_mat_scaled)
    vals = np.abs(vals)

    sqrt_vals = np.sqrt(vals)
    D_sqrt = np.diag(sqrt_vals)

    # Calculate symmetric S = V * D_sqrt * V.T to preserve orientation
    S = np.dot(vecs, np.dot(D_sqrt, vecs.T))

    return center, S


# --- CSV Persistence ---

def save_points_csv(points, chip_id):
    """Save collected raw mag points to CSV in tools/data/."""
    if not points:
        print("  \u2717 No points to save")
        return None

    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
    os.makedirs(data_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    chip_tag = f'_{chip_id}' if chip_id else ''
    filename = os.path.join(data_dir, f'mag_cal{chip_tag}_{timestamp}.csv')

    with open(filename, 'w') as f:
        f.write('sample,mag_x_ut,mag_y_ut,mag_z_ut\n')
        for i, pt in enumerate(points):
            f.write(f'{i + 1},{pt[0]:.2f},{pt[1]:.2f},{pt[2]:.2f}\n')

    print(f'  \u2713 Saved {len(points)} points to {filename}')
    return filename


def load_points_csv(filepath):
    """Load raw mag points from CSV. Returns list of (x,y,z) tuples."""
    points = []
    try:
        with open(filepath, 'r') as f:
            header = f.readline()  # skip header
            for line in f:
                parts = line.strip().split(',')
                if len(parts) >= 4:
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    points.append((x, y, z))
        print(f'  \u2713 Loaded {len(points)} points from {filepath}')
    except Exception as e:
        print(f'  \u2717 Failed to load CSV: {e}')
    return points


# --- GUI ---

def main():
    global is_collecting, raw_data_points, calibration_result, last_calibration_time
    global g_received_count, g_last_raw
    global g_test_storage_state, g_test_storage_time

    show_raw = True
    show_calib = True
    run_calibration = True

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

    # Initialize plots (marker-only for reliable 3D rendering)
    plot_raw, = ax.plot([], [], [], 'ro', markersize=3, label='Raw Data')
    plot_last, = ax.plot([], [], [], 'co', markersize=8, label='Last Point')
    plot_corr, = ax.plot([], [], [], 'go', markersize=3, label='Corrected Data')
    live_arrow = [None]
    plot_live_tip, = ax.plot([], [], [], 'yD', markersize=8, label='Live Reading')

    # --- Info Panel ---
    text_ax = plt.axes([0.76, 0.15, 0.23, 0.75])
    text_ax.axis('off')
    text_ax.set_facecolor(PANEL_COLOR)
    instructions = (
        "MAG CALIBRATION\n"
        f"{'=' * 26}\n\n"
        "STEP 1: Click 'Start Log'\n"
        "  Yellow arrow = live data\n\n"
        "STEP 2: Click 'Stream'\n"
        "  Rotate drone in ALL\n"
        "  directions (figure-8).\n"
        "  Cover the full sphere.\n\n"
        "STEP 3: Watch green dots\n"
        "  form a sphere. Auto-fit\n"
        "  runs every 1 second.\n\n"
        "STEP 4: Click 'Upload'\n"
        "  Saves to FC flash\n\n"
        "STEP 5: Click 'Verify'\n"
        "  Reads back flash data\n\n"
        "STEP 6: Click 'View Cal'\n"
        "  Magnitude should be\n"
        "  ~1.0 in all orientations"
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

    # Row 2 (middle): Start Log | View Cal | Stream | Stop Algo | Test Stor | Chip ID | Reset FC
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

    btn_stream_ax = plt.axes([start_x + 2 * (w + gap), row2_y, w, btn_h])
    btn_stream = Button(btn_stream_ax, 'Stream', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_stream.label.set_color(TEXT_COLOR)
    btn_stream.label.set_fontsize(8)

    btn_algo_ax = plt.axes([start_x + 3 * (w + gap), row2_y, w, btn_h])
    btn_algo = Button(btn_algo_ax, 'Stop Algo', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_algo.label.set_color(TEXT_COLOR)
    btn_algo.label.set_fontsize(8)

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

    # Row 3 (bottom): Hide Raw | Hide Corr | Upload | Verify | Save CSV | Load CSV | Clear
    row3_y = 0.04

    btn_raw_ax = plt.axes([start_x, row3_y, w, btn_h])
    btn_raw = Button(btn_raw_ax, 'Hide Raw', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_raw.label.set_color(TEXT_COLOR)
    btn_raw.label.set_fontsize(8)

    btn_calib_ax = plt.axes([start_x + (w + gap), row3_y, w, btn_h])
    btn_calib = Button(btn_calib_ax, 'Hide Corr', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_calib.label.set_color(TEXT_COLOR)
    btn_calib.label.set_fontsize(8)

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
                send_log_class_command(g_serial, LOG_CLASS_COMPASS)
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
                send_log_class_command(g_serial, LOG_CLASS_COMPASS_CALIB)
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

    def toggle_stream(event):
        global is_collecting
        is_collecting = not is_collecting
        btn_stream.label.set_text('Stop' if is_collecting else 'Stream')
        btn_stream.color = BTN_RED if is_collecting else BTN_COLOR
        btn_stream.hovercolor = BTN_RED_HOV if is_collecting else BTN_HOVER
    btn_stream.on_clicked(toggle_stream)

    def toggle_algo(event):
        nonlocal run_calibration
        run_calibration = not run_calibration
        btn_algo.label.set_text('Stop Algo' if run_calibration else 'Run Algo')
    btn_algo.on_clicked(toggle_algo)

    def toggle_raw(event):
        nonlocal show_raw
        show_raw = not show_raw
        plot_raw.set_visible(show_raw)
        plot_last.set_visible(show_raw)
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

    def upload_calibration(event):
        B, S = calibration_result
        if np.allclose(B, 0) and np.allclose(S, np.eye(3)):
            print("  \u2717 No calibration computed yet. Collect data and wait for auto-fit.")
            return
        if not g_serial or not g_serial.is_open:
            print("  \u2717 Serial not connected")
            return
        send_calibration_upload(g_serial, B, S)
        info_text.set_text(
            f"UPLOADED \u2713\n"
            f"{'=' * 26}\n\n"
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
        send_calibration_upload(g_serial, TEST_STORAGE_OFFSET, np.array(TEST_STORAGE_SCALE))
        g_test_storage_state = 'UPLOADING'
        g_test_storage_time = time.time()
        info_text.set_text(
            "STORAGE TEST\n"
            f"{'=' * 26}\n\n"
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
            print("  \u2717 No points collected")
            return
        filename = save_points_csv(raw_data_points, g_chip_id)
        if filename:
            info_text.set_text(
                f"CSV SAVED \u2713\n"
                f"{'=' * 26}\n\n"
                f"Points: {len(raw_data_points)}\n"
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
            f'set theFile to choose file with prompt "Select Mag CSV" '
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

        points = load_points_csv(filepath)
        if not points:
            print('  \u2717 No valid points in CSV')
            return

        raw_data_points = list(points)
        calibration_result = (np.zeros(3), np.eye(3))

        # Update plot
        data_np = np.array(raw_data_points)
        plot_raw.set_data(data_np[:, 0], data_np[:, 1])
        plot_raw.set_3d_properties(data_np[:, 2])
        plot_corr.set_data([], [])
        plot_corr.set_3d_properties([])

        if len(data_np) > 0:
            last_pt = data_np[-1]
            plot_last.set_data([last_pt[0]], [last_pt[1]])
            plot_last.set_3d_properties([last_pt[2]])

        info_text.set_text(
            f"CSV LOADED \u2713\n"
            f"{'=' * 26}\n\n"
            f"Points: {len(raw_data_points)}\n"
            f"File: {os.path.basename(filepath)}\n\n"
            f"Auto-fit will run if\n"
            f"'Run Algo' is active\n"
            f"and streaming resumes."
        )
        plt.draw()
    btn_loadcsv.on_clicked(load_csv)

    def clear_points(event):
        global raw_data_points, calibration_result
        raw_data_points = []
        calibration_result = (np.zeros(3), np.eye(3))
        plot_raw.set_data([], [])
        plot_raw.set_3d_properties([])
        plot_last.set_data([], [])
        plot_last.set_3d_properties([])
        plot_corr.set_data([], [])
        plot_corr.set_3d_properties([])
        info_text.set_text(
            f"CLEARED\n"
            f"{'=' * 26}\n\n"
            f"All points removed.\n"
            f"Ready to collect."
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
        global last_calibration_time

        data_updated = False
        points_added = False
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

        # Process compass data
        while not data_queue.empty():
            pt = data_queue.get()
            g_received_count += 1
            g_last_raw = pt
            data_updated = True
            if is_collecting:
                raw_data_points.append(pt)
                points_added = True

        if points_added:
            data_np = np.array(raw_data_points)

            # Update Raw Plot
            plot_raw.set_data(data_np[:, 0], data_np[:, 1])
            plot_raw.set_3d_properties(data_np[:, 2])

            # Update Last Point
            if len(data_np) > 0:
                last_pt = data_np[-1]
                plot_last.set_data([last_pt[0]], [last_pt[1]])
                plot_last.set_3d_properties([last_pt[2]])

            # Realtime calibration every 1s
            if run_calibration and time.time() - last_calibration_time > 1.0 and len(raw_data_points) > 20:
                res = calibrate_magnetometer(data_np)
                if res is not None:
                    B, S = res
                    calibration_result = (B, S)
                last_calibration_time = time.time()

            # Update corrected points display
            if len(raw_data_points) > 0:
                B, S = calibration_result

                raw_centered = (data_np - B).T
                corrected = np.dot(S, raw_centered).T

                avg_radius = np.mean(np.linalg.norm(raw_centered, axis=0))
                corrected_scaled = corrected * avg_radius

                plot_corr.set_data(corrected_scaled[:, 0], corrected_scaled[:, 1])
                plot_corr.set_3d_properties(corrected_scaled[:, 2])

                if len(corrected) > 0:
                    last_corr = corrected[-1]
                    B_str = f"[{B[0]:.2f}, {B[1]:.2f}, {B[2]:.2f}]"
                    S_str = "\n".join([f"[{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}]" for r in S])
                    chip_line = f"Chip: {g_chip_id}\n" if g_chip_id else ""
                    info_text.set_text(
                        f"STREAMING ({len(raw_data_points)} pts)\n"
                        f"{'=' * 26}\n"
                        f"{chip_line}\n"
                        f"Hard Iron Bias B (uT):\n{B_str}\n\n"
                        f"Soft Iron Matrix S:\n{S_str}\n\n"
                        f"Last Calibrated (unit):\n"
                        f"[{last_corr[0]:.3f}, {last_corr[1]:.3f},\n"
                        f" {last_corr[2]:.3f}]"
                    )

        # Update live arrow and status when new data arrived (even if not collecting)
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
                    f"{'=' * 26}\n"
                    f"{chip_line}"
                    f"Frames: {g_received_count}\n\n"
                    f"Calib X: {g_last_raw[0]:.4f}\n"
                    f"Calib Y: {g_last_raw[1]:.4f}\n"
                    f"Calib Z: {g_last_raw[2]:.4f}\n\n"
                    f"Magnitude: {mag:.4f}\n"
                    f"Expected:  ~1.0000\n\n"
                    f"Rotate drone around.\n"
                    f"Magnitude should stay\n"
                    f"consistent (~1.0) in\n"
                    f"all orientations."
                )
            elif not is_collecting and len(raw_data_points) == 0:
                mag = np.sqrt(g_last_raw[0]**2 + g_last_raw[1]**2 + g_last_raw[2]**2)
                chip_line = f"Chip: {g_chip_id}\n\n" if g_chip_id else "\n"
                info_text.set_text(
                    f"LIVE DATA\n"
                    f"{'=' * 26}\n"
                    f"{chip_line}"
                    f"Frames: {g_received_count}\n\n"
                    f"Mag X: {g_last_raw[0]:.1f} uT\n"
                    f"Mag Y: {g_last_raw[1]:.1f} uT\n"
                    f"Mag Z: {g_last_raw[2]:.1f} uT\n"
                    f"Field:  {mag:.1f} uT\n\n"
                    f"Click 'Stream' then\n"
                    f"rotate drone."
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
            page_tag, params = verify_queue.get()

            # Only process page 1 (contains mag params at indices 17-29)
            if page_tag != 'page1':
                continue

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
                stored_cal = params[17] if len(params) > 17 else 0.0
                ok = abs(stored_cal - 1.0) < 0.01
                if ok:
                    expected = TEST_STORAGE_OFFSET + [s for row in TEST_STORAGE_SCALE for s in row]
                    for i in range(12):
                        got = params[18 + i] if len(params) > 18 + i else 0.0
                        if abs(got - expected[i]) > 0.0001:
                            ok = False
                            break
                if ok:
                    info_text.set_text(
                        f"STORAGE TEST PASSED \u2713\n"
                        f"{'=' * 26}\n\n"
                        f"Upload  \u2713\n"
                        f"Flash   \u2713\n"
                        f"Readback \u2713\n\n"
                        f"All 12 values\n"
                        f"verified correctly.\n\n"
                        f"{'=' * 26}\n"
                        f"Ready to calibrate.\n"
                    )
                    print('  \u2713 Storage test PASSED')
                else:
                    info_text.set_text(
                        f"STORAGE TEST FAILED \u2717\n"
                        f"{'=' * 26}\n\n"
                        f"Readback mismatch.\n"
                        f"Flag: {stored_cal:.1f}\n\n"
                        f"Check serial link\n"
                        f"and try again.\n"
                    )
                    print(f'  \u2717 Storage test FAILED (flag={stored_cal:.1f})')
            else:
                # Normal verify: Mag params at indices 17-29
                stored_cal = params[17]
                stored_offset = [params[18], params[19], params[20]]
                stored_scale = [[params[21 + r * 3 + c] for c in range(3)] for r in range(3)]

                if stored_cal > 0.0:
                    info_text.set_text(
                        f"FLASH VERIFIED \u2705\n"
                        f"{'=' * 26}\n\n"
                        f"Mag Calibrated: YES\n\n"
                        f"Stored Offset (uT):\n"
                        f"[{stored_offset[0]:.2f},\n"
                        f" {stored_offset[1]:.2f},\n"
                        f" {stored_offset[2]:.2f}]\n\n"
                        f"Stored Scale:\n" +
                        "\n".join([f"[{r[0]:.4f}, {r[1]:.4f},\n {r[2]:.4f}]" for r in stored_scale]) +
                        f"\n\nActive immediately.\n"
                        f"Click 'View Cal' to\n"
                        f"check magnitude."
                    )
                else:
                    info_text.set_text(
                        f"FLASH NOT CALIBRATED \u274c\n"
                        f"{'=' * 26}\n\n"
                        f"No mag calibration\n"
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
