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
import csv
import os
import subprocess

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
    1. Click 'Start Log' -- yellow arrow = live reading.
    2. Place drone in each orientation, click 'Capture' (6 positions).
    3. Click 'Compute' -- fits ellipsoid.
    4. Click 'Upload' -- sends bias + scale to flash.
    5. Click 'Query FC' to verify stored coefficients.
    6. Click 'View Cal' to check calibrated output (~16384 magnitude).
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 19200
SEND_LOG_ID = 0x00

# Log class constants (match messages.h)
LOG_CLASS_NONE            = 0x00
LOG_CLASS_HEART_BEAT      = 0x09
LOG_CLASS_IMU_ACCEL_RAW   = 0x01
LOG_CLASS_IMU_ACCEL_CALIB = 0x0A
LOG_CLASS_STORAGE         = 0x10
DB_CMD_LOG_CLASS          = 0x03
DB_CMD_CALIBRATE_ACCEL    = 0x05
DB_CMD_RESET              = 0x07
DB_CMD_CHIP_ID            = 0x09
PLOT_LIMIT = 20000
SAMPLES_PER_POSITION = 100

# FC storage layout: accel params at indices 4-16
# index 4 = calibrated flag, 5-7 = bias, 8-16 = scale matrix
PARAM_ACCEL_CAL_FLAG = 4
PARAM_ACCEL_BIAS_BASE = 5
PARAM_ACCEL_SCALE_BASE = 8

# Auto-detect serial port
ports = serial.tools.list_ports.comports()
print("Scanning for ports...")
for port, desc, hwid in sorted(ports):
    if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB']):
        SERIAL_PORT = port
        print(f"  \u2713 Auto-selected: {port} ({desc})")
        break
    else:
        print(f"  \u00b7 Skipped: {port} ({desc})")

if not SERIAL_PORT:
    print("  \u2717 No compatible serial port found.")

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
BTN_ACTIVE     = '#4488cc'
BTN_ACTIVE_HOV = '#5599dd'

# --- Global State ---
data_queue = queue.Queue()
storage_queue = queue.Queue()
heartbeat_queue = queue.Queue()
raw_data_points = []
current_position_samples = []
is_capturing = False
g_serial = None
g_received_count = 0
g_last_raw = None
g_chip_id = None
g_querying_storage = False
calibration_result = (np.zeros(3), np.eye(3))

ORIENTATIONS = [
    "Flat (level, Z up)",
    "Inverted (Z down)",
    "Left side (Y up)",
    "Right side (Y down)",
    "Nose up (X up)",
    "Nose down (X down)",
]


# --- DB Protocol Helpers ---

def build_db_frame(cmd_id, payload_bytes):
    msg_class = 0x00
    length = len(payload_bytes)
    header = struct.pack('<2sBBH', b'db', cmd_id, msg_class, length)
    checksum = cmd_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
    for b in payload_bytes:
        checksum += b
    checksum &= 0xFFFF
    return header + payload_bytes + struct.pack('<H', checksum)


def send_log_class(ser, log_class):
    ser.write(build_db_frame(DB_CMD_LOG_CLASS, bytes([log_class])))
    ser.flush()


def send_chip_id_request(ser):
    ser.write(build_db_frame(DB_CMD_CHIP_ID, bytes([0x00])))
    ser.flush()


def send_reset(ser):
    ser.write(build_db_frame(DB_CMD_RESET, bytes([0x00])))
    ser.flush()


def send_calibration_upload(ser, bias, scale):
    values = list(bias) + [scale[r][c] for r in range(3) for c in range(3)]
    payload = struct.pack('<12f', *values)
    ser.write(build_db_frame(DB_CMD_CALIBRATE_ACCEL, payload))
    ser.flush()
    print(f"  \u2192 Accel calibration uploaded (12 floats)")


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
            time.sleep(0.3)
            send_chip_id_request(ser)
            send_log_class(ser, LOG_CLASS_HEART_BEAT)

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

                if msg_id == SEND_LOG_ID:
                    # Accel data: 4 floats (ax, ay, az, temp) = 16 bytes
                    if length == 16:
                        x, y, z, temp = struct.unpack('<4f', payload)
                        if abs(x) <= 40000 and abs(y) <= 40000 and abs(z) <= 40000:
                            data_queue.put((x, y, z))

                    # Chip ID: 8 bytes
                    elif length == 8 and any(b != 0 for b in payload):
                        heartbeat_queue.put(('chip_id', payload.hex().upper()))

                    # Storage page 0: 30 floats = 120 bytes
                    elif length == 120:
                        storage_queue.put(('page0', payload))

                    # Storage page 1: 18 floats = 72 bytes
                    elif length == 72:
                        storage_queue.put(('page1', payload))

    except Exception as e:
        print(f"Serial error: {e}")


# --- Calibration Algorithm ---

def calibrate_accelerometer(data):
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

    raw_centered = data - center
    avg_radius = np.mean(np.linalg.norm(raw_centered, axis=1))
    S = S * avg_radius

    return center, S


# --- GUI ---

def main():
    global is_capturing, raw_data_points, current_position_samples, calibration_result
    global g_received_count, g_last_raw, g_querying_storage

    threading.Thread(target=serial_reader, daemon=True).start()

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
    plt.subplots_adjust(bottom=0.22, right=0.75)
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.pane.fill = False
        axis.pane.set_edgecolor(GRID_COLOR)
        axis.label.set_color(TEXT_COLOR)
        axis._axinfo['tick']['color'] = TEXT_COLOR
        axis._axinfo['tick']['inward_factor'] = 0
    ax.tick_params(colors=DIM_TEXT)

    plot_raw, = ax.plot([], [], [], 'ro', markersize=10, label='Raw (LSB)')
    plot_corr, = ax.plot([], [], [], 'go', markersize=10, label='Corrected')
    live_arrow = [None]
    plot_live_tip, = ax.plot([], [], [], 'yD', markersize=8, label='Live')

    # --- Info Panel ---
    text_ax = plt.axes([0.76, 0.22, 0.23, 0.68])
    text_ax.axis('off')
    text_ax.set_facecolor(PANEL_COLOR)
    instructions = (
        "ACCEL 6-POSITION CALIBRATION\n"
        f"{'=' * 30}\n\n"
        "1. Click 'Start Log'\n"
        "   Yellow arrow = live reading\n\n"
        "2. Place drone in each orientation,\n"
        "   click 'Capture':\n"
        "   \u2022 Flat (level, Z up)\n"
        "   \u2022 Inverted (Z down)\n"
        "   \u2022 Left side (Y up)\n"
        "   \u2022 Right side (Y down)\n"
        "   \u2022 Nose up (X up)\n"
        "   \u2022 Nose down (X down)\n\n"
        "3. Click 'Compute'\n"
        "4. Click 'Upload'\n"
        "5. Click 'Query FC' to verify\n"
        "6. Click 'View Cal' to check"
    )
    info_text = text_ax.text(0.05, 0.98, instructions, fontsize=8, va='top',
                             fontfamily='monospace', color=TEXT_COLOR)

    # --- Button Layout ---
    start_x = 0.02
    btn_h = 0.04
    w = 0.10
    gap = 0.01

    # Row 1: View buttons (Top, Bottom, Front, Back, Left, Right)
    row1_y = 0.16
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

    # Row 2: Start Log | View Cal | Capture | Compute | Upload | Default | Query FC
    row2_y = 0.10

    g_logging_active = False
    g_verify_active = False

    ax_log = plt.axes([start_x, row2_y, w, btn_h])
    btn_log = Button(ax_log, 'Start Log', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_log.label.set_color(TEXT_COLOR)
    btn_log.label.set_fontsize(8)

    ax_viewcal = plt.axes([start_x + (w + gap), row2_y, w, btn_h])
    btn_viewcal = Button(ax_viewcal, 'View Cal', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_viewcal.label.set_color(TEXT_COLOR)
    btn_viewcal.label.set_fontsize(8)

    ax_capture = plt.axes([start_x + 2 * (w + gap), row2_y, w, btn_h])
    btn_capture = Button(ax_capture, 'Capture', color=BTN_ORANGE, hovercolor=BTN_ORANGE_HOV)
    btn_capture.label.set_color(TEXT_COLOR)
    btn_capture.label.set_fontsize(8)

    ax_calc = plt.axes([start_x + 3 * (w + gap), row2_y, w, btn_h])
    btn_calc = Button(ax_calc, 'Compute', color=BTN_ORANGE, hovercolor=BTN_ORANGE_HOV)
    btn_calc.label.set_color(TEXT_COLOR)
    btn_calc.label.set_fontsize(8)

    ax_upload = plt.axes([start_x + 4 * (w + gap), row2_y, w, btn_h])
    btn_upload = Button(ax_upload, 'Upload', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_upload.label.set_color(TEXT_COLOR)
    btn_upload.label.set_fontsize(8)
    btn_upload.label.set_fontweight('bold')

    ax_default = plt.axes([start_x + 5 * (w + gap), row2_y, w, btn_h])
    btn_default = Button(ax_default, 'Default', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_default.label.set_color(TEXT_COLOR)
    btn_default.label.set_fontsize(8)

    ax_query = plt.axes([start_x + 6 * (w + gap), row2_y, w, btn_h])
    btn_query = Button(ax_query, 'Query FC', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_query.label.set_color(TEXT_COLOR)
    btn_query.label.set_fontsize(8)

    # Row 3: Save CSV | Load CSV | Clear | Reset FC
    row3_y = 0.04

    ax_savecsv = plt.axes([start_x, row3_y, w, btn_h])
    btn_savecsv = Button(ax_savecsv, 'Save CSV', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_savecsv.label.set_color(TEXT_COLOR)
    btn_savecsv.label.set_fontsize(8)

    ax_loadcsv = plt.axes([start_x + (w + gap), row3_y, w, btn_h])
    btn_loadcsv = Button(ax_loadcsv, 'Load CSV', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_loadcsv.label.set_color(TEXT_COLOR)
    btn_loadcsv.label.set_fontsize(8)

    ax_clear = plt.axes([start_x + 2 * (w + gap), row3_y, w, btn_h])
    btn_clear = Button(ax_clear, 'Clear', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_clear.label.set_color(TEXT_COLOR)
    btn_clear.label.set_fontsize(8)

    ax_reset = plt.axes([start_x + 3 * (w + gap), row3_y, w, btn_h])
    btn_reset = Button(ax_reset, 'Reset FC', color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_reset.label.set_color(TEXT_COLOR)
    btn_reset.label.set_fontsize(8)

    # --- Button Callbacks ---

    def toggle_log(event):
        nonlocal g_logging_active, g_verify_active
        if not g_serial or not g_serial.is_open:
            return
        if g_logging_active:
            send_log_class(g_serial, LOG_CLASS_NONE)
            g_logging_active = False
            btn_log.label.set_text('Start Log')
            btn_log.color = BTN_GREEN
            btn_log.hovercolor = BTN_GREEN_HOV
        else:
            send_log_class(g_serial, LOG_CLASS_IMU_ACCEL_RAW)
            g_logging_active = True
            g_verify_active = False
            btn_log.label.set_text('Stop Log')
            btn_log.color = BTN_RED
            btn_log.hovercolor = BTN_RED_HOV
            btn_viewcal.label.set_text('View Cal')
            btn_viewcal.color = BTN_COLOR
            btn_viewcal.hovercolor = BTN_HOVER
    btn_log.on_clicked(toggle_log)

    def toggle_view_cal(event):
        nonlocal g_logging_active, g_verify_active
        if not g_serial or not g_serial.is_open:
            return
        if g_verify_active:
            send_log_class(g_serial, LOG_CLASS_NONE)
            g_verify_active = False
            btn_viewcal.label.set_text('View Cal')
            btn_viewcal.color = BTN_COLOR
            btn_viewcal.hovercolor = BTN_HOVER
        else:
            send_log_class(g_serial, LOG_CLASS_IMU_ACCEL_CALIB)
            g_verify_active = True
            g_logging_active = False
            btn_viewcal.label.set_text('View Raw')
            btn_viewcal.color = BTN_ACTIVE
            btn_viewcal.hovercolor = BTN_ACTIVE_HOV
            btn_log.label.set_text('Start Log')
            btn_log.color = BTN_GREEN
            btn_log.hovercolor = BTN_GREEN_HOV
    btn_viewcal.on_clicked(toggle_view_cal)

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
            plt.draw()
            return

        data_np = np.array(raw_data_points)
        res = calibrate_accelerometer(data_np)
        if res is not None:
            B, S = res
            calibration_result = (B, S)

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
            print("  \u2713 Calibration computed")
        else:
            info_text.set_text(
                f"CALIBRATION FAILED \u2717\n"
                f"{'=' * 30}\n\n"
                f"Ellipsoid fit failed.\n"
                f"Check point spread.\n\n"
                f"Ensure 6 distinct\n"
                f"orientations."
            )
            plt.draw()
            print("  \u2717 Calibration failed")
    btn_calc.on_clicked(compute_calibration)

    def upload_calibration(event):
        B, S = calibration_result
        if np.allclose(B, 0) and np.allclose(S, np.eye(3)):
            print("  \u2717 No calibration computed yet.")
            return
        if not g_serial or not g_serial.is_open:
            return
        send_calibration_upload(g_serial, B, S)
        info_text.set_text(
            f"UPLOADED \u2713\n"
            f"{'=' * 30}\n\n"
            f"Calibration sent to FC.\n"
            f"Saved to flash.\n\n"
            f"Click 'Query FC' to verify.\n\n"
            f"Click 'View Cal' to see\n"
            f"calibrated output."
        )
        plt.draw()
    btn_upload.on_clicked(upload_calibration)

    def default_calibration(event):
        if not g_serial or not g_serial.is_open:
            return
        bias = [0.0, 0.0, 0.0]
        scale = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        send_calibration_upload(g_serial, bias, scale)
        info_text.set_text(
            f"DEFAULT UPLOADED \u2713\n"
            f"{'=' * 30}\n\n"
            f"Identity calibration sent:\n"
            f"  Bias  = [0, 0, 0]\n"
            f"  Scale = Identity 3x3\n\n"
            f"Click 'Query FC' to verify."
        )
        plt.draw()
    btn_default.on_clicked(default_calibration)

    def on_query(event):
        global g_querying_storage
        if not g_serial or not g_serial.is_open:
            return
        while not storage_queue.empty(): storage_queue.get_nowait()
        g_querying_storage = True
        send_log_class(g_serial, LOG_CLASS_STORAGE)
        info_text.set_text('Querying FC...')
        plt.draw()
    btn_query.on_clicked(on_query)

    def get_cal_dir():
        cal_dir = os.path.join(os.path.dirname(__file__), '.calibration_data')
        os.makedirs(cal_dir, exist_ok=True)
        return cal_dir

    def save_csv(event):
        if not raw_data_points:
            print("  \u2717 No positions captured")
            return
        chip_tag = f'_{g_chip_id}' if g_chip_id else ''
        fname = f'accel_cal{chip_tag}.csv'
        fpath = os.path.join(get_cal_dir(), fname)
        with open(fpath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['position', 'accel_x_lsb', 'accel_y_lsb', 'accel_z_lsb'])
            for i, pt in enumerate(raw_data_points):
                writer.writerow([i + 1, f'{pt[0]:.2f}', f'{pt[1]:.2f}', f'{pt[2]:.2f}'])
        info_text.set_text(
            f"CSV SAVED \u2713\n"
            f"{'=' * 30}\n\n"
            f"Positions: {len(raw_data_points)}\n"
            f"File: {fname}"
        )
        plt.draw()
        print(f'  \u2713 Saved {len(raw_data_points)} positions to {fpath}')
    btn_savecsv.on_clicked(save_csv)

    # Async file picker queue
    load_csv_queue = queue.Queue()

    def load_csv(event):
        cal_dir = get_cal_dir()
        info_text.set_text('Opening file picker...')
        plt.draw()
        def _pick_file():
            try:
                result = subprocess.run(
                    ['osascript', '-e',
                     'POSIX path of (choose file with prompt "Load accel calibration CSV" '
                     'default location (POSIX file "' + cal_dir + '") '
                     'of type {"public.comma-separated-values-text", "public.text"})'],
                    capture_output=True, text=True, timeout=120
                )
                fpath = result.stdout.strip()
                load_csv_queue.put(fpath if fpath else None)
            except Exception:
                load_csv_queue.put(None)
        threading.Thread(target=_pick_file, daemon=True).start()
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
        plt.draw()
    btn_clear.on_clicked(clear_points)

    def reset_fc(event):
        nonlocal g_logging_active, g_verify_active
        if not g_serial or not g_serial.is_open:
            return
        g_logging_active = False
        g_verify_active = False
        btn_log.label.set_text('Start Log')
        btn_log.color = BTN_GREEN
        btn_log.hovercolor = BTN_GREEN_HOV
        btn_viewcal.label.set_text('View Cal')
        btn_viewcal.color = BTN_COLOR
        btn_viewcal.hovercolor = BTN_HOVER
        send_reset(g_serial)
    btn_reset.on_clicked(reset_fc)

    # --- Axes setup ---
    ax.set_title('Accelerometer Calibration', color=TEXT_COLOR, fontsize=12, pad=10)
    ax.set_xlabel('X (LSB)', color=TEXT_COLOR)
    ax.set_ylabel('Y (LSB)', color=TEXT_COLOR)
    ax.set_zlabel('Z (LSB)', color=TEXT_COLOR)
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

    # --- Animation ---
    storage_pages = {}

    def update(frame):
        nonlocal g_logging_active, g_verify_active
        global g_chip_id, g_received_count, g_last_raw
        global is_capturing, current_position_samples
        global calibration_result, raw_data_points, g_querying_storage

        # Process chip ID
        while not heartbeat_queue.empty():
            try:
                msg_type, value = heartbeat_queue.get_nowait()
            except queue.Empty:
                break
            if msg_type == 'chip_id':
                g_chip_id = value
                print(f"  \u2713 Chip ID: {g_chip_id}")

        # Retry chip ID
        if g_chip_id is None and g_serial and g_serial.is_open:
            if not hasattr(update, '_chip_retry'):
                update._chip_retry = time.time()
            now = time.time()
            if now - update._chip_retry > 2.0:
                update._chip_retry = now
                try:
                    send_chip_id_request(g_serial)
                except Exception:
                    pass

        # Process async file picker result
        while not load_csv_queue.empty():
            try:
                fpath = load_csv_queue.get_nowait()
            except queue.Empty:
                break
            if fpath is None:
                info_text.set_text('')
                continue
            positions = []
            try:
                with open(fpath, 'r') as f:
                    reader = csv.reader(f)
                    next(reader, None)
                    for row in reader:
                        if len(row) >= 4:
                            positions.append((float(row[1]), float(row[2]), float(row[3])))
            except Exception as e:
                info_text.set_text(f'Error: {e}')
                continue
            if not positions:
                info_text.set_text('No valid positions in CSV')
                continue

            raw_data_points = list(positions)
            calibration_result = (np.zeros(3), np.eye(3))
            data_np = np.array(raw_data_points)
            plot_raw.set_data(data_np[:, 0], data_np[:, 1])
            plot_raw.set_3d_properties(data_np[:, 2])
            plot_corr.set_data([], [])
            plot_corr.set_3d_properties([])
            fname = os.path.basename(fpath)
            info_text.set_text(
                f"CSV LOADED \u2713\n"
                f"{'=' * 30}\n\n"
                f"Positions: {len(raw_data_points)}\n"
                f"File: {fname}\n\n"
                f"Click 'Compute' to fit."
            )
            print(f'  \u2713 Loaded {len(raw_data_points)} positions from {fpath}')

        # Process storage readback
        while not storage_queue.empty():
            try:
                page_tag, payload = storage_queue.get_nowait()
            except queue.Empty:
                break
            if page_tag == 'page0':
                storage_pages[0] = struct.unpack('<30f', payload)

        if g_querying_storage and 0 in storage_pages:
            params = storage_pages[0]
            stored_cal = params[PARAM_ACCEL_CAL_FLAG]

            if stored_cal > 0.0:
                stored_bias = [params[PARAM_ACCEL_BIAS_BASE + i] for i in range(3)]
                stored_scale = [[params[PARAM_ACCEL_SCALE_BASE + r * 3 + c]
                                 for c in range(3)] for r in range(3)]
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
                    f"Active immediately."
                )
                print(f'  \u2713 Loaded accel calibration from flash')
            else:
                info_text.set_text(
                    f"NOT CALIBRATED\n"
                    f"{'=' * 30}\n\n"
                    f"No accel calibration\n"
                    f"found in flash."
                )

            storage_pages.clear()
            g_querying_storage = False
            if g_logging_active and g_serial and g_serial.is_open:
                send_log_class(g_serial, LOG_CLASS_IMU_ACCEL_RAW)

        # Process accel data
        data_updated = False
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
                            f"Click 'Compute' to fit."
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

        return []

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
