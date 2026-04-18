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
import os
import subprocess

"""
Magnetometer Calibration Tool (Ellipsoid Fit)

Computes Hard Iron (Bias) and Soft Iron (Scale) calibration for the
compass using full ellipsoid fitting.

Calibration model:
    V_cal = S * (V_raw - B)
where B = ellipsoid center (hard iron bias, 3 floats, uT) and S = 3x3
symmetric matrix (soft iron correction) that maps the ellipsoid to a sphere.

Workflow:
    1. Click 'Start Log' -- yellow arrow = live reading.
    2. Click 'Stream', rotate drone in ALL directions (figure-8 motion).
       Cover the full sphere -- the more coverage, the better the fit.
       Watch red dots fill the sphere and green dots become spherical.
    3. Auto-fit runs every 1 second with >= 20 points.
    4. Click 'Stream' again to stop when coverage is complete.
    5. Click 'Upload' -- sends bias + scale to FC flash.
    6. Click 'Query FC' to verify stored coefficients.
    7. Click 'View Cal' to check calibrated output (~1.0 magnitude).

CSV persistence:
    - 'Save CSV' saves collected raw points to .calibration_data/ (gitignored).
    - 'Load CSV' reloads points for re-computation or cross-board comparison.
    - Chip ID is auto-requested and embedded in the CSV filename.
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 19200
SEND_LOG_ID = 0x00

# Log class constants (match messages.h)
LOG_CLASS_NONE          = 0x00
LOG_CLASS_HEART_BEAT    = 0x09
LOG_CLASS_COMPASS       = 0x02
LOG_CLASS_COMPASS_CALIB = 0x0D
LOG_CLASS_STORAGE       = 0x10
DB_CMD_LOG_CLASS        = 0x03
DB_CMD_CALIBRATE_MAG    = 0x06
DB_CMD_RESET            = 0x07
DB_CMD_CHIP_ID          = 0x09
PLOT_LIMIT = 500  # Initial axis limit (scroll to zoom)

# FC storage layout: mag params at indices 17-29
# index 17 = calibrated flag, 18-20 = bias, 21-29 = scale matrix
PARAM_MAG_CAL_FLAG = 17
PARAM_MAG_BIAS_BASE = 18
PARAM_MAG_SCALE_BASE = 21

# Auto-detect serial port
ports = serial.tools.list_ports.comports()
print("Scanning for ports...")
for port, desc, hwid in sorted(ports):
    if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB', 'COM']):
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

# --- Global State ---
data_queue = queue.Queue()
storage_queue = queue.Queue()
raw_data_points = []
is_collecting = False
g_serial = None
g_received_count = 0
g_last_raw = None
g_chip_id = None
g_querying_storage = False
calibration_result = (np.zeros(3), np.eye(3))
last_calibration_time = 0


# --- DB Protocol Helpers ---


def _open_file_dialog(title="Select file", initialdir=".", filetypes=None):
    """Cross-platform file picker (works on macOS, Windows, Linux)."""
    if filetypes is None:
        filetypes = [("CSV files", "*.csv"), ("All files", "*.*")]
    if sys.platform == 'darwin':
        import subprocess
        script = (
            f'set theFile to choose file with prompt "{title}" '
            f'default location POSIX file "{initialdir}" '
            f'of type {{"csv"}}\n'
            f'return POSIX path of theFile'
        )
        try:
            result = subprocess.run(['osascript', '-e', script],
                                    capture_output=True, text=True, timeout=120)
            fpath = result.stdout.strip()
            return fpath if fpath else None
        except Exception:
            return None
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes('-topmost', True)
        fpath = filedialog.askopenfilename(
            title=title, initialdir=initialdir, filetypes=filetypes)
        root.destroy()
        return fpath if fpath else None
    except Exception:
        return None

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
    ser.write(build_db_frame(DB_CMD_CALIBRATE_MAG, payload))
    ser.flush()
    print(f"  \u2192 Mag calibration uploaded (12 floats)")


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
                        global g_chip_id
                        g_chip_id = payload.hex().upper()
                        print(f"  \u2713 Chip ID: {g_chip_id}")

                    # LOG_CLASS_STORAGE: 26 floats = 104 bytes (4 pages sent sequentially)
                    elif length == 104:
                        values = struct.unpack('<26f', payload)
                        storage_queue.put(values)

    except Exception as e:
        print(f"Serial error: {e}")


# --- Calibration Algorithm ---

def calibrate_magnetometer(data):
    """
    Full ellipsoid fit for magnetometer calibration.

    Fits: Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
    (9 parameters, with cross-coupling terms for soft iron)

    Returns:
        (center, S): Hard iron bias vector (3,) and soft iron correction matrix (3,3)
        None on failure
    """
    if len(data) < 10:
        return None

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

    S = np.dot(vecs, np.dot(D_sqrt, vecs.T))

    return center, S


# --- GUI ---

def main():
    global is_collecting, raw_data_points, calibration_result, last_calibration_time
    global g_received_count, g_last_raw, g_querying_storage

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
    plt.subplots_adjust(bottom=0.22, right=0.75)
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.pane.fill = False
        axis.pane.set_edgecolor(GRID_COLOR)
        axis.label.set_color(TEXT_COLOR)
        axis._axinfo['tick']['color'] = TEXT_COLOR
        axis._axinfo['tick']['inward_factor'] = 0
    ax.tick_params(colors=DIM_TEXT)

    # Initialize plots
    plot_raw, = ax.plot([], [], [], 'ro', markersize=3, label='Raw (uT)')
    plot_last, = ax.plot([], [], [], 'co', markersize=8, label='Latest')
    plot_corr, = ax.plot([], [], [], 'go', markersize=3, label='Corrected')
    live_arrow = [None]
    plot_live_tip, = ax.plot([], [], [], 'yD', markersize=8, label='Live')

    # --- Info Panel ---
    text_ax = plt.axes([0.76, 0.22, 0.23, 0.68])
    text_ax.axis('off')
    text_ax.set_facecolor(PANEL_COLOR)
    instructions = (
        "MAG CALIBRATION\n"
        f"{'=' * 26}\n\n"
        "1. Click 'Start Log'\n"
        "   Yellow arrow = live data\n\n"
        "2. Click 'Stream'\n"
        "   Rotate drone in ALL\n"
        "   directions (figure-8).\n"
        "   Cover the full sphere.\n\n"
        "3. Watch red dots fill sphere,\n"
        "   green dots become round.\n"
        "   Auto-fit runs every 1s.\n\n"
        "4. Click 'Stream' to stop\n\n"
        "5. Click 'Upload'\n"
        "   Saves to FC flash\n\n"
        "6. Click 'Query FC' to verify\n\n"
        "7. Click 'View Cal'\n"
        "   Magnitude should be\n"
        "   ~1.0 in all orientations"
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

    # Row 2: Start Log | View Cal | Stream | Upload | Default | Query FC
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

    ax_stream = plt.axes([start_x + 2 * (w + gap), row2_y, w, btn_h])
    btn_stream = Button(ax_stream, 'Stream', color=BTN_ORANGE, hovercolor=BTN_ORANGE_HOV)
    btn_stream.label.set_color(TEXT_COLOR)
    btn_stream.label.set_fontsize(8)

    ax_upload = plt.axes([start_x + 3 * (w + gap), row2_y, w, btn_h])
    btn_upload = Button(ax_upload, 'Upload', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_upload.label.set_color(TEXT_COLOR)
    btn_upload.label.set_fontsize(8)
    btn_upload.label.set_fontweight('bold')

    ax_default = plt.axes([start_x + 4 * (w + gap), row2_y, w, btn_h])
    btn_default = Button(ax_default, 'Default', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_default.label.set_color(TEXT_COLOR)
    btn_default.label.set_fontsize(8)

    ax_query = plt.axes([start_x + 5 * (w + gap), row2_y, w, btn_h])
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
            send_log_class(g_serial, LOG_CLASS_COMPASS)
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
            send_log_class(g_serial, LOG_CLASS_COMPASS_CALIB)
            g_verify_active = True
            g_logging_active = False
            btn_viewcal.label.set_text('View Raw')
            btn_viewcal.color = BTN_RED
            btn_viewcal.hovercolor = BTN_RED_HOV
            btn_log.label.set_text('Start Log')
            btn_log.color = BTN_GREEN
            btn_log.hovercolor = BTN_GREEN_HOV
    btn_viewcal.on_clicked(toggle_view_cal)

    def toggle_stream(event):
        global is_collecting
        is_collecting = not is_collecting
        btn_stream.label.set_text('Stop' if is_collecting else 'Stream')
        btn_stream.color = BTN_RED if is_collecting else BTN_ORANGE
        btn_stream.hovercolor = BTN_RED_HOV if is_collecting else BTN_ORANGE_HOV
    btn_stream.on_clicked(toggle_stream)

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
            f"{'=' * 26}\n\n"
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
            f"{'=' * 26}\n\n"
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
        while not storage_queue.empty():
            storage_queue.get_nowait()
        storage_pages.clear()
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
            print("  \u2717 No points to save")
            return
        cal_dir = get_cal_dir()
        chip_tag = f'_{g_chip_id}' if g_chip_id else ''
        filename = os.path.join(cal_dir, f'mag_cal{chip_tag}.csv')
        with open(filename, 'w') as f:
            f.write('sample,mag_x_ut,mag_y_ut,mag_z_ut\n')
            for i, pt in enumerate(raw_data_points):
                f.write(f'{i + 1},{pt[0]:.2f},{pt[1]:.2f},{pt[2]:.2f}\n')
        info_text.set_text(
            f"CSV SAVED \u2713\n"
            f"{'=' * 26}\n\n"
            f"Points: {len(raw_data_points)}\n"
            f"File: {os.path.basename(filename)}\n\n"
            f"Saved to .calibration_data/"
        )
        print(f"  \u2713 Saved {len(raw_data_points)} points to {filename}")
        plt.draw()
    btn_savecsv.on_clicked(save_csv)


    def load_csv(event):
        global raw_data_points, calibration_result, last_calibration_time
        cal_dir = get_cal_dir()
        filepath = _open_file_dialog(title='Load calibration CSV', initialdir=cal_dir)
        if not filepath:
            return
        points = []
        try:
            with open(filepath, 'r') as f:
                header = f.readline()
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) >= 4:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        points.append((x, y, z))
        except Exception as e:
            print(f"  \u2717 Failed to load CSV: {e}")
            return
        if not points:
            return

        raw_data_points = list(points)
        calibration_result = (np.zeros(3), np.eye(3))
        last_calibration_time = 0

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
            f"Click 'Stream' to add\n"
            f"more points, or auto-fit\n"
            f"will run on next stream."
        )
        print(f"  \u2713 Loaded {len(points)} points from {filepath}")
        plt.draw()
    btn_loadcsv.on_clicked(load_csv)

    def clear_points(event):
        global raw_data_points, calibration_result, last_calibration_time
        raw_data_points = []
        calibration_result = (np.zeros(3), np.eye(3))
        last_calibration_time = 0
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

        def _post_reset():
            time.sleep(2.0)
            if g_serial and g_serial.is_open:
                g_serial.reset_input_buffer()
                send_log_class(g_serial, LOG_CLASS_HEART_BEAT)

        threading.Thread(target=_post_reset, daemon=True).start()
    btn_reset.on_clicked(reset_fc)

    # --- Axes setup ---
    ax.set_title('Magnetometer Calibration', color=TEXT_COLOR, fontsize=12, pad=10)
    ax.set_xlabel('X (uT)', color=TEXT_COLOR)
    ax.set_ylabel('Y (uT)', color=TEXT_COLOR)
    ax.set_zlabel('Z (uT)', color=TEXT_COLOR)
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

    # --- Chip ID auto-retry ---
    chip_id_request_time = time.time()

    # --- Animation ---
    storage_pages = []
    def update(frame):
        nonlocal chip_id_request_time
        global g_received_count, g_last_raw, g_chip_id
        global is_collecting, raw_data_points, calibration_result, last_calibration_time
        global g_querying_storage

        data_updated = False
        points_added = False

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

            # Update raw plot
            plot_raw.set_data(data_np[:, 0], data_np[:, 1])
            plot_raw.set_3d_properties(data_np[:, 2])

            # Update last point
            if len(data_np) > 0:
                last_pt = data_np[-1]
                plot_last.set_data([last_pt[0]], [last_pt[1]])
                plot_last.set_3d_properties([last_pt[2]])

            # Realtime calibration every 1s with >= 20 points
            if time.time() - last_calibration_time > 1.0 and len(raw_data_points) > 20:
                res = calibrate_magnetometer(data_np)
                if res is not None:
                    B, S = res
                    calibration_result = (B, S)
                last_calibration_time = time.time()

            # Update corrected points display
            B, S = calibration_result
            if not (np.allclose(B, 0) and np.allclose(S, np.eye(3))):
                raw_centered = (data_np - B).T
                corrected = np.dot(S, raw_centered).T
                avg_radius = np.mean(np.linalg.norm(raw_centered, axis=0))
                corrected_scaled = corrected * avg_radius

                plot_corr.set_data(corrected_scaled[:, 0], corrected_scaled[:, 1])
                plot_corr.set_3d_properties(corrected_scaled[:, 2])

                if len(corrected) > 0:
                    last_corr = corrected[-1]
                    B_str = f"[{B[0]:.2f}, {B[1]:.2f}, {B[2]:.2f}]"
                    S_str = "\n".join([f"[{r[0]:.4f}, {r[1]:.4f}, {r[2]:.4f}]" for r in S])
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
            elif is_collecting:
                info_text.set_text(
                    f"STREAMING ({len(raw_data_points)} pts)\n"
                    f"{'=' * 26}\n\n"
                    f"Collecting data...\n"
                    f"Need 20+ points for\n"
                    f"auto-fit to start.\n\n"
                    f"Rotate in ALL directions."
                )

        # Update live arrow and status when new data arrived
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

        # Check for storage readback (Query FC) — 4 pages of 26 floats each
        if g_querying_storage:
            while not storage_queue.empty():
                page = storage_queue.get()
                storage_pages.append(page)

            if len(storage_pages) >= 2:  # mag params 17-29 span page 0 and 1
                all_params = list(storage_pages[0]) + list(storage_pages[1])

                g_querying_storage = False
                storage_pages.clear()
                if g_serial and g_serial.is_open:
                    send_log_class(g_serial, LOG_CLASS_NONE)

                stored_cal = all_params[PARAM_MAG_CAL_FLAG] if len(all_params) > PARAM_MAG_CAL_FLAG else 0.0
                if stored_cal > 0.0:
                    stored_bias = [all_params[PARAM_MAG_BIAS_BASE + i] for i in range(3)]
                    stored_scale = [[all_params[PARAM_MAG_SCALE_BASE + r * 3 + c] for c in range(3)] for r in range(3)]
                    info_text.set_text(
                        f"FLASH VERIFIED \u2705\n"
                        f"{'=' * 26}\n\n"
                        f"Mag Calibrated: YES\n\n"
                        f"Stored Bias (uT):\n"
                        f"[{stored_bias[0]:.2f},\n"
                        f" {stored_bias[1]:.2f},\n"
                        f" {stored_bias[2]:.2f}]\n\n"
                        f"Stored Scale:\n" +
                        "\n".join([f"[{r[0]:.4f}, {r[1]:.4f},\n {r[2]:.4f}]" for r in stored_scale]) +
                        f"\n\nActive immediately.\n"
                        f"Click 'View Cal' to\n"
                        f"check magnitude."
                    )
                else:
                    info_text.set_text(
                        f"NOT CALIBRATED \u274c\n"
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
