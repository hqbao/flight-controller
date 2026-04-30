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
import csv
import os
import subprocess


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
Gyroscope Temperature Calibration Tool — Flight Controller

Calibrates gyro temperature polynomial per axis using raw LSB data.

Calibration model:
  bias(T) = a·T² + b·T + c   (per axis, in raw LSB)
  V_cal = V_raw - bias(T)

Workflow:
  1. Click "Start Log" — raw gyro + temp streams at 25 Hz
  2. Keep drone perfectly still, let it warm up naturally
  3. Click "Record" to start capturing temperature sweep data
  4. Wait for ≥3°C temperature range, click "Stop Rec"
  5. Click "Compute" — fits degree-2 polynomial per axis
  6. Click "Upload" — sends 9 coefficients to flash
  7. Click "Query FC" to verify stored coefficients

  Alternatively: save CSV, collect data over time, load CSV later.

Usage:
  python3 calibration_gyro.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 38400
SEND_LOG_ID = 0x00
HISTORY_LEN = 200
WINDOW_SIZE = 50           # Samples per averaging window
MOTION_THRESHOLD = 32.0    # Max spread in LSB to count as stationary
MIN_TEMP_RANGE = 3.0       # Minimum temperature range for valid fit
MIN_POINTS = 10            # Minimum stationary windows for valid fit

# Log class constants (match messages.h)
LOG_CLASS_NONE           = 0x00
LOG_CLASS_HEART_BEAT     = 0x09
LOG_CLASS_IMU_GYRO_RAW   = 0x0B
LOG_CLASS_IMU_GYRO_CALIB = 0x0C
LOG_CLASS_STORAGE        = 0x10
DB_CMD_LOG_CLASS         = 0x03
DB_CMD_CALIBRATE_GYRO_TEMP = 0x08
DB_CMD_RESET             = 0x07
DB_CMD_CHIP_ID           = 0x09

# FC storage layout: 104 params across 4 pages (26 params × 4 bytes = 104 bytes each)
# Gyro temp params: ID 30=flag, 31-39=coefficients (Xa,Xb,Xc, Ya,Yb,Yc, Za,Zb,Zc)
STORAGE_PAGE_SIZE  = 104   # 26 floats per page
STORAGE_TOTAL_PAGES = 4
PARAM_GYRO_TEMP_FLAG = 30
PARAM_GYRO_TEMP_BASE = 31

# --- UI Colors ---
BG_COLOR       = '#1e1e1e'
PANEL_COLOR    = '#252526'
TEXT_COLOR     = '#cccccc'
DIM_TEXT       = '#888888'
GRID_COLOR     = '#3c3c3c'
BTN_GREEN      = '#2d5a2d'
BTN_GREEN_HOV  = '#3d7a3d'
BTN_RED        = '#5a2d2d'
BTN_RED_HOV    = '#7a3d3d'
BTN_ORANGE     = '#5a4a2d'
BTN_ORANGE_HOV = '#7a6a3d'
BTN_COLOR      = '#333333'
BTN_HOVER      = '#444444'
BTN_ACTIVE     = '#4488cc'
BTN_ACTIVE_HOV = '#5599dd'
COLOR_X        = '#ff5555'
COLOR_Y        = '#55dd55'
COLOR_Z        = '#5599ff'
COLOR_TEMP     = '#ffaa55'

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
heartbeat_queue = queue.Queue()
storage_queue = queue.Queue()
g_serial = None
g_logging_active = False
g_chip_id = None
g_querying_storage = False

# Recording state
g_recording = False
g_record_buf = []
g_cal_coeffs = None        # [3][3] array: [axis][a, b, c]
g_stationary_points = []   # Windowed averages: (temp, gx, gy, gz)
g_cal_done = False
g_view_cal = False         # False = raw (LSB), True = calibrated (deg/s)


# --- DB Frame Helpers (FC protocol) ---


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
    """Build a DB frame using FC protocol: 'db' + msg_id + msg_class(0x00) + length + payload + checksum."""
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


def send_reset(ser):
    ser.write(build_db_frame(DB_CMD_RESET, bytes([0x00])))
    ser.flush()


def send_chip_id_request(ser):
    ser.write(build_db_frame(DB_CMD_CHIP_ID, bytes([0x00])))
    ser.flush()


def send_gyro_calibration(ser, coeffs):
    """Send 9 temperature polynomial coefficients: [Xa,Xb,Xc, Ya,Yb,Yc, Za,Zb,Zc]"""
    values = []
    for axis in range(3):
        for c in range(3):
            values.append(coeffs[axis][c])
    payload = struct.pack('<9f', *values)
    ser.write(build_db_frame(DB_CMD_CALIBRATE_GYRO_TEMP, payload))
    ser.flush()


def get_gyro_log_class():
    return LOG_CLASS_IMU_GYRO_CALIB if g_view_cal else LOG_CLASS_IMU_GYRO_RAW


# --- Serial Reader Thread ---

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
            time.sleep(0.1)
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

                # Chip ID: 8 bytes
                if msg_id == SEND_LOG_ID and length == 8:
                    heartbeat_queue.put(('chip_id', payload.hex().upper()))

                # Gyro data: 4 floats (gx, gy, gz, temp) = 16 bytes
                elif msg_id == SEND_LOG_ID and length == 16:
                    data_queue.put(struct.unpack('<4f', payload))

                # Legacy: 3 floats (12 bytes) — no temperature
                elif msg_id == SEND_LOG_ID and length == 12:
                    data_queue.put(struct.unpack('<3f', payload) + (None,))

                # Storage page: 26 floats = 104 bytes (4 pages sent sequentially)
                elif msg_id == SEND_LOG_ID and length == STORAGE_PAGE_SIZE:
                    storage_queue.put(payload)

    except Exception as e:
        print(f"Serial error: {e}")


# --- GUI ---
def main():
    global g_logging_active, g_chip_id, g_querying_storage
    global g_recording, g_record_buf, g_cal_coeffs, g_stationary_points, g_cal_done

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

    fig = plt.figure(figsize=screen_fit_figsize(15, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('Gyroscope Temperature Calibration', fontsize=14, color=TEXT_COLOR)
    plt.subplots_adjust(left=0.07, right=0.70, bottom=0.22, top=0.92, hspace=0.35)

    # --- Gyro history chart ---
    ax_gyro = fig.add_subplot(2, 1, 1)
    ax_gyro.set_title('Raw Gyro (LSB)', fontsize=11, color=TEXT_COLOR)
    ax_gyro.set_ylabel('Raw LSB', fontsize=9)
    ax_gyro.grid(True, alpha=0.3)
    line_gx, = ax_gyro.plot([], [], color=COLOR_X, linewidth=1.2, label='X')
    line_gy, = ax_gyro.plot([], [], color=COLOR_Y, linewidth=1.2, label='Y')
    line_gz, = ax_gyro.plot([], [], color=COLOR_Z, linewidth=1.2, label='Z')
    ax_gyro.legend(loc='upper left', fontsize=8,
                   facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)
    ax_gyro.set_ylim(-100, 100)

    # --- Temperature vs bias chart ---
    ax_temp = fig.add_subplot(2, 1, 2)
    ax_temp.set_title('Gyro Bias vs Temperature', fontsize=11, color=TEXT_COLOR)
    ax_temp.set_ylabel('Bias (raw LSB)', fontsize=9)
    ax_temp.set_xlabel('Temperature (\u00b0C)', fontsize=9)
    ax_temp.grid(True, alpha=0.3)

    # --- Right-side status panel ---
    ax_info = fig.add_axes([0.72, 0.22, 0.26, 0.70])
    ax_info.set_facecolor(PANEL_COLOR)
    ax_info.set_xlim(0, 1)
    ax_info.set_ylim(0, 1)
    ax_info.axis('off')

    info_chip = ax_info.text(0.05, 0.97, 'Chip ID: --', fontsize=9, color=DIM_TEXT,
                             va='top', family='monospace')

    ax_info.plot([0.05, 0.95], [0.93, 0.93], color=GRID_COLOR, linewidth=0.5)

    # Recording info
    ax_info.text(0.05, 0.89, 'Recording:', fontsize=10, color=TEXT_COLOR,
                 va='top', fontweight='bold')
    rec_status_text = ax_info.text(0.45, 0.89, 'Idle', fontsize=10, color=DIM_TEXT,
                                   va='top', family='monospace')
    rec_points_text = ax_info.text(0.08, 0.84, 'Points: 0', fontsize=9, color=DIM_TEXT,
                                   va='top', family='monospace')
    rec_range_text = ax_info.text(0.08, 0.80, 'Temp range: --', fontsize=9, color=DIM_TEXT,
                                  va='top', family='monospace')

    ax_info.plot([0.05, 0.95], [0.76, 0.76], color=GRID_COLOR, linewidth=0.5)

    # Coefficients display
    ax_info.text(0.05, 0.72, 'Coefficients (a\u00b7T\u00b2+b\u00b7T+c):', fontsize=10,
                 color=TEXT_COLOR, va='top', fontweight='bold')
    coeff_x_text = ax_info.text(0.08, 0.67, 'X: \u2014', fontsize=9, color=COLOR_X,
                                va='top', family='monospace')
    coeff_y_text = ax_info.text(0.08, 0.62, 'Y: \u2014', fontsize=9, color=COLOR_Y,
                                va='top', family='monospace')
    coeff_z_text = ax_info.text(0.08, 0.57, 'Z: \u2014', fontsize=9, color=COLOR_Z,
                                va='top', family='monospace')
    r2_text = ax_info.text(0.08, 0.52, 'R\u00b2: \u2014', fontsize=9, color=DIM_TEXT,
                           va='top', family='monospace')

    ax_info.plot([0.05, 0.95], [0.48, 0.48], color=GRID_COLOR, linewidth=0.5)

    # Live values
    ax_info.text(0.05, 0.44, 'Live:', fontsize=10, color=TEXT_COLOR, va='top', fontweight='bold')
    cur_x_text = ax_info.text(0.08, 0.39, 'X:  \u2014', fontsize=9, color=DIM_TEXT,
                              va='top', family='monospace')
    cur_y_text = ax_info.text(0.08, 0.35, 'Y:  \u2014', fontsize=9, color=DIM_TEXT,
                              va='top', family='monospace')
    cur_z_text = ax_info.text(0.08, 0.31, 'Z:  \u2014', fontsize=9, color=DIM_TEXT,
                              va='top', family='monospace')
    cur_t_text = ax_info.text(0.08, 0.27, 'T:  \u2014', fontsize=9, color=COLOR_TEMP,
                              va='top', family='monospace')

    status_text = ax_info.text(0.05, 0.16, '', fontsize=9, color='#55cc55',
                               va='top', family='monospace')

    # --- Button row ---
    btn_h = 0.04
    btn_w = 0.07
    row1_y = 0.12
    row2_y = 0.065

    # Row 1: Start Log | Record | Compute | Upload | Query FC | Save CSV | Load CSV | Reset FC
    ax_toggle = fig.add_axes([0.04, row1_y, btn_w, btn_h])
    btn_toggle = Button(ax_toggle, 'Start Log', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_toggle.label.set_color(TEXT_COLOR)

    ax_rec = fig.add_axes([0.12, row1_y, btn_w, btn_h])
    btn_rec = Button(ax_rec, 'Record', color=BTN_ORANGE, hovercolor=BTN_ORANGE_HOV)
    btn_rec.label.set_color(TEXT_COLOR)

    ax_compute = fig.add_axes([0.20, row1_y, btn_w, btn_h])
    btn_compute = Button(ax_compute, 'Compute', color=BTN_ORANGE, hovercolor=BTN_ORANGE_HOV)
    btn_compute.label.set_color(TEXT_COLOR)

    ax_upload = fig.add_axes([0.28, row1_y, 0.09, btn_h])
    btn_upload = Button(ax_upload, 'Upload', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_upload.label.set_color(TEXT_COLOR)
    btn_upload.label.set_fontweight('bold')

    ax_query = fig.add_axes([0.38, row1_y, 0.07, btn_h])
    btn_query = Button(ax_query, 'Query FC', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_query.label.set_color(TEXT_COLOR)

    ax_savecsv = fig.add_axes([0.46, row1_y, 0.08, btn_h])
    btn_savecsv = Button(ax_savecsv, 'Save CSV', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_savecsv.label.set_color(TEXT_COLOR)

    ax_loadcsv = fig.add_axes([0.55, row1_y, 0.08, btn_h])
    btn_loadcsv = Button(ax_loadcsv, 'Load CSV', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_loadcsv.label.set_color(TEXT_COLOR)

    ax_reset = fig.add_axes([0.64, row1_y, 0.07, btn_h])
    btn_reset = Button(ax_reset, 'Reset FC', color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_reset.label.set_color(TEXT_COLOR)

    # Row 2: View Cal/Raw | Clear
    ax_viewcal = fig.add_axes([0.04, row2_y, 0.09, btn_h])
    btn_viewcal = Button(ax_viewcal, 'View: Cal', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_viewcal.label.set_color(TEXT_COLOR)

    ax_clear = fig.add_axes([0.14, row2_y, 0.07, btn_h])
    btn_clear = Button(ax_clear, 'Clear', color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_clear.label.set_color(TEXT_COLOR)

    # --- History buffers ---
    hist_gx = np.zeros(HISTORY_LEN)
    hist_gy = np.zeros(HISTORY_LEN)
    hist_gz = np.zeros(HISTORY_LEN)

    # --- Scroll-to-Zoom ---
    def on_scroll(event):
        if event.inaxes not in (ax_gyro, ax_temp):
            return
        scale = 0.8 if event.button == 'up' else 1.25
        ylo, yhi = event.inaxes.get_ylim()
        yc = event.ydata
        event.inaxes.set_ylim(yc - (yc - ylo) * scale, yc + (yhi - yc) * scale)

    fig.canvas.mpl_connect('scroll_event', on_scroll)

    # --- Window accumulator for recording ---
    window_buf = []

    def reset_cal_state():
        global g_cal_coeffs, g_cal_done, g_stationary_points, g_record_buf, g_recording
        g_cal_coeffs = None
        g_cal_done = False
        g_stationary_points = []
        g_record_buf = []
        g_recording = False
        window_buf.clear()
        coeff_x_text.set_text('X: \u2014')
        coeff_y_text.set_text('Y: \u2014')
        coeff_z_text.set_text('Z: \u2014')
        r2_text.set_text('R\u00b2: \u2014')
        rec_status_text.set_text('Idle')
        rec_status_text.set_color(DIM_TEXT)
        rec_points_text.set_text('Points: 0')
        rec_range_text.set_text('Temp range: --')
        btn_rec.label.set_text('Record')
        ax_rec.set_facecolor(BTN_ORANGE)
        ax_temp.clear()
        ax_temp.set_title('Gyro Bias vs Temperature', fontsize=11, color=TEXT_COLOR)
        ax_temp.set_ylabel('Bias (raw LSB)', fontsize=9)
        ax_temp.set_xlabel('Temperature (\u00b0C)', fontsize=9)
        ax_temp.grid(True, alpha=0.3)

    # --- Button callbacks ---
    def on_toggle(event):
        global g_logging_active
        if not g_serial or not g_serial.is_open:
            return
        if g_logging_active:
            send_log_class(g_serial, LOG_CLASS_NONE)
            g_logging_active = False
            btn_toggle.label.set_text('Start Log')
            ax_toggle.set_facecolor(BTN_GREEN)
        else:
            hist_gx[:] = 0; hist_gy[:] = 0; hist_gz[:] = 0
            while not data_queue.empty(): data_queue.get_nowait()
            send_log_class(g_serial, get_gyro_log_class())
            g_logging_active = True
            btn_toggle.label.set_text('Stop Log')
            ax_toggle.set_facecolor(BTN_RED)

    def on_record(event):
        global g_recording
        if g_recording:
            g_recording = False
            btn_rec.label.set_text('Record')
            ax_rec.set_facecolor(BTN_ORANGE)
            rec_status_text.set_text('Stopped')
            rec_status_text.set_color(DIM_TEXT)
            n = len(g_stationary_points)
            if n > 0:
                temps = [p[0] for p in g_stationary_points]
                t_range = max(temps) - min(temps)
                rec_range_text.set_text(f'Temp range: {t_range:.1f}\u00b0C')
            print(f'  Recording stopped: {n} stationary points')
        else:
            if not g_logging_active:
                status_text.set_text('Start log first!')
                status_text.set_color(COLOR_X)
                return
            g_recording = True
            window_buf.clear()
            btn_rec.label.set_text('Stop Rec')
            ax_rec.set_facecolor(BTN_RED)
            rec_status_text.set_text('Recording...')
            rec_status_text.set_color('#ff5555')
            status_text.set_text('Keep still, let board warm up')
            status_text.set_color(DIM_TEXT)
            print(f'  Recording started')

    def on_compute(event):
        global g_cal_coeffs, g_cal_done
        n = len(g_stationary_points)
        if n < MIN_POINTS:
            status_text.set_text(f'Need \u2265{MIN_POINTS} points ({n} so far)')
            status_text.set_color(COLOR_X)
            return
        pts = np.array(g_stationary_points)
        temps = pts[:, 0]
        t_range = temps.max() - temps.min()
        low_range = t_range < MIN_TEMP_RANGE

        coeffs = []
        r2_vals = []
        for axis in range(3):
            bias = pts[:, 1 + axis]
            p = np.polyfit(temps, bias, 2)
            coeffs.append(list(p))
            predicted = np.polyval(p, temps)
            ss_res = np.sum((bias - predicted) ** 2)
            ss_tot = np.sum((bias - np.mean(bias)) ** 2)
            r2 = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0
            r2_vals.append(r2)

        g_cal_coeffs = coeffs
        g_cal_done = True

        coeff_x_text.set_text(f'X: {coeffs[0][0]:.4f} {coeffs[0][1]:+.2f} {coeffs[0][2]:+.1f}')
        coeff_y_text.set_text(f'Y: {coeffs[1][0]:.4f} {coeffs[1][1]:+.2f} {coeffs[1][2]:+.1f}')
        coeff_z_text.set_text(f'Z: {coeffs[2][0]:.4f} {coeffs[2][1]:+.2f} {coeffs[2][2]:+.1f}')
        r2_text.set_text(f'R\u00b2: {r2_vals[0]:.3f} {r2_vals[1]:.3f} {r2_vals[2]:.3f}')
        if low_range:
            status_text.set_text(f'\u26a0 Low temp range ({t_range:.1f}\u00b0C) \u2014 cal may be inaccurate')
            status_text.set_color(COLOR_TEMP)
        else:
            status_text.set_text(f'\u2713 Polynomial fit ({n} pts, {t_range:.1f}\u00b0C)')
            status_text.set_color('#55cc55')

        # Plot fit curves on bottom chart
        ax_temp.clear()
        ax_temp.set_title('Gyro Bias vs Temperature', fontsize=11, color=TEXT_COLOR)
        ax_temp.set_ylabel('Bias (raw LSB)', fontsize=9)
        ax_temp.set_xlabel('Temperature (\u00b0C)', fontsize=9)
        ax_temp.grid(True, alpha=0.3)
        ax_temp.scatter(temps, pts[:, 1], c=COLOR_X, s=15, alpha=0.6, label='X')
        ax_temp.scatter(temps, pts[:, 2], c=COLOR_Y, s=15, alpha=0.6, label='Y')
        ax_temp.scatter(temps, pts[:, 3], c=COLOR_Z, s=15, alpha=0.6, label='Z')
        t_fit = np.linspace(temps.min(), temps.max(), 100)
        for axis, color in zip(range(3), [COLOR_X, COLOR_Y, COLOR_Z]):
            y_fit = np.polyval(coeffs[axis], t_fit)
            ax_temp.plot(t_fit, y_fit, color=color, linewidth=2, alpha=0.8)
        ax_temp.legend(fontsize=8, facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

        print(f'  Polynomial fit complete:')
        for i, label in enumerate(['X', 'Y', 'Z']):
            print(f'    {label}: {coeffs[i][0]:.6f}*T^2 + {coeffs[i][1]:.4f}*T + {coeffs[i][2]:.2f}  (R\u00b2={r2_vals[i]:.4f})')

    def on_upload(event):
        if not g_cal_done or g_cal_coeffs is None:
            status_text.set_text('Compute first!')
            status_text.set_color(COLOR_X)
            return
        if not g_serial or not g_serial.is_open:
            return
        send_gyro_calibration(g_serial, g_cal_coeffs)
        status_text.set_text('\u2713 Uploaded to FC')
        status_text.set_color('#55cc55')
        print(f'  Uploaded 9 coefficients to FC')

    def on_query(event):
        global g_querying_storage
        if not g_serial or not g_serial.is_open:
            return
        while not storage_queue.empty(): storage_queue.get_nowait()
        storage_pages.clear()
        g_querying_storage = True
        send_log_class(g_serial, LOG_CLASS_STORAGE)
        status_text.set_text('Querying FC...')
        status_text.set_color(DIM_TEXT)

    def get_cal_dir():
        cal_dir = os.path.join(os.path.dirname(__file__), '.calibration_data')
        os.makedirs(cal_dir, exist_ok=True)
        return cal_dir

    def on_save_csv(event):
        if len(g_stationary_points) == 0:
            status_text.set_text('No data to save')
            status_text.set_color(COLOR_X)
            return
        chip_tag = f'_{g_chip_id}' if g_chip_id else ''
        fname = f'gyro_temp{chip_tag}.csv'
        fpath = os.path.join(get_cal_dir(), fname)
        with open(fpath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['temp', 'gx', 'gy', 'gz'])
            for pt in g_stationary_points:
                writer.writerow([f'{pt[0]:.3f}', f'{pt[1]:.3f}', f'{pt[2]:.3f}', f'{pt[3]:.3f}'])
        status_text.set_text(f'Saved {fname}')
        status_text.set_color('#55cc55')
        print(f'  Saved {len(g_stationary_points)} points to {fpath}')

    # --- Queue for async file picker result ---
    load_csv_queue = queue.Queue()

    def on_load_csv(event):
        cal_dir = get_cal_dir()
        fpath = _open_file_dialog(title='Load calibration CSV', initialdir=cal_dir)
        load_csv_queue.put(fpath)

    def on_reset(event):
        global g_logging_active, g_recording
        if not g_serial or not g_serial.is_open:
            return
        g_logging_active = False
        g_recording = False
        btn_toggle.label.set_text('Start Log')
        ax_toggle.set_facecolor(BTN_GREEN)
        btn_rec.label.set_text('Record')
        ax_rec.set_facecolor(BTN_ORANGE)
        send_reset(g_serial)
        status_text.set_text('Reset sent')
        status_text.set_color(DIM_TEXT)

        def _post_reset():
            time.sleep(2.0)
            if g_serial and g_serial.is_open:
                g_serial.reset_input_buffer()
                send_log_class(g_serial, LOG_CLASS_HEART_BEAT)

        threading.Thread(target=_post_reset, daemon=True).start()

    def on_viewcal(event):
        global g_view_cal
        g_view_cal = not g_view_cal
        if g_view_cal:
            btn_viewcal.label.set_text('View: Raw')
            ax_viewcal.set_facecolor(BTN_ACTIVE)
            btn_viewcal.color = BTN_ACTIVE
            btn_viewcal.hovercolor = BTN_ACTIVE_HOV
            ax_gyro.set_title('Calibrated Gyro (deg/s)', fontsize=11, color=TEXT_COLOR)
            ax_gyro.set_ylabel('deg/s', fontsize=9)
        else:
            btn_viewcal.label.set_text('View: Cal')
            ax_viewcal.set_facecolor(BTN_COLOR)
            btn_viewcal.color = BTN_COLOR
            btn_viewcal.hovercolor = BTN_HOVER
            ax_gyro.set_title('Raw Gyro (LSB)', fontsize=11, color=TEXT_COLOR)
            ax_gyro.set_ylabel('Raw LSB', fontsize=9)
        hist_gx[:] = 0; hist_gy[:] = 0; hist_gz[:] = 0
        while not data_queue.empty(): data_queue.get_nowait()
        if g_logging_active and g_serial and g_serial.is_open:
            send_log_class(g_serial, get_gyro_log_class())

    def on_clear(event):
        hist_gx[:] = 0; hist_gy[:] = 0; hist_gz[:] = 0
        while not data_queue.empty(): data_queue.get_nowait()
        reset_cal_state()
        status_text.set_text('Cleared')
        status_text.set_color(DIM_TEXT)
        print("  Cleared")

    btn_toggle.on_clicked(on_toggle)
    btn_rec.on_clicked(on_record)
    btn_compute.on_clicked(on_compute)
    btn_upload.on_clicked(on_upload)
    btn_query.on_clicked(on_query)
    btn_savecsv.on_clicked(on_save_csv)
    btn_loadcsv.on_clicked(on_load_csv)
    btn_reset.on_clicked(on_reset)
    btn_viewcal.on_clicked(on_viewcal)
    btn_clear.on_clicked(on_clear)

    # --- Animation ---
    storage_pages = []

    def update(frame_num):
        global g_chip_id, g_cal_coeffs, g_cal_done, g_querying_storage
        global g_stationary_points
        latest = None

        # Process async file picker result
        while not load_csv_queue.empty():
            try:
                fpath = load_csv_queue.get_nowait()
            except queue.Empty:
                break
            if fpath is None:
                status_text.set_text('')
                continue
            loaded = []
            try:
                with open(fpath, 'r') as f:
                    reader = csv.reader(f)
                    next(reader, None)
                    for row in reader:
                        if len(row) >= 4:
                            loaded.append((float(row[0]), float(row[1]),
                                           float(row[2]), float(row[3])))
            except Exception as e:
                status_text.set_text(f'Error: {e}')
                status_text.set_color(COLOR_X)
                continue
            if len(loaded) == 0:
                status_text.set_text('No data in CSV')
                status_text.set_color(COLOR_X)
                continue
            g_stationary_points = loaded
            n = len(loaded)
            temps = [p[0] for p in loaded]
            t_range = max(temps) - min(temps)
            rec_points_text.set_text(f'Points: {n}')
            rec_range_text.set_text(f'Temp range: {t_range:.1f}\u00b0C')
            rec_status_text.set_text('Loaded from CSV')
            rec_status_text.set_color('#55cc55')
            fname = os.path.basename(fpath)
            status_text.set_text(f'\u2713 Loaded {n} pts from {fname}')
            status_text.set_color('#55cc55')
            print(f'  Loaded {n} points from {fpath}')

        # Process storage readback (4 pages of 26 floats each, sent sequentially)
        while not storage_queue.empty():
            try:
                payload = storage_queue.get_nowait()
            except queue.Empty:
                break
            storage_pages.append(struct.unpack('<26f', payload))

        if g_querying_storage and len(storage_pages) >= STORAGE_TOTAL_PAGES:
            all_params = []
            for page in storage_pages[:STORAGE_TOTAL_PAGES]:
                all_params.extend(page)

            cal_flag = all_params[PARAM_GYRO_TEMP_FLAG] if len(all_params) > PARAM_GYRO_TEMP_FLAG else 0.0
            if cal_flag > 0:
                coeffs = []
                for axis in range(3):
                    a = all_params[PARAM_GYRO_TEMP_BASE + axis * 3]
                    b = all_params[PARAM_GYRO_TEMP_BASE + axis * 3 + 1]
                    c = all_params[PARAM_GYRO_TEMP_BASE + axis * 3 + 2]
                    coeffs.append([a, b, c])
                g_cal_coeffs = coeffs
                g_cal_done = True
                coeff_x_text.set_text(f'X: {coeffs[0][0]:.4f} {coeffs[0][1]:+.2f} {coeffs[0][2]:+.1f}')
                coeff_y_text.set_text(f'Y: {coeffs[1][0]:.4f} {coeffs[1][1]:+.2f} {coeffs[1][2]:+.1f}')
                coeff_z_text.set_text(f'Z: {coeffs[2][0]:.4f} {coeffs[2][1]:+.2f} {coeffs[2][2]:+.1f}')
                status_text.set_text('\u2713 Loaded from flash')
                status_text.set_color('#55cc55')
                print(f'  Loaded coefficients from flash')
            else:
                status_text.set_text('Not calibrated in flash')
                status_text.set_color(COLOR_TEMP)

            storage_pages.clear()
            g_querying_storage = False
            if g_logging_active and g_serial and g_serial.is_open:
                send_log_class(g_serial, get_gyro_log_class())

        # Process gyro data
        while not data_queue.empty():
            try:
                sample = data_queue.get_nowait()
            except queue.Empty:
                break
            gx, gy, gz = sample[0], sample[1], sample[2]
            temp = sample[3] if len(sample) > 3 and sample[3] is not None else 0
            hist_gx[:-1] = hist_gx[1:]; hist_gx[-1] = gx
            hist_gy[:-1] = hist_gy[1:]; hist_gy[-1] = gy
            hist_gz[:-1] = hist_gz[1:]; hist_gz[-1] = gz
            latest = sample

            # Recording: accumulate into windows
            if g_recording:
                window_buf.append((gx, gy, gz, temp))
                if len(window_buf) >= WINDOW_SIZE:
                    arr = np.array(window_buf)
                    spreads = arr[:, :3].max(axis=0) - arr[:, :3].min(axis=0)
                    if all(s < MOTION_THRESHOLD for s in spreads):
                        avg = arr.mean(axis=0)
                        g_stationary_points.append((avg[3], avg[0], avg[1], avg[2]))
                    window_buf.clear()

        # Heartbeat / chip ID
        while not heartbeat_queue.empty():
            try:
                msg_type, value = heartbeat_queue.get_nowait()
            except queue.Empty:
                break
            if msg_type == 'chip_id':
                g_chip_id = value

        if g_chip_id:
            info_chip.set_text(f'Chip ID: {g_chip_id[:8]}')

        # Retry chip ID request if no response
        if g_chip_id is None and g_serial and g_serial.is_open:
            if not hasattr(update, '_chip_retry'):
                update._chip_retry = time.time()
            now = time.time()
            if now - update._chip_retry > 2.0:
                update._chip_retry = now
                send_chip_id_request(g_serial)

        # Update recording stats
        n_pts = len(g_stationary_points)
        rec_points_text.set_text(f'Points: {n_pts}')
        if n_pts > 0:
            temps = [p[0] for p in g_stationary_points]
            t_range = max(temps) - min(temps)
            rec_range_text.set_text(f'Temp range: {t_range:.1f}\u00b0C')

        # Update charts
        xs = np.arange(HISTORY_LEN)
        line_gx.set_data(xs, hist_gx)
        line_gy.set_data(xs, hist_gy)
        line_gz.set_data(xs, hist_gz)
        ax_gyro.set_xlim(0, HISTORY_LEN)

        if latest:
            cur_x_text.set_text(f'X: {latest[0]:+8.1f}')
            cur_y_text.set_text(f'Y: {latest[1]:+8.1f}')
            cur_z_text.set_text(f'Z: {latest[2]:+8.1f}')
            temp_val = latest[3] if len(latest) > 3 and latest[3] is not None else 0
            cur_t_text.set_text(f'T: {temp_val:6.1f}\u00b0C')

        return (line_gx, line_gy, line_gz)

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
