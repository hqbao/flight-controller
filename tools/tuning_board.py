#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import struct
import threading
import queue
import sys
import matplotlib
matplotlib.use('macosx' if sys.platform == 'darwin' else 'TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
import time

"""
Flight Controller Tuning Board

GUI tool for real-time parameter tuning. Displays 56 tuning parameters
organized by category.  Query flash storage, edit individual parameters,
upload changes, or reset all to compiled defaults.

Workflow:
  1. Click 'Query FC' to read flash storage.
  2. Click a parameter row to select it.
  3. Type a new value and press Enter or click 'Upload'.
  4. Alternatively, type 'ID VALUE' to upload without row selection.

Usage:
  python3 tuning_board.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 19200
SEND_LOG_ID = 0x00

LOG_CLASS_HEART_BEAT = 0x09
LOG_CLASS_STORAGE    = 0x10
DB_CMD_LOG_CLASS     = 0x03
DB_CMD_RESET         = 0x07
DB_CMD_TUNING        = 0x0A

PARAMS_PER_PAGE = 26
TOTAL_PARAMS    = 128
TOTAL_PAGES     = 5
TUNING_FIRST    = 48
TUNING_LAST     = 118
TUNING_COUNT    = 71  # 48..118 contiguous

# --- Tuning Parameters: (param_id, field_name, display_name, default_value) ---
TUNING_PARAMS = [
    # --- Attitude PID (IDs 48-69) ---
    (48,  'att_roll_p',         'Roll P',              4.0),
    (49,  'att_roll_i',         'Roll I',              1.0),
    (50,  'att_roll_d',         'Roll D',              2.0),
    (51,  'att_roll_i_limit',   'Roll I Limit',        5.0),
    (52,  'att_roll_p_limit',   'Roll P-Term Limit',   1000000.0),
    (53,  'att_roll_o_limit',   'Roll Output Limit',   1000000.0),
    (54,  'att_pitch_p',        'Pitch P',             4.0),
    (55,  'att_pitch_i',        'Pitch I',             1.0),
    (56,  'att_pitch_d',        'Pitch D',             2.0),
    (57,  'att_pitch_i_limit',  'Pitch I Limit',       5.0),
    (58,  'att_pitch_p_limit',  'Pitch P-Term Limit',  1000000.0),
    (59,  'att_pitch_o_limit',  'Pitch Output Limit',  1000000.0),
    (60,  'att_yaw_p',          'Yaw P',              10.0),
    (61,  'att_yaw_i',          'Yaw I',               1.0),
    (62,  'att_yaw_d',          'Yaw D',               5.0),
    (63,  'att_yaw_i_limit',    'Yaw I Limit',         5.0),
    (64,  'att_yaw_p_limit',    'Yaw P-Term Limit',    1000000.0),
    (65,  'att_yaw_o_limit',    'Yaw Output Limit',    1000000.0),
    (66,  'att_smooth_input',   'Smooth Input',        1.0),
    (67,  'att_smooth_p_term',  'Smooth P Term',       1.0),
    (68,  'att_smooth_output',  'Smooth Output',       1.0),
    (69,  'att_gain_time',      'Gain Ramp Time',      1.0),
    (70,  'pos_xy_p',           'XY P Gain',         100.0),
    (71,  'pos_z_p',            'Z P Gain',         2000.0),
    (72,  'pos_veloc_xy_s',     'XY Vel Scale',       50.0),
    (73,  'pos_veloc_z_s',      'Z Vel Scale',      2000.0),
    (74,  'pos_lpf_xy',         'XY LPF Alpha',        1.0),
    (75,  'pos_lpf_z',          'Z LPF Alpha',         5.0),
    (76,  'pos_angle_limit',    'Angle Limit (deg)',  30.0),
    (77,  'pos_rc_deadband',    'RC Deadband',         0.1),
    # --- Motor / Servo (IDs 78-82) ---
    (78,  'motor_min',          'Motor Min',         150.0),
    (79,  'motor_max',          'Motor Max',        1800.0),
    (80,  'servo_min',          'Servo Min',        1000.0),
    (81,  'servo_max',          'Servo Max',        2000.0),
    (82,  'servo_center',       'Servo Center',     1500.0),
    # --- Attitude Estimation (IDs 83-90) ---
    (83,  'att_mahony_kp',      'Mahony Kp',           0.1),
    (84,  'att_mahony_ki',      'Mahony Ki',         0.001),
    (85,  'att_f3_beta',        'Madgwick Beta',     0.001),
    (86,  'att_f3_zeta',        'Madgwick Zeta',    0.0001),
    (87,  'att_accel_smooth',   'Accel Smooth',        4.0),
    (88,  'att_lin_acc_decay',  'LinAcc Decay',        0.5),
    (89,  'att_lin_accel_min',  'LinAcc Min',          0.5),
    (90,  'att_lin_accel_max',  'LinAcc Max',          2.0),
    # --- Position Estimation (IDs 91-101) ---
    (91,  'pe_xy_s1_integ',     'XY S1 Integ',        1.0),
    (92,  'pe_xy_s1_corr',      'XY S1 Corr',        1.25),
    (93,  'pe_xy_s2_integ',     'XY S2 Integ',        1.0),
    (94,  'pe_xy_s2_corr',      'XY S2 Corr',        10.0),
    (95,  'pe_xy_v_fb',         'XY Vel Feedback',     0.1),
    (96,  'pe_z_s1_integ',      'Z S1 Integ',         1.0),
    (97,  'pe_z_s1_corr',       'Z S1 Corr',          0.5),
    (98,  'pe_z_s2_integ',      'Z S2 Integ',         1.0),
    (99,  'pe_z_s2_corr',       'Z S2 Corr',         10.0),
    (100, 'pe_z_v_fb',          'Z Vel Feedback',      0.1),
    (101, 'pe_optflow_gain',    'OptFlow Gain',        5.0),
    # --- FFT/Notch (IDs 102-105) ---
    (102, 'notch_q',            'Notch Q Factor',      3.0),
    (103, 'notch_min_hz',       'Notch Min Hz',       50.0),
    (104, 'fft_peak_snr',       'FFT Peak SNR',        5.0),
    (105, 'fft_freq_alpha',     'FFT Freq Alpha',     0.15),
    # --- Flight State (IDs 106-109) ---
    (106, 'disarm_angle',       'Disarm Angle (deg)',  60.0),
    (107, 'disarm_range',       'Disarm Range (mm)',   10.0),
    (108, 'landing_range',      'Landing Range (mm)', 500.0),
    (109, 'took_off_range',     'Takeoff Range (mm)', 100.0),
    # --- Servo Bias (IDs 110-113) ---
    (110, 'servo_bias_1',       'Servo 1 Bias (bicopter L)',  0.0),
    (111, 'servo_bias_2',       'Servo 2 Bias (bicopter R)',  0.0),
    (112, 'servo_bias_3',       'Servo 3 Bias',               0.0),
    (113, 'servo_bias_4',       'Servo 4 Bias',               0.0),
    # --- Thrust Linearization (IDs 114-115) ---
    (114, 'thrust_p1',          'Thrust P1 (linear)',          1.0),
    (115, 'thrust_p2',          'Thrust P2 (quadratic)',       0.0),
    # --- RC Scale (IDs 116-118) ---
    (116, 'rc_xy_scale',        'RC XY Scale',                0.01),
    (117, 'rc_z_scale',         'RC Z Scale',                 0.04),
    (118, 'rc_yaw_scale',       'RC Yaw Scale',              -0.5),
]

CATEGORIES = [
    ('Attitude PID',     [(48,  69)]),
    ('Position Ctl',     [(70,  77), (116, 118)]),
    ('Motor/Servo',      [(78,  82), (110, 115)]),
    ('Att Estimation',   [(83,  90)]),
    ('Pos Estimation',   [(91, 101)]),
    ('FFT/Notch',        [(102, 105)]),
    ('Flight State',     [(106, 109)]),
]

# --- UI Colors ---
BG_COLOR       = '#1e1e1e'
PANEL_COLOR    = '#252526'
TEXT_COLOR     = '#cccccc'
DIM_TEXT       = '#888888'
GRID_COLOR     = '#3c3c3c'
ACCENT_BLUE    = '#5599ff'
ACCENT_GREEN   = '#55cc55'
ACCENT_RED     = '#ff5555'
ACCENT_YELLOW  = '#ffcc44'
BTN_COLOR      = '#333333'
BTN_HOVER      = '#444444'
BTN_GREEN      = '#2d5a2d'
BTN_GREEN_HOV  = '#3d7a3d'
BTN_RED        = '#5a2d2d'
BTN_RED_HOV    = '#7a3d3d'
BTN_ORANGE     = '#5a4a2d'
BTN_ORANGE_HOV = '#7a6a3d'
BTN_TAB_ACTIVE     = '#264f78'
BTN_TAB_ACTIVE_HOV = '#305f88'
HIGHLIGHT_BG   = '#264f78'

MAX_ROWS  = 22
ROW_START = 0.88
ROW_STEP  = 0.039

# --- Auto-detect serial port ---
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


# --- DB Protocol ---

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


def send_reset(ser):
    ser.write(build_db_frame(DB_CMD_RESET, bytes([0x00])))
    ser.flush()


def send_tuning_param(ser, param_id, value):
    payload = struct.pack('<If', param_id, value)
    ser.write(build_db_frame(DB_CMD_TUNING, payload))
    ser.flush()


# --- Global State ---
g_serial = None
g_connected = False
storage_queue = queue.Queue()
status_queue = queue.Queue()
g_flash_values = {}
g_active_cat = 0
g_selected_pid = None
g_dirty = True
g_querying = False
g_query_start = 0
g_query_pages = 0
g_query_bytes = bytearray()
g_confirm_defaults = False
g_confirm_time = 0


# --- Serial Reader Thread ---

def serial_reader():
    global g_serial, g_connected
    if not SERIAL_PORT:
        return

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            g_serial = ser
            g_connected = True
            status_queue.put((f"Connected to {SERIAL_PORT}", ACCENT_GREEN))
            time.sleep(0.2)
            ser.reset_input_buffer()
            ser.write(b'\x00' * 32)
            ser.flush()
            time.sleep(0.3)
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
                ser.read(2)

                if msg_id == SEND_LOG_ID:
                    if length > 0 and length % 4 == 0:
                        storage_queue.put(payload)

    except Exception as e:
        status_queue.put((f"Serial error: {e}", ACCENT_RED))
        g_connected = False


# --- Storage Helpers ---

def parse_storage_pages():
    global g_flash_values, g_dirty
    num_floats = len(g_query_bytes) // 4
    if num_floats == 0:
        return
    values = struct.unpack(f'<{num_floats}f', g_query_bytes[:num_floats * 4])
    for pid, name, display, default in TUNING_PARAMS:
        if pid < len(values):
            g_flash_values[pid] = values[pid]
        else:
            g_flash_values[pid] = default
    status_queue.put((f"Flash loaded: {g_query_pages} pages, {num_floats} params", ACCENT_GREEN))
    g_dirty = True


def get_category_params(cat_idx):
    _, ranges = CATEGORIES[cat_idx]
    return [(pid, name, disp, default) for pid, name, disp, default in TUNING_PARAMS
            if any(lo <= pid <= hi for lo, hi in ranges)]


# =====================================================================
#  GUI SETUP
# =====================================================================

fig = plt.figure(figsize=(14, 9))
fig.patch.set_facecolor(BG_COLOR)
fig.canvas.manager.set_window_title('Flight Controller Tuning Board')

# --- Title ---
fig.text(0.03, 0.965, 'FLIGHT CONTROLLER TUNING BOARD',
         fontsize=15, color=TEXT_COLOR, fontweight='bold', fontfamily='monospace')

conn_text = fig.text(0.97, 0.965, '', fontsize=10, fontfamily='monospace', ha='right')

# --- Category Tab Buttons ---
cat_btn_objects = []
tab_width = 0.125
tab_gap = 0.006
tab_x0 = 0.02

for i, (cat_name, _) in enumerate(CATEGORIES):
    x = tab_x0 + i * (tab_width + tab_gap)
    ax_tab = fig.add_axes([x, 0.905, tab_width, 0.04])
    color = BTN_TAB_ACTIVE if i == 0 else BTN_COLOR
    hover = BTN_TAB_ACTIVE_HOV if i == 0 else BTN_HOVER
    btn = Button(ax_tab, cat_name, color=color, hovercolor=hover)
    btn.label.set_fontsize(8)
    btn.label.set_color(TEXT_COLOR)
    cat_btn_objects.append(btn)


def _on_tab(idx):
    def handler(event):
        global g_active_cat, g_selected_pid, g_dirty
        g_active_cat = idx
        g_selected_pid = None
        g_dirty = True
        for j, b in enumerate(cat_btn_objects):
            if j == idx:
                b.color = BTN_TAB_ACTIVE
                b.hovercolor = BTN_TAB_ACTIVE_HOV
                b.ax.set_facecolor(BTN_TAB_ACTIVE)
            else:
                b.color = BTN_COLOR
                b.hovercolor = BTN_HOVER
                b.ax.set_facecolor(BTN_COLOR)
    return handler


for i, btn in enumerate(cat_btn_objects):
    btn.on_clicked(_on_tab(i))

# --- Table Axes ---
ax_table = fig.add_axes([0.02, 0.15, 0.96, 0.74])
ax_table.set_facecolor(PANEL_COLOR)
ax_table.set_xlim(0, 1)
ax_table.set_ylim(0, 1)
ax_table.set_xticks([])
ax_table.set_yticks([])
for spine in ax_table.spines.values():
    spine.set_edgecolor(GRID_COLOR)

# Category label inside table
cat_label = ax_table.text(0.02, 0.97, '', fontsize=12, fontfamily='monospace',
                          color=ACCENT_BLUE, fontweight='bold', va='top')

# Column headers
hdr_y = 0.93
ax_table.text(0.03, hdr_y, 'ID',        fontsize=9, fontfamily='monospace',
              color=DIM_TEXT, fontweight='bold', va='center')
ax_table.text(0.10, hdr_y, 'Parameter',  fontsize=9, fontfamily='monospace',
              color=DIM_TEXT, fontweight='bold', va='center')
ax_table.text(0.58, hdr_y, 'Flash',      fontsize=9, fontfamily='monospace',
              color=DIM_TEXT, fontweight='bold', va='center', ha='right')
ax_table.text(0.76, hdr_y, 'Default',    fontsize=9, fontfamily='monospace',
              color=DIM_TEXT, fontweight='bold', va='center', ha='right')
ax_table.text(0.84, hdr_y, 'Status',     fontsize=9, fontfamily='monospace',
              color=DIM_TEXT, fontweight='bold', va='center')
ax_table.axhline(y=0.91, color=GRID_COLOR, lw=0.5)

# Pre-allocate row elements
row_rects = []
row_id = []
row_name = []
row_flash = []
row_default = []
row_status = []

for i in range(MAX_ROWS):
    y = ROW_START - i * ROW_STEP
    rect = Rectangle((0, y - ROW_STEP / 2), 1, ROW_STEP,
                      facecolor='none', edgecolor='none', zorder=0)
    ax_table.add_patch(rect)
    row_rects.append(rect)

    kw = dict(fontsize=10, fontfamily='monospace', va='center', zorder=1)
    row_id.append(ax_table.text(0.03, y, '', color=DIM_TEXT, **kw))
    row_name.append(ax_table.text(0.10, y, '', color=TEXT_COLOR, **kw))
    row_flash.append(ax_table.text(0.58, y, '', color=TEXT_COLOR, ha='right', **kw))
    row_default.append(ax_table.text(0.76, y, '', color=DIM_TEXT, ha='right', **kw))
    row_status.append(ax_table.text(0.84, y, '', **kw))

# --- Info Line ---
info_text = fig.text(0.03, 0.115, 'Click a parameter row to select it',
                     fontsize=10, fontfamily='monospace', color=DIM_TEXT)

# --- Bottom Controls ---
ax_textbox = fig.add_axes([0.14, 0.055, 0.14, 0.038])
ax_textbox.set_facecolor('#2a2a2a')
for spine in ax_textbox.spines.values():
    spine.set_edgecolor(GRID_COLOR)
textbox = TextBox(ax_textbox, 'Value: ', initial='',
                  color='#2a2a2a', hovercolor='#333333')
textbox.label.set_color(DIM_TEXT)
textbox.label.set_fontsize(10)
textbox.label.set_fontfamily('monospace')
textbox.text_disp.set_color(TEXT_COLOR)
textbox.text_disp.set_fontfamily('monospace')
try:
    textbox.cursor.set_color(TEXT_COLOR)
except Exception:
    pass

ax_upload = fig.add_axes([0.29, 0.055, 0.08, 0.038])
btn_upload = Button(ax_upload, 'Upload', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
btn_upload.label.set_fontsize(9)
btn_upload.label.set_color(TEXT_COLOR)

ax_query = fig.add_axes([0.50, 0.055, 0.11, 0.038])
btn_query = Button(ax_query, 'Query FC', color=BTN_COLOR, hovercolor=BTN_HOVER)
btn_query.label.set_fontsize(9)
btn_query.label.set_color(TEXT_COLOR)

ax_defaults = fig.add_axes([0.62, 0.055, 0.14, 0.038])
btn_defaults = Button(ax_defaults, 'Upload Defaults', color=BTN_ORANGE, hovercolor=BTN_ORANGE_HOV)
btn_defaults.label.set_fontsize(9)
btn_defaults.label.set_color(TEXT_COLOR)

ax_reset = fig.add_axes([0.77, 0.055, 0.10, 0.038])
btn_reset = Button(ax_reset, 'Reset FC', color=BTN_RED, hovercolor=BTN_RED_HOV)
btn_reset.label.set_fontsize(9)
btn_reset.label.set_color(TEXT_COLOR)

status_bar = fig.text(0.03, 0.015, 'Ready', fontsize=9, fontfamily='monospace', color=DIM_TEXT)


# =====================================================================
#  TABLE RENDERING
# =====================================================================

def rebuild_table():
    global g_dirty
    cat_name, _ = CATEGORIES[g_active_cat]
    params = get_category_params(g_active_cat)
    cat_label.set_text(f'{cat_name}  ({len(params)} params)')

    for i in range(MAX_ROWS):
        if i < len(params):
            pid, _, disp, default = params[i]
            flash_val = g_flash_values.get(pid)
            is_selected = (pid == g_selected_pid)

            row_id[i].set_text(str(pid))
            row_name[i].set_text(disp)
            row_default[i].set_text(f'{default:.4f}')

            if flash_val is not None:
                row_flash[i].set_text(f'{flash_val:.4f}')
                if abs(flash_val - default) > 1e-6:
                    row_flash[i].set_color(ACCENT_YELLOW)
                    row_status[i].set_text('\u2260')
                    row_status[i].set_color(ACCENT_YELLOW)
                else:
                    row_flash[i].set_color(ACCENT_GREEN)
                    row_status[i].set_text('=')
                    row_status[i].set_color(ACCENT_GREEN)
            else:
                row_flash[i].set_text('\u2014')
                row_flash[i].set_color(DIM_TEXT)
                row_status[i].set_text('?')
                row_status[i].set_color(DIM_TEXT)

            if is_selected:
                row_rects[i].set_facecolor(HIGHLIGHT_BG)
                row_name[i].set_color(ACCENT_BLUE)
                row_id[i].set_color(ACCENT_BLUE)
            else:
                row_rects[i].set_facecolor('none')
                row_name[i].set_color(TEXT_COLOR)
                row_id[i].set_color(DIM_TEXT)

            for obj in (row_id[i], row_name[i], row_flash[i],
                        row_default[i], row_status[i], row_rects[i]):
                obj.set_visible(True)
        else:
            for obj in (row_id[i], row_name[i], row_flash[i],
                        row_default[i], row_status[i], row_rects[i]):
                obj.set_visible(False)

    if g_selected_pid is not None:
        for pid, _, disp, default in TUNING_PARAMS:
            if pid == g_selected_pid:
                flash_val = g_flash_values.get(pid)
                current = flash_val if flash_val is not None else default
                info_text.set_text(
                    f'Selected: [{pid}] {disp} = {current:.4f}   (default: {default:.4f})')
                info_text.set_color(ACCENT_BLUE)
                break
    else:
        info_text.set_text('Click a parameter row to select it')
        info_text.set_color(DIM_TEXT)

    g_dirty = False


# =====================================================================
#  EVENT HANDLERS
# =====================================================================

def on_table_click(event):
    global g_selected_pid, g_dirty
    if event.inaxes != ax_table:
        return
    y = event.ydata
    if y is None or y > 0.91:
        return
    idx = int((ROW_START + ROW_STEP / 2 - y) / ROW_STEP)
    params = get_category_params(g_active_cat)
    if 0 <= idx < len(params):
        g_selected_pid = params[idx][0]
    else:
        g_selected_pid = None
    g_dirty = True


fig.canvas.mpl_connect('button_press_event', on_table_click)


def _find_param(pid):
    for p in TUNING_PARAMS:
        if p[0] == pid:
            return p
    return None


def do_upload_param():
    global g_dirty
    val_str = textbox.text.strip()
    if not val_str:
        status_queue.put(("Enter a value first", ACCENT_YELLOW))
        return

    parts = val_str.split()
    if len(parts) >= 2:
        try:
            pid = int(parts[0])
            val = float(parts[1])
        except ValueError:
            status_queue.put(("Invalid input. Use: VALUE  or  ID VALUE", ACCENT_RED))
            return
    elif g_selected_pid is not None:
        try:
            val = float(parts[0])
        except ValueError:
            status_queue.put((f"Invalid value: {parts[0]}", ACCENT_RED))
            return
        pid = g_selected_pid
    else:
        status_queue.put(("Select a param first, or type: ID VALUE", ACCENT_YELLOW))
        return

    if pid < TUNING_FIRST or pid > TUNING_LAST:
        status_queue.put((f"ID {pid} out of range ({TUNING_FIRST}-{TUNING_LAST})", ACCENT_RED))
        return
    if not g_serial:
        status_queue.put(("Not connected to FC", ACCENT_RED))
        return

    param = _find_param(pid)
    disp = param[2] if param else f'ID {pid}'
    send_tuning_param(g_serial, pid, val)
    g_flash_values[pid] = val
    g_dirty = True
    textbox.set_val('')
    status_queue.put((f"Uploaded: {disp} = {val:.4f}", ACCENT_GREEN))


def on_upload(event):
    do_upload_param()


def on_textbox_submit(text):
    do_upload_param()


def on_query(event):
    global g_querying, g_query_pages, g_query_bytes, g_query_start
    if not g_serial:
        status_queue.put(("Not connected to FC", ACCENT_RED))
        return
    while not storage_queue.empty():
        try:
            storage_queue.get_nowait()
        except queue.Empty:
            break
    g_querying = True
    g_query_pages = 0
    g_query_bytes = bytearray()
    g_query_start = time.time()
    send_log_class(g_serial, LOG_CLASS_STORAGE)
    status_queue.put(("Querying flash storage...", ACCENT_YELLOW))


def on_defaults(event):
    global g_confirm_defaults, g_confirm_time
    if g_confirm_defaults and time.time() - g_confirm_time < 3.0:
        g_confirm_defaults = False
        btn_defaults.label.set_text('Upload Defaults')
        btn_defaults.color = BTN_ORANGE
        btn_defaults.hovercolor = BTN_ORANGE_HOV
        btn_defaults.ax.set_facecolor(BTN_ORANGE)
        _upload_all_defaults()
    else:
        g_confirm_defaults = True
        g_confirm_time = time.time()
        btn_defaults.label.set_text('Confirm?')
        btn_defaults.color = BTN_RED
        btn_defaults.hovercolor = BTN_RED_HOV
        btn_defaults.ax.set_facecolor(BTN_RED)


def _upload_all_defaults():
    if not g_serial:
        status_queue.put(("Not connected to FC", ACCENT_RED))
        return

    def _worker():
        global g_dirty
        for pid, _, _, default in TUNING_PARAMS:
            send_tuning_param(g_serial, pid, default)
            g_flash_values[pid] = default
            time.sleep(0.05)
        g_dirty = True
        status_queue.put((f"All {TUNING_COUNT} defaults uploaded", ACCENT_GREEN))

    threading.Thread(target=_worker, daemon=True).start()
    status_queue.put((f"Uploading {TUNING_COUNT} defaults...", ACCENT_YELLOW))


def on_reset(event):
    if not g_serial:
        status_queue.put(("Not connected to FC", ACCENT_RED))
        return
    send_reset(g_serial)
    status_queue.put(("Reset command sent", ACCENT_GREEN))

    def _post_reset():
        time.sleep(2.0)
        if g_serial and g_serial.is_open:
            g_serial.reset_input_buffer()
            send_log_class(g_serial, LOG_CLASS_HEART_BEAT)
            status_queue.put(("Post-reset: heartbeat sent", DIM_TEXT))

    threading.Thread(target=_post_reset, daemon=True).start()


def on_close(event):
    if g_serial and g_serial.is_open:
        try:
            send_log_class(g_serial, LOG_CLASS_HEART_BEAT)
        except Exception:
            pass


btn_upload.on_clicked(on_upload)
btn_query.on_clicked(on_query)
btn_defaults.on_clicked(on_defaults)
btn_reset.on_clicked(on_reset)
textbox.on_submit(on_textbox_submit)
fig.canvas.mpl_connect('close_event', on_close)


# =====================================================================
#  ANIMATION UPDATE
# =====================================================================

def update(frame):
    global g_querying, g_query_pages, g_query_bytes, g_query_start
    global g_dirty, g_confirm_defaults

    # --- Collect storage pages ---
    while not storage_queue.empty():
        try:
            payload = storage_queue.get_nowait()
            if g_querying:
                g_query_bytes.extend(payload)
                g_query_pages += 1
                status_queue.put((f"Page {g_query_pages}/{TOTAL_PAGES} received", ACCENT_YELLOW))
                if g_query_pages >= TOTAL_PAGES:
                    parse_storage_pages()
                    g_querying = False
        except queue.Empty:
            break

    # --- Query timeout ---
    if g_querying and time.time() - g_query_start > 6.0:
        if g_query_pages > 0:
            parse_storage_pages()
        else:
            status_queue.put(("Query timeout \u2014 no response from FC", ACCENT_RED))
        g_querying = False

    # --- Drain status queue (show latest) ---
    last_msg = None
    while not status_queue.empty():
        try:
            last_msg = status_queue.get_nowait()
        except queue.Empty:
            break
    if last_msg:
        msg, color = last_msg
        status_bar.set_text(msg)
        status_bar.set_color(color)

    # --- Connection indicator ---
    if g_connected:
        conn_text.set_text('\u25cf Connected')
        conn_text.set_color(ACCENT_GREEN)
    else:
        conn_text.set_text('\u25cb Disconnected')
        conn_text.set_color(ACCENT_RED)

    # --- Confirm defaults timeout ---
    if g_confirm_defaults and time.time() - g_confirm_time > 3.0:
        g_confirm_defaults = False
        btn_defaults.label.set_text('Upload Defaults')
        btn_defaults.color = BTN_ORANGE
        btn_defaults.hovercolor = BTN_ORANGE_HOV
        btn_defaults.ax.set_facecolor(BTN_ORANGE)

    # --- Rebuild table ---
    if g_dirty:
        rebuild_table()

    return []


# =====================================================================
#  MAIN
# =====================================================================

rebuild_table()

reader = threading.Thread(target=serial_reader, daemon=True)
reader.start()

ani = FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)
plt.show()
