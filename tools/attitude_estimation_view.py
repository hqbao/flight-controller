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
Attitude Estimation Dashboard

Real-time 3D visualization of attitude fusion vectors.

Supports two log classes (toggled via Body/Earth button):
  LOG_CLASS_ATTITUDE       (0x03): Body-frame linear acceleration
  LOG_CLASS_ATTITUDE_EARTH (0x13): Earth-frame linear acceleration
  (v_pred and v_true are identical in both modes)

Vectors:
  Red:   Gyro Prediction (v_pred)  \u2014 gyro-integrated gravity estimate
  Blue:  Fused Estimate  (v_true)  \u2014 corrected gravity vector
  Green: Linear Acceleration       \u2014 gravity-subtracted (body or earth frame)

At rest (level), gravity vectors point to (0, 0, -1) in NED body frame.

3D Axis Mapping (display frame):
  Plot X axis (horizontal): Body Y \u2014 Right (Roll)
  Plot Y axis (depth):      Body X \u2014 Forward (Pitch)
  Plot Z axis (vertical):   -Z (Up)

Frame layout (36 bytes = 9 floats):
  float[0..2] = v_pred  (gyro prediction)
  float[3..5] = v_true  (fused estimate)
  float[6..8] = v_linear_acc (body frame) or v_linear_acc_earth_frame (earth frame)

Usage:
  python3 attitude_estimation_view.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
SEND_LOG_ID = 0x00

LOG_CLASS_NONE     = 0x00
LOG_CLASS_ATTITUDE = 0x03
LOG_CLASS_ATTITUDE_EARTH = 0x13
DB_CMD_LOG_CLASS   = 0x03
DB_CMD_RESET       = 0x07
DB_CMD_CHIP_ID     = 0x09

ATTITUDE_FRAME_SIZE = 36  # 9 floats

# --- UI Colors ---
BG_COLOR       = '#1e1e1e'
PANEL_COLOR    = '#252526'
TEXT_COLOR     = '#cccccc'
DIM_TEXT       = '#888888'
GRID_COLOR     = '#3c3c3c'
ACCENT_BLUE    = '#5599ff'
ACCENT_GREEN   = '#55cc55'
ACCENT_RED     = '#ff5555'
ACCENT_ORANGE  = '#ff9955'
BTN_GREEN      = '#2d5a2d'
BTN_GREEN_HOV  = '#3d7a3d'
BTN_RED        = '#5a2d2d'
BTN_RED_HOV    = '#7a3d3d'
BTN_COLOR      = '#333333'
BTN_HOVER      = '#444444'

COLOR_PRED     = '#ff5555'   # Red: gyro prediction
COLOR_TRUE     = '#5599ff'   # Blue: fused estimate
COLOR_LINEAR   = '#55dd55'   # Green: linear acceleration

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
data_queue = queue.Queue()
g_serial = None
g_logging_active = False
g_chip_id = None
g_earth_view = False  # False = body frame, True = earth frame


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
    ser.write(frame)
    ser.flush()
    names = {0x00: 'NONE', 0x03: 'ATTITUDE', 0x13: 'ATTITUDE_EARTH'}
    print(f"  \u2192 Log class: {names.get(log_class, f'0x{log_class:02X}')}")


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


def send_chip_id_request(ser):
    """Send DB frame to request the 8-byte unique chip ID."""
    msg_id = DB_CMD_CHIP_ID
    msg_class = 0x00
    length = 1
    payload = bytes([0x00])
    header = struct.pack('<2sBBH', b'db', msg_id, msg_class, length)
    checksum = (msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF) + 0x00) & 0xFFFF
    frame = header + payload + struct.pack('<H', checksum)
    ser.write(frame)
    ser.write(frame)
    ser.flush()
    print("  \u2192 Chip ID request sent")


def serial_reader():
    """Background thread: reads DB frames from FC, parses attitude payloads."""
    global g_serial, g_chip_id
    if not SERIAL_PORT:
        return
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(b'\x00' * 32)
        ser.flush()
        time.sleep(0.05)
        g_serial = ser
        print(f"  \u2713 Connected to {SERIAL_PORT}")
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
            _ = ser.read(2)  # checksum

            if msg_id == SEND_LOG_ID and length == 8 and g_chip_id is None:
                g_chip_id = payload[:8].hex().upper()
                print(f"  \u2713 Chip ID: {g_chip_id}")
            elif msg_id == SEND_LOG_ID and length == ATTITUDE_FRAME_SIZE:
                vals = struct.unpack('<9f', payload)
                data_queue.put(vals)
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


# --- GUI ---
def main():
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

    fig = plt.figure(figsize=(16, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('Attitude Estimation \u2014 Fusion Vectors', fontsize=14,
                 color=TEXT_COLOR, fontweight='bold', y=0.99)
    frame_label = fig.text(0.33, 0.95, 'Body Frame', fontsize=11,
                           ha='center', color=ACCENT_ORANGE, fontweight='bold')

    # =========================================================================
    # 3D Viewport (NED body frame)
    # =========================================================================
    ax = fig.add_axes([0.02, 0.12, 0.62, 0.82], projection='3d')
    ax.set_facecolor(BG_COLOR)
    ax.set_box_aspect((1, 1, 1))
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor(GRID_COLOR)
    ax.yaxis.pane.set_edgecolor(GRID_COLOR)
    ax.zaxis.pane.set_edgecolor(GRID_COLOR)

    # X reversed for visual clarity (right = right on screen)
    ax.set_xlim(1.2, -1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)
    ax.set_xlabel('Y \u2014 Right (Roll)', fontsize=9, labelpad=6)
    ax.set_ylabel('X \u2014 Forward (Pitch)', fontsize=9, labelpad=6)
    ax.set_zlabel('\u2013Z (Up)', fontsize=9, labelpad=6)
    ax.tick_params(labelsize=7)

    # Reference: unit sphere wireframe
    u = np.linspace(0, 2 * np.pi, 24)
    v = np.linspace(0, np.pi, 12)
    xs = np.outer(np.cos(u), np.sin(v))
    ys = np.outer(np.sin(u), np.sin(v))
    zs = np.outer(np.ones_like(u), np.cos(v))
    ax.plot_wireframe(xs, ys, zs, color=GRID_COLOR, alpha=0.08, linewidth=0.3)

    # Origin axes guides
    guide_len = 1.1
    ax.plot([-guide_len, guide_len], [0, 0], [0, 0],
            color=GRID_COLOR, linewidth=0.5, alpha=0.3)
    ax.plot([0, 0], [-guide_len, guide_len], [0, 0],
            color=GRID_COLOR, linewidth=0.5, alpha=0.3)
    ax.plot([0, 0], [0, 0], [-guide_len, guide_len],
            color=GRID_COLOR, linewidth=0.5, alpha=0.3)

    # Vector lines
    line_pred, = ax.plot([0, 0], [0, 0], [0, 0], color=COLOR_PRED,
                         linewidth=3, label='Gyro Prediction')
    head_pred, = ax.plot([0], [0], [0], color=COLOR_PRED, marker='o',
                         markersize=8)

    line_true, = ax.plot([0, 0], [0, 0], [0, 0], color=COLOR_TRUE,
                         linewidth=3, label='Fused Estimate')
    head_true, = ax.plot([0], [0], [0], color=COLOR_TRUE, marker='o',
                         markersize=8)

    line_linear, = ax.plot([0, 0], [0, 0], [0, 0], color=COLOR_LINEAR,
                           linewidth=2, label='Linear Accel', alpha=0.8)
    head_linear, = ax.plot([0], [0], [0], color=COLOR_LINEAR, marker='o',
                           markersize=6)

    ax.legend(loc='upper left', fontsize=8, framealpha=0.3,
              facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # =========================================================================
    # Data panel (right side)
    # =========================================================================
    ax_data = fig.add_axes([0.66, 0.12, 0.33, 0.82])
    ax_data.set_facecolor(PANEL_COLOR)
    ax_data.patch.set_alpha(0.7)
    ax_data.set_xlim(0, 1)
    ax_data.set_ylim(0, 1)
    ax_data.set_xticks([])
    ax_data.set_yticks([])
    for spine in ax_data.spines.values():
        spine.set_edgecolor(GRID_COLOR)
        spine.set_alpha(0.3)

    _mono = dict(fontfamily='monospace', transform=ax_data.transAxes)
    _hdr = dict(fontsize=9, color=DIM_TEXT, fontweight='bold', ha='left', **_mono)
    _val = dict(fontsize=10, fontweight='bold', ha='left', **_mono)
    _dim = dict(fontsize=9, ha='left', color=DIM_TEXT, **_mono)

    # Gyro Prediction
    ax_data.text(0.06, 0.95, 'GYRO PREDICTION', **_hdr)
    ax_data.plot([0.04, 0.96], [0.943, 0.943], color=COLOR_PRED, lw=1.0,
                 alpha=0.5, transform=ax_data.transAxes, clip_on=False)
    data_pred_x = ax_data.text(0.06, 0.91, 'X(Fwd): +0.000', color=COLOR_PRED, **_val)
    data_pred_y = ax_data.text(0.06, 0.87, 'Y(Rgt): +0.000', color=COLOR_PRED, **_val)
    data_pred_z = ax_data.text(0.06, 0.83, 'Z(Dwn): +0.000', color=COLOR_PRED, **_val)
    data_pred_m = ax_data.text(0.06, 0.79, '|v|:     0.000', color=DIM_TEXT, **_val)

    # Fused Estimate
    ax_data.text(0.06, 0.72, 'FUSED ESTIMATE', **_hdr)
    ax_data.plot([0.04, 0.96], [0.713, 0.713], color=COLOR_TRUE, lw=1.0,
                 alpha=0.5, transform=ax_data.transAxes, clip_on=False)
    data_true_x = ax_data.text(0.06, 0.68, 'X(Fwd): +0.000', color=COLOR_TRUE, **_val)
    data_true_y = ax_data.text(0.06, 0.64, 'Y(Rgt): +0.000', color=COLOR_TRUE, **_val)
    data_true_z = ax_data.text(0.06, 0.60, 'Z(Dwn): +0.000', color=COLOR_TRUE, **_val)
    data_true_m = ax_data.text(0.06, 0.56, '|v|:     0.000', color=DIM_TEXT, **_val)

    # Linear Acceleration
    lbl_linear = ax_data.text(0.06, 0.49, 'LINEAR ACCEL (Body)', **_hdr)
    ax_data.plot([0.04, 0.96], [0.483, 0.483], color=COLOR_LINEAR, lw=1.0,
                 alpha=0.5, transform=ax_data.transAxes, clip_on=False)
    data_lin_x = ax_data.text(0.06, 0.45, 'X(Fwd): +0.000', color=COLOR_LINEAR, **_val)
    data_lin_y = ax_data.text(0.06, 0.41, 'Y(Rgt): +0.000', color=COLOR_LINEAR, **_val)
    data_lin_z = ax_data.text(0.06, 0.37, 'Z(Dwn): +0.000', color=COLOR_LINEAR, **_val)
    data_lin_m = ax_data.text(0.06, 0.33, '|v|:     0.000', color=DIM_TEXT, **_val)

    # Error metric
    ax_data.text(0.06, 0.26, 'FUSION', **_hdr)
    ax_data.plot([0.04, 0.96], [0.253, 0.253], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_error = ax_data.text(0.06, 0.22, 'Err:  0.000\u00b0', color=ACCENT_ORANGE, **_val)
    data_tilt  = ax_data.text(0.06, 0.18, 'Tilt: 0.0\u00b0', color=TEXT_COLOR, **_val)

    # =========================================================================
    # View buttons (below 3D viewport)
    # =========================================================================
    views = {
        'Top':   (90, 0),
        'Front': (0, 180),
        'Back':  (0, 0),
        'Left':  (0, -90),
        'Right': (0, 90),
    }

    view_btns = []
    for i, (label, (elev, azim)) in enumerate(views.items()):
        bx = fig.add_axes([0.02 + i * 0.065, 0.05, 0.06, 0.035])
        b = Button(bx, label, color=BTN_COLOR, hovercolor=BTN_HOVER)
        b.label.set_color(TEXT_COLOR)
        b.label.set_fontsize(7)
        b.on_clicked(lambda event, e=elev, a=azim: ax.view_init(elev=e, azim=a))
        view_btns.append(b)

    # =========================================================================
    # Status bar + control buttons
    # =========================================================================
    chip_id_text = fig.text(0.96, 0.035, 'Chip ID: ---', fontsize=7,
                            ha='right', color=DIM_TEXT)
    fps_text = fig.text(0.96, 0.015, '', fontsize=8, ha='right',
                        color=DIM_TEXT)

    ax_toggle = fig.add_axes([0.66, 0.005, 0.08, 0.04])
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
                lc = LOG_CLASS_ATTITUDE_EARTH if g_earth_view else LOG_CLASS_ATTITUDE
                send_log_class_command(g_serial, lc)
                g_logging_active = True
                btn_toggle.label.set_text('Stop Log')
                ax_toggle.set_facecolor(BTN_RED)

    btn_toggle.on_clicked(on_toggle)

    ax_reset = fig.add_axes([0.745, 0.005, 0.08, 0.04])
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

    # Body/Earth frame toggle button
    ax_frame = fig.add_axes([0.83, 0.005, 0.10, 0.04])
    btn_frame = Button(ax_frame, 'Earth Frame',
                       color=BTN_COLOR, hovercolor=BTN_HOVER)
    btn_frame.label.set_color(TEXT_COLOR)
    btn_frame.label.set_fontsize(8)

    def on_frame_toggle(event):
        global g_earth_view
        g_earth_view = not g_earth_view
        if g_earth_view:
            btn_frame.label.set_text('Body Frame')
            ax_frame.set_facecolor(ACCENT_BLUE)
            frame_label.set_text('Earth Frame')
            frame_label.set_color(ACCENT_BLUE)
        else:
            btn_frame.label.set_text('Earth Frame')
            ax_frame.set_facecolor(BTN_COLOR)
            frame_label.set_text('Body Frame')
            frame_label.set_color(ACCENT_ORANGE)
        # If logging, switch log class live
        if g_logging_active and g_serial and g_serial.is_open:
            lc = LOG_CLASS_ATTITUDE_EARTH if g_earth_view else LOG_CLASS_ATTITUDE
            send_log_class_command(g_serial, lc)

    btn_frame.on_clicked(on_frame_toggle)

    # =========================================================================
    # Animation loop
    # =========================================================================
    frame_count = [0]
    last_fps_time = [time.time()]

    def update(frame_num):
        updated = []
        latest = None

        while not data_queue.empty():
            try:
                latest = data_queue.get_nowait()
            except queue.Empty:
                break

        if latest is None:
            return updated

        v_pred = np.array(latest[0:3])
        v_true = np.array(latest[3:6])
        v_lin  = np.array(latest[6:9])

        # =================================================================
        # 3D vectors
        # =================================================================
        line_pred.set_data([0, v_pred[0]], [0, v_pred[1]])
        line_pred.set_3d_properties([0, v_pred[2]])
        head_pred.set_data([v_pred[0]], [v_pred[1]])
        head_pred.set_3d_properties([v_pred[2]])

        line_true.set_data([0, v_true[0]], [0, v_true[1]])
        line_true.set_3d_properties([0, v_true[2]])
        head_true.set_data([v_true[0]], [v_true[1]])
        head_true.set_3d_properties([v_true[2]])

        line_linear.set_data([0, v_lin[0]], [0, v_lin[1]])
        line_linear.set_3d_properties([0, v_lin[2]])
        head_linear.set_data([v_lin[0]], [v_lin[1]])
        head_linear.set_3d_properties([v_lin[2]])

        # =================================================================
        # Data panel
        # =================================================================
        mag_pred = np.linalg.norm(v_pred)
        mag_true = np.linalg.norm(v_true)
        mag_lin  = np.linalg.norm(v_lin)

        # Update linear accel label based on current frame
        if g_earth_view:
            lbl_linear.set_text('LINEAR ACCEL (Earth)')
        else:
            lbl_linear.set_text('LINEAR ACCEL (Body)')

        data_pred_x.set_text(f'X(Fwd):{v_pred[0]:+7.3f}')
        data_pred_y.set_text(f'Y(Rgt):{v_pred[1]:+7.3f}')
        data_pred_z.set_text(f'Z(Dwn):{v_pred[2]:+7.3f}')
        data_pred_m.set_text(f'|v|:   {mag_pred:7.3f}')

        data_true_x.set_text(f'X(Fwd):{v_true[0]:+7.3f}')
        data_true_y.set_text(f'Y(Rgt):{v_true[1]:+7.3f}')
        data_true_z.set_text(f'Z(Dwn):{v_true[2]:+7.3f}')
        data_true_m.set_text(f'|v|:   {mag_true:7.3f}')

        data_lin_x.set_text(f'X(Fwd):{v_lin[0]:+7.3f}')
        data_lin_y.set_text(f'Y(Rgt):{v_lin[1]:+7.3f}')
        data_lin_z.set_text(f'Z(Dwn):{v_lin[2]:+7.3f}')
        data_lin_m.set_text(f'|v|:   {mag_lin:7.3f}')

        # Angular error between pred and true
        dot = np.clip(np.dot(v_pred, v_true) / max(mag_pred * mag_true, 1e-9), -1, 1)
        err_deg = np.degrees(np.arccos(dot))
        data_error.set_text(f'Err:  {err_deg:.3f}\u00b0')

        # Tilt from vertical (fused vector vs [0,0,-1])
        if mag_true > 0.01:
            tilt_dot = np.clip(np.dot(v_true, [0, 0, -1]) / mag_true, -1, 1)
            tilt_deg = np.degrees(np.arccos(tilt_dot))
        else:
            tilt_deg = 0.0
        data_tilt.set_text(f'Tilt: {tilt_deg:.1f}\u00b0')

        # =================================================================
        # Chip ID + FPS
        # =================================================================
        if g_chip_id is not None:
            chip_id_text.set_text(f'Chip ID: {g_chip_id}')
            updated.append(chip_id_text)

        now = time.time()
        frame_count[0] += 1
        elapsed = now - last_fps_time[0]
        if elapsed >= 1.0:
            fps = frame_count[0] / elapsed
            fps_text.set_text(f'{fps:.0f} Hz')
            frame_count[0] = 0
            last_fps_time[0] = now
            updated.append(fps_text)

        return updated

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
