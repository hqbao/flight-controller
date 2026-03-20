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
Magnetometer Debug Dashboard

Real-time 3D visualization of tilt-compensated magnetometer data for
debugging compass calibration and heading estimation.

Vectors:
  Red:   Raw Magnetometer (body frame)
  Blue:  Earth-Frame Mag (tilt-compensated)
  Green: Predicted Gravity (attitude reference)

3D Axes (NED body frame):
  X axis: Forward, Y axis: Right, Z axis: Down

Frame layout (LOG_CLASS_ATTITUDE_MAG = 0x07, 36 bytes):
  float[0..2] = Raw magnetometer (body frame, 3 floats)
  float[3..5] = Earth-frame mag (tilt-compensated, 3 floats)
  float[6..8] = Predicted gravity (attitude, 3 floats)

Usage:
  python3 attitude_estimation_mag_view.py

Useful for:
  - Verifying compass calibration (hard/soft iron)
  - Checking tilt compensation quality
  - Debugging heading estimation
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
SEND_LOG_ID = 0x00

LOG_CLASS_NONE         = 0x00
LOG_CLASS_ATTITUDE_MAG = 0x07
DB_CMD_LOG_CLASS       = 0x03
DB_CMD_RESET           = 0x07
DB_CMD_CHIP_ID         = 0x09

MAG_FRAME_SIZE = 36  # 9 floats

# --- UI Colors ---
BG_COLOR       = '#1e1e1e'
PANEL_COLOR    = '#252526'
TEXT_COLOR     = '#cccccc'
DIM_TEXT       = '#888888'
GRID_COLOR     = '#3c3c3c'
ACCENT_ORANGE  = '#ff9955'
BTN_GREEN      = '#2d5a2d'
BTN_GREEN_HOV  = '#3d7a3d'
BTN_RED        = '#5a2d2d'
BTN_RED_HOV    = '#7a3d3d'
BTN_COLOR      = '#333333'
BTN_HOVER      = '#444444'

COLOR_RAW      = '#ff5555'   # Red: raw magnetometer
COLOR_EARTH    = '#5599ff'   # Blue: earth-frame mag
COLOR_GRAVITY  = '#55dd55'   # Green: gravity vector

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
    names = {0x00: 'NONE', 0x07: 'ATTITUDE_MAG'}
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
    """Background thread: reads DB frames from FC, parses mag debug payloads."""
    global g_serial, g_chip_id
    if not SERIAL_PORT:
        return
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(b'\x00' * 32)
        ser.flush()
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
            elif msg_id == SEND_LOG_ID and length == MAG_FRAME_SIZE:
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
    fig.suptitle('Magnetometer Debug \u2014 Tilt Compensation', fontsize=14,
                 color=TEXT_COLOR, fontweight='bold', y=0.99)

    # =========================================================================
    # 3D Viewport
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

    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)
    ax.set_xlabel('X \u2014 Forward', fontsize=9, labelpad=6)
    ax.set_ylabel('Y \u2014 Right', fontsize=9, labelpad=6)
    ax.set_zlabel('Z \u2014 Down', fontsize=9, labelpad=6)
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
    line_raw, = ax.plot([0, 0], [0, 0], [0, 0], color=COLOR_RAW,
                        linewidth=3, label='Raw Mag (body)')
    head_raw, = ax.plot([0], [0], [0], color=COLOR_RAW, marker='o',
                        markersize=8)

    line_earth, = ax.plot([0, 0], [0, 0], [0, 0], color=COLOR_EARTH,
                          linewidth=3, label='Earth Mag (tilt-comp)')
    head_earth, = ax.plot([0], [0], [0], color=COLOR_EARTH, marker='o',
                          markersize=8)

    line_grav, = ax.plot([0, 0], [0, 0], [0, 0], color=COLOR_GRAVITY,
                         linewidth=2, label='Gravity (v_pred)', alpha=0.8)
    head_grav, = ax.plot([0], [0], [0], color=COLOR_GRAVITY, marker='o',
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

    # Raw Magnetometer
    ax_data.text(0.06, 0.95, 'RAW MAGNETOMETER', **_hdr)
    ax_data.plot([0.04, 0.96], [0.943, 0.943], color=COLOR_RAW, lw=1.0,
                 alpha=0.5, transform=ax_data.transAxes, clip_on=False)
    data_raw_x = ax_data.text(0.06, 0.91, 'X: +0.000', color=COLOR_RAW, **_val)
    data_raw_y = ax_data.text(0.06, 0.87, 'Y: +0.000', color=COLOR_RAW, **_val)
    data_raw_z = ax_data.text(0.06, 0.83, 'Z: +0.000', color=COLOR_RAW, **_val)
    data_raw_m = ax_data.text(0.06, 0.79, '|B|: 0.000', color=DIM_TEXT, **_val)

    # Earth-Frame Mag
    ax_data.text(0.06, 0.72, 'EARTH-FRAME MAG', **_hdr)
    ax_data.plot([0.04, 0.96], [0.713, 0.713], color=COLOR_EARTH, lw=1.0,
                 alpha=0.5, transform=ax_data.transAxes, clip_on=False)
    data_earth_x = ax_data.text(0.06, 0.68, 'X: +0.000', color=COLOR_EARTH, **_val)
    data_earth_y = ax_data.text(0.06, 0.64, 'Y: +0.000', color=COLOR_EARTH, **_val)
    data_earth_z = ax_data.text(0.06, 0.60, 'Z: +0.000', color=COLOR_EARTH, **_val)
    data_earth_m = ax_data.text(0.06, 0.56, '|B|: 0.000', color=DIM_TEXT, **_val)

    # Gravity Vector
    ax_data.text(0.06, 0.49, 'GRAVITY VECTOR', **_hdr)
    ax_data.plot([0.04, 0.96], [0.483, 0.483], color=COLOR_GRAVITY, lw=1.0,
                 alpha=0.5, transform=ax_data.transAxes, clip_on=False)
    data_grav_x = ax_data.text(0.06, 0.45, 'X: +0.000', color=COLOR_GRAVITY, **_val)
    data_grav_y = ax_data.text(0.06, 0.41, 'Y: +0.000', color=COLOR_GRAVITY, **_val)
    data_grav_z = ax_data.text(0.06, 0.37, 'Z: +0.000', color=COLOR_GRAVITY, **_val)

    # Heading
    ax_data.text(0.06, 0.30, 'HEADING', **_hdr)
    ax_data.plot([0.04, 0.96], [0.293, 0.293], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    data_heading = ax_data.text(0.06, 0.26, 'Hdg:   0.0\u00b0', color=ACCENT_ORANGE,
                                fontsize=12, fontweight='bold', ha='left',
                                fontfamily='monospace', transform=ax_data.transAxes)
    data_compass = ax_data.text(0.06, 0.22, 'Dir:   N', color=TEXT_COLOR, **_val)

    # =========================================================================
    # View buttons
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
                send_log_class_command(g_serial, LOG_CLASS_ATTITUDE_MAG)
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

    # =========================================================================
    # Animation loop
    # =========================================================================
    frame_count = [0]
    last_fps_time = [time.time()]

    def heading_to_compass(deg):
        """Convert heading degrees to compass direction string."""
        dirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
        idx = int((deg + 22.5) % 360 / 45)
        return dirs[idx]

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

        v_raw   = np.array(latest[0:3])
        v_earth = np.array(latest[3:6])
        v_grav  = np.array(latest[6:9])

        # =================================================================
        # 3D vectors
        # =================================================================
        line_raw.set_data([0, v_raw[0]], [0, v_raw[1]])
        line_raw.set_3d_properties([0, v_raw[2]])
        head_raw.set_data([v_raw[0]], [v_raw[1]])
        head_raw.set_3d_properties([v_raw[2]])

        line_earth.set_data([0, v_earth[0]], [0, v_earth[1]])
        line_earth.set_3d_properties([0, v_earth[2]])
        head_earth.set_data([v_earth[0]], [v_earth[1]])
        head_earth.set_3d_properties([v_earth[2]])

        line_grav.set_data([0, v_grav[0]], [0, v_grav[1]])
        line_grav.set_3d_properties([0, v_grav[2]])
        head_grav.set_data([v_grav[0]], [v_grav[1]])
        head_grav.set_3d_properties([v_grav[2]])

        # =================================================================
        # Data panel
        # =================================================================
        mag_raw   = np.linalg.norm(v_raw)
        mag_earth = np.linalg.norm(v_earth)

        data_raw_x.set_text(f'X:{v_raw[0]:+8.3f}')
        data_raw_y.set_text(f'Y:{v_raw[1]:+8.3f}')
        data_raw_z.set_text(f'Z:{v_raw[2]:+8.3f}')
        data_raw_m.set_text(f'|B|:{mag_raw:7.3f}')

        data_earth_x.set_text(f'X:{v_earth[0]:+8.3f}')
        data_earth_y.set_text(f'Y:{v_earth[1]:+8.3f}')
        data_earth_z.set_text(f'Z:{v_earth[2]:+8.3f}')
        data_earth_m.set_text(f'|B|:{mag_earth:7.3f}')

        data_grav_x.set_text(f'X:{v_grav[0]:+8.3f}')
        data_grav_y.set_text(f'Y:{v_grav[1]:+8.3f}')
        data_grav_z.set_text(f'Z:{v_grav[2]:+8.3f}')

        # Heading from earth-frame mag
        heading = np.degrees(np.arctan2(-v_earth[1], v_earth[0])) % 360
        data_heading.set_text(f'Hdg: {heading:5.1f}\u00b0')
        data_compass.set_text(f'Dir: {heading_to_compass(heading):>3s}')

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
