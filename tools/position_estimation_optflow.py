import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import sys
import matplotlib
matplotlib.use('macosx' if sys.platform == 'darwin' else 'TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation
import time

"""
Optical Flow & Altitude Sensor Viewer

Real-time visualization of optical flow sensor data and altitude readings
from the position estimation module.

DATA FORMAT (LOG_CLASS_POSITION_OPTFLOW = 0x06, 32 bytes):
  float[0] = Optical Flow Downward dx (rad)
  float[1] = Optical Flow Downward dy (rad)
  float[2] = Optical Flow Upward   dx (rad)
  float[3] = Optical Flow Upward   dy (rad)
  float[4] = Range Finder Altitude (m)
  float[5] = Air Pressure Altitude (m)
  float[6] = Body-frame linear accel X (m/s^2, X = Forward)
  float[7] = Body-frame linear accel Y (m/s^2, Y = Right)

Layout (3 vertically stacked time-series):
  Top:    Optical Flow — Downward (dx, dy)
  Middle: Optical Flow — Upward (dx, dy)
  Bottom: Altitude Sensors (Range Finder, Air Pressure)

MOUNTING CHECK
  An IMU-derived reference velocity (leaky-integrated body accel) is compared
  against the optical flow displacement on a per-axis basis. If the signs
  disagree consistently while the drone is being moved, a banner appears
  asking the user to re-mount the optical flow module (likely rotated 90°
  or mounted upside-down).

Usage:
  python3 position_estimation_optflow.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 38400
SEND_LOG_ID = 0x00

LOG_CLASS_NONE             = 0x00
LOG_CLASS_HEART_BEAT       = 0x09
LOG_CLASS_POSITION_OPTFLOW = 0x06
DB_CMD_LOG_CLASS           = 0x03
DB_CMD_RESET               = 0x07
DB_CMD_CHIP_ID             = 0x09

OPTFLOW_FRAME_SIZE = 32   # 8 floats
HISTORY_LEN = 300          # ~30s at 10 Hz

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

# Signal colors
COLOR_DOWN_DX  = '#4fc3f7'  # Cyan for downward dx
COLOR_DOWN_DY  = '#81d4fa'  # Light cyan for downward dy
COLOR_UP_DX    = '#ef5350'  # Red for upward dx
COLOR_UP_DY    = '#ef9a9a'  # Light red for upward dy
COLOR_RANGE    = '#ffa726'  # Orange for range finder
COLOR_BARO     = '#42a5f5'  # Blue for air pressure

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
    ser.flush()
    names = {0x00: 'NONE', 0x06: 'POSITION_OPTFLOW'}
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

    ser.flush()
    print("  \u2192 Chip ID request sent")


def serial_reader():
    """Background thread: reads DB frames from FC, parses optical flow payloads."""
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
        send_log_class_command(ser, LOG_CLASS_HEART_BEAT)

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
            elif msg_id == SEND_LOG_ID and length >= OPTFLOW_FRAME_SIZE:
                vals = struct.unpack('<8f', payload[:32])
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

    fig, (ax_down, ax_up, ax_alt) = plt.subplots(3, 1, figsize=(14, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('Optical Flow & Altitude Sensors', fontsize=14,
                 color=TEXT_COLOR, fontweight='bold', y=0.99)
    fig.subplots_adjust(left=0.08, right=0.95, top=0.93, bottom=0.10,
                        hspace=0.40)

    # =========================================================================
    # Optical Flow Downward
    # =========================================================================
    ax_down.set_title('Optical Flow \u2014 Downward', color=COLOR_DOWN_DX,
                      fontsize=10, fontweight='bold', pad=6)
    ax_down.set_ylabel('Flow (rad)', fontsize=8)
    ax_down.grid(True, alpha=0.2, linestyle=':')
    ax_down.tick_params(labelsize=7)
    ax_down.axhline(0, color=GRID_COLOR, linewidth=0.5, alpha=0.5)
    line_ddx, = ax_down.plot([], [], '-', color=COLOR_DOWN_DX, linewidth=1.5,
                             label='dx')
    line_ddy, = ax_down.plot([], [], '-', color=COLOR_DOWN_DY, linewidth=1.5,
                             label='dy')
    ax_down.legend(loc='upper left', fontsize=7, framealpha=0.3)
    val_text_down = ax_down.text(
        0.98, 0.92, '', transform=ax_down.transAxes,
        fontsize=8, ha='right', va='top', fontfamily='monospace',
        color=TEXT_COLOR)

    # =========================================================================
    # Optical Flow Upward
    # =========================================================================
    ax_up.set_title('Optical Flow \u2014 Upward', color=COLOR_UP_DX,
                    fontsize=10, fontweight='bold', pad=6)
    ax_up.set_ylabel('Flow (rad)', fontsize=8)
    ax_up.grid(True, alpha=0.2, linestyle=':')
    ax_up.tick_params(labelsize=7)
    ax_up.axhline(0, color=GRID_COLOR, linewidth=0.5, alpha=0.5)
    line_udx, = ax_up.plot([], [], '-', color=COLOR_UP_DX, linewidth=1.5,
                           label='dx')
    line_udy, = ax_up.plot([], [], '-', color=COLOR_UP_DY, linewidth=1.5,
                           label='dy')
    ax_up.legend(loc='upper left', fontsize=7, framealpha=0.3)
    val_text_up = ax_up.text(
        0.98, 0.92, '', transform=ax_up.transAxes,
        fontsize=8, ha='right', va='top', fontfamily='monospace',
        color=TEXT_COLOR)

    # =========================================================================
    # Altitude Sensors
    # =========================================================================
    ax_alt.set_title('Altitude Sensors', color=TEXT_COLOR,
                     fontsize=10, fontweight='bold', pad=6)
    ax_alt.set_ylabel('Altitude (m)', fontsize=8)
    ax_alt.set_xlabel('Time (s)', fontsize=8)
    ax_alt.grid(True, alpha=0.2, linestyle=':')
    ax_alt.tick_params(labelsize=7)
    ax_alt.axhline(0, color=GRID_COLOR, linewidth=0.5, alpha=0.5)
    line_range, = ax_alt.plot([], [], '-', color=COLOR_RANGE, linewidth=1.5,
                              label='Range Finder')
    line_baro, = ax_alt.plot([], [], '-', color=COLOR_BARO, linewidth=1.5,
                             label='Air Pressure')
    ax_alt.legend(loc='upper left', fontsize=7, framealpha=0.3)
    val_text_alt = ax_alt.text(
        0.98, 0.92, '', transform=ax_alt.transAxes,
        fontsize=8, ha='right', va='top', fontfamily='monospace',
        color=TEXT_COLOR)

    # History buffer
    hist = {'t': [], 'ddx': [], 'ddy': [], 'udx': [], 'udy': [],
            'range': [], 'baro': []}
    t0 = [None]

    # =========================================================================
    # Mounting Check state
    # =========================================================================
    # Leaky-integrated body-frame velocity (independent reference from accel).
    # We compare its sign against the sign of the optical flow displacement
    # to detect a mis-mounted optical flow module.
    ref_vel  = {'x': 0.0, 'y': 0.0}     # m/s (leaky integral of body accel)
    last_acc_t = [None]
    VEL_LEAK_TAU = 1.5                   # s, decay time constant
    VEL_TRIGGER  = 0.10                  # m/s, ignore micro-motion
    FLOW_TRIGGER = 0.003                 # rad, ignore noise
    # Per direction (down/up), per axis (x/y) running counters.
    # ok = signs agree, bad = signs disagree, while motion exceeds triggers.
    mismatch = {
        'down_x': {'ok': 0, 'bad': 0},
        'down_y': {'ok': 0, 'bad': 0},
        'up_x'  : {'ok': 0, 'bad': 0},
        'up_y'  : {'ok': 0, 'bad': 0},
    }
    MISMATCH_DECAY = 0.995               # per sample
    MISMATCH_VERDICT_MIN = 20            # min samples before verdict
    MISMATCH_VERDICT_RATIO = 3.0         # bad/ok ratio to declare WRONG

    # Mounting status banner (top of figure)
    status_text = fig.text(
        0.5, 0.955, '', ha='center', va='center',
        fontsize=10, fontweight='bold', color=TEXT_COLOR,
        bbox=dict(boxstyle='round,pad=0.4', facecolor=PANEL_COLOR,
                  edgecolor=GRID_COLOR, linewidth=1))

    def axis_verdict(key):
        m = mismatch[key]
        total = m['ok'] + m['bad']
        if total < MISMATCH_VERDICT_MIN:
            return 'idle'
        if m['bad'] > MISMATCH_VERDICT_RATIO * max(m['ok'], 1e-3):
            return 'wrong'
        if m['ok'] > MISMATCH_VERDICT_RATIO * max(m['bad'], 1e-3):
            return 'ok'
        return 'idle'

    # =========================================================================
    # Status bar
    # =========================================================================
    chip_id_text = fig.text(0.96, 0.035, 'Chip ID: ---', fontsize=7,
                            ha='right', color=DIM_TEXT)
    fps_text = fig.text(0.96, 0.015, '', fontsize=8, ha='right',
                        color=DIM_TEXT)

    # =========================================================================
    # Buttons
    # =========================================================================
    ax_toggle = fig.add_axes([0.005, 0.005, 0.08, 0.04])
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
                btn_toggle.color = BTN_GREEN
                btn_toggle.hovercolor = BTN_GREEN_HOV
                ax_toggle.set_facecolor(BTN_GREEN)
            else:
                send_log_class_command(g_serial, LOG_CLASS_POSITION_OPTFLOW)
                g_logging_active = True
                btn_toggle.label.set_text('Stop Log')
                btn_toggle.color = BTN_RED
                btn_toggle.hovercolor = BTN_RED_HOV
                ax_toggle.set_facecolor(BTN_RED)

    btn_toggle.on_clicked(on_toggle)

    ax_reset_btn = fig.add_axes([0.09, 0.005, 0.08, 0.04])
    btn_reset = Button(ax_reset_btn, 'Reset FC',
                       color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_reset.label.set_color(TEXT_COLOR)
    btn_reset.label.set_fontsize(8)

    def on_reset(event):
        global g_logging_active
        if g_serial and g_serial.is_open:
            send_reset_command(g_serial)
            g_logging_active = False
            btn_toggle.label.set_text('Start Log')
            btn_toggle.color = BTN_GREEN
            btn_toggle.hovercolor = BTN_GREEN_HOV
            ax_toggle.set_facecolor(BTN_GREEN)

            def _post_reset():
                time.sleep(2.0)
                if g_serial and g_serial.is_open:
                    g_serial.reset_input_buffer()
                    send_log_class_command(g_serial, LOG_CLASS_HEART_BEAT)

            threading.Thread(target=_post_reset, daemon=True).start()

    btn_reset.on_clicked(on_reset)

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

        ddx, ddy, udx, udy, rng, baro, acc_x, acc_y = latest

        now = time.time()
        if t0[0] is None:
            t0[0] = now
        t = now - t0[0]

        # =================================================================
        # Update reference velocity from body-frame accel (leaky integral)
        # =================================================================
        if last_acc_t[0] is None:
            dt_a = 0.04
        else:
            dt_a = max(1e-3, min(0.2, now - last_acc_t[0]))
        last_acc_t[0] = now
        decay = np.exp(-dt_a / VEL_LEAK_TAU)
        ref_vel['x'] = ref_vel['x'] * decay + acc_x * dt_a
        ref_vel['y'] = ref_vel['y'] * decay + acc_y * dt_a

        # =================================================================
        # Mounting check: per direction & axis sign comparison
        #   Body frame: X = Forward (pitch), Y = Right (roll)
        #   Downward optflow uses dx,  dy  directly (in C: vy = +dy)
        #   Upward   optflow uses dx, -dy  (in C: vy = -dy due to mirror)
        # =================================================================
        flow_signed = {
            'down_x': ddx,
            'down_y': ddy,
            'up_x'  : udx,
            'up_y'  : -udy,
        }
        ref_axis = {'down_x': ref_vel['x'], 'down_y': ref_vel['y'],
                    'up_x'  : ref_vel['x'], 'up_y'  : ref_vel['y']}
        for key, fv in flow_signed.items():
            rv = ref_axis[key]
            # decay counters every sample so old flights age out
            mismatch[key]['ok']  *= MISMATCH_DECAY
            mismatch[key]['bad'] *= MISMATCH_DECAY
            if abs(fv) >= FLOW_TRIGGER and abs(rv) >= VEL_TRIGGER:
                if (fv > 0) == (rv > 0):
                    mismatch[key]['ok'] += 1.0
                else:
                    mismatch[key]['bad'] += 1.0

        # =================================================================
        # Update mounting status banner
        # =================================================================
        wrong_axes = []
        any_active = False
        for key in flow_signed:
            v = axis_verdict(key)
            if v == 'wrong':
                wrong_axes.append(key.replace('_', '-').upper())
            if v != 'idle':
                any_active = True
        if wrong_axes:
            status_text.set_text(
                'MOUNTING WRONG \u2014 re-mount optical flow module: ' +
                ', '.join(wrong_axes))
            status_text.set_color('#ffffff')
            status_text.get_bbox_patch().set_facecolor('#a02020')
            status_text.get_bbox_patch().set_edgecolor('#ff6060')
        elif any_active:
            status_text.set_text('Mounting check: OK \u2014 flow signs match motion')
            status_text.set_color('#a0e0a0')
            status_text.get_bbox_patch().set_facecolor('#1e3a1e')
            status_text.get_bbox_patch().set_edgecolor('#3d7a3d')
        else:
            status_text.set_text(
                'Mounting check: idle \u2014 slide drone forward / sideways to test')
            status_text.set_color(DIM_TEXT)
            status_text.get_bbox_patch().set_facecolor(PANEL_COLOR)
            status_text.get_bbox_patch().set_edgecolor(GRID_COLOR)
        updated.append(status_text)

        hist['t'].append(t)
        hist['ddx'].append(ddx)
        hist['ddy'].append(ddy)
        hist['udx'].append(udx)
        hist['udy'].append(udy)
        hist['range'].append(rng)
        hist['baro'].append(baro)
        if len(hist['t']) > HISTORY_LEN:
            for k in hist:
                hist[k] = hist[k][-HISTORY_LEN:]

        ta = np.array(hist['t'])

        # =================================================================
        # Optical Flow Downward
        # =================================================================
        line_ddx.set_data(ta, hist['ddx'])
        line_ddy.set_data(ta, hist['ddy'])
        vdx = axis_verdict('down_x')
        vdy = axis_verdict('down_y')
        tag = lambda v: ('OK' if v == 'ok' else
                         'WRONG' if v == 'wrong' else '\u2014')
        val_text_down.set_text(
            f'dx:{ddx:+.4f} [{tag(vdx)}]   dy:{ddy:+.4f} [{tag(vdy)}]   '
            f'vref x:{ref_vel["x"]:+.2f} y:{ref_vel["y"]:+.2f}')
        if len(ta) > 1:
            ax_down.set_xlim(ta[0], ta[-1] + 0.1)
            d = np.array(hist['ddx'] + hist['ddy'])
            dmin, dmax = d.min(), d.max()
            m = max(0.01, (dmax - dmin) * 0.2)
            ax_down.set_ylim(dmin - m, dmax + m)
        updated.extend([line_ddx, line_ddy])

        # =================================================================
        # Optical Flow Upward
        # =================================================================
        line_udx.set_data(ta, hist['udx'])
        line_udy.set_data(ta, hist['udy'])
        vux = axis_verdict('up_x')
        vuy = axis_verdict('up_y')
        val_text_up.set_text(
            f'dx:{udx:+.4f} [{tag(vux)}]   dy:{udy:+.4f} [{tag(vuy)}]   '
            f'vref x:{ref_vel["x"]:+.2f} y:{ref_vel["y"]:+.2f}')
        if len(ta) > 1:
            ax_up.set_xlim(ta[0], ta[-1] + 0.1)
            d = np.array(hist['udx'] + hist['udy'])
            dmin, dmax = d.min(), d.max()
            m = max(0.01, (dmax - dmin) * 0.2)
            ax_up.set_ylim(dmin - m, dmax + m)
        updated.extend([line_udx, line_udy])

        # =================================================================
        # Altitude Sensors
        # =================================================================
        line_range.set_data(ta, hist['range'])
        line_baro.set_data(ta, hist['baro'])
        val_text_alt.set_text(f'Range:{rng:+.3f}  Baro:{baro:+.3f}')
        if len(ta) > 1:
            ax_alt.set_xlim(ta[0], ta[-1] + 0.1)
            d = np.array(hist['range'] + hist['baro'])
            dmin, dmax = d.min(), d.max()
            m = max(0.1, (dmax - dmin) * 0.2)
            ax_alt.set_ylim(dmin - m, dmax + m)
        updated.extend([line_range, line_baro])

        # =================================================================
        # Chip ID + FPS
        # =================================================================
        if g_chip_id is not None:
            chip_id_text.set_text(f'Chip ID: {g_chip_id}')
            updated.append(chip_id_text)

        frame_count[0] += 1
        elapsed = now - last_fps_time[0]
        if elapsed >= 1.0:
            fps = frame_count[0] / elapsed
            fps_text.set_text(f'{fps:.0f} Hz')
            frame_count[0] = 0
            last_fps_time[0] = now
            updated.append(fps_text)

        return updated

    ani = FuncAnimation(fig, update, interval=40, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
