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
Position Estimation Time-Series Charts

Real-time time-series display of all 6 position/velocity channels in a 2x2 grid:
  Top-Left:     Pos X (Pitch) & Pos Y (Roll)
  Top-Right:    Pos Z (Altitude)
  Bottom-Left:  Vel X (Pitch) & Vel Y (Roll)
  Bottom-Right: Vel Z (Vertical Velocity)

Useful for tuning PID gains and checking sensor fusion stability.

Frame layout (LOG_CLASS_POSITION = 0x04, 24 bytes):
  float[0] = Pos X (Pitch/Forward, meters)
  float[1] = Pos Y (Roll/Right, meters)
  float[2] = Pos Z (Altitude, meters)
  float[3] = Vel X (Pitch, m/s)
  float[4] = Vel Y (Roll, m/s)
  float[5] = Vel Z (Vertical, m/s)

Usage:
  python3 position_estimation_chart.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 19200
SEND_LOG_ID = 0x00

LOG_CLASS_NONE      = 0x00
LOG_CLASS_HEART_BEAT = 0x09
LOG_CLASS_POSITION  = 0x04
DB_CMD_LOG_CLASS    = 0x03
DB_CMD_RESET       = 0x07
DB_CMD_CHIP_ID     = 0x09

POSITION_FRAME_SIZE = 24  # 6 floats
HISTORY_LEN = 300         # ~30s at 10 Hz

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
ACCENT_YELLOW  = '#ffcc55'
ACCENT_CYAN    = '#55cccc'
BTN_GREEN      = '#2d5a2d'
BTN_GREEN_HOV  = '#3d7a3d'
BTN_RED        = '#5a2d2d'
BTN_RED_HOV    = '#7a3d3d'

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
    names = {0x00: 'NONE', 0x04: 'POSITION'}
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
    """Background thread: reads DB frames from FC, parses position payloads."""
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
            elif msg_id == SEND_LOG_ID and length == POSITION_FRAME_SIZE:
                vals = struct.unpack('<6f', payload)
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

    fig, ((ax_pos_xy, ax_pos_z), (ax_vel_xy, ax_vel_z)) = plt.subplots(
        2, 2, figsize=(16, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('Position Estimation \u2014 Time Series', fontsize=14,
                 color=TEXT_COLOR, fontweight='bold', y=0.99)
    fig.subplots_adjust(left=0.07, right=0.97, top=0.93, bottom=0.10,
                        hspace=0.35, wspace=0.25)

    # =========================================================================
    # Top-Left: Pos X & Pos Y
    # =========================================================================
    ax_pos_xy.set_title('XY Position', color=DIM_TEXT, fontsize=10, pad=6)
    ax_pos_xy.set_ylabel('Position (m)', fontsize=8)
    ax_pos_xy.grid(True, alpha=0.2, linestyle=':')
    ax_pos_xy.tick_params(labelsize=7)
    line_px, = ax_pos_xy.plot([], [], '-', color=ACCENT_BLUE, linewidth=1.2,
                              label='X (Pitch)')
    line_py, = ax_pos_xy.plot([], [], '-', color=ACCENT_GREEN, linewidth=1.2,
                              label='Y (Roll)')
    ax_pos_xy.legend(loc='upper left', fontsize=7, framealpha=0.3)
    val_text_pos_xy = ax_pos_xy.text(
        0.98, 0.95, '', transform=ax_pos_xy.transAxes,
        fontsize=8, ha='right', va='top', fontfamily='monospace',
        color=TEXT_COLOR)

    # =========================================================================
    # Top-Right: Pos Z (Altitude)
    # =========================================================================
    ax_pos_z.set_title('Altitude (Z)', color=DIM_TEXT, fontsize=10, pad=6)
    ax_pos_z.set_ylabel('Altitude (m)', fontsize=8)
    ax_pos_z.grid(True, alpha=0.2, linestyle=':')
    ax_pos_z.tick_params(labelsize=7)
    line_pz, = ax_pos_z.plot([], [], '-', color=ACCENT_YELLOW, linewidth=1.2,
                             label='Z')
    ax_pos_z.legend(loc='upper left', fontsize=7, framealpha=0.3)
    val_text_pos_z = ax_pos_z.text(
        0.98, 0.95, '', transform=ax_pos_z.transAxes,
        fontsize=8, ha='right', va='top', fontfamily='monospace',
        color=TEXT_COLOR)

    # =========================================================================
    # Bottom-Left: Vel X & Vel Y
    # =========================================================================
    ax_vel_xy.set_title('XY Velocity', color=DIM_TEXT, fontsize=10, pad=6)
    ax_vel_xy.set_ylabel('Velocity (m/s)', fontsize=8)
    ax_vel_xy.set_xlabel('Time (s)', fontsize=8)
    ax_vel_xy.grid(True, alpha=0.2, linestyle=':')
    ax_vel_xy.tick_params(labelsize=7)
    line_vx, = ax_vel_xy.plot([], [], '-', color=ACCENT_RED, linewidth=1.2,
                              label='Vx (Pitch)')
    line_vy, = ax_vel_xy.plot([], [], '-', color=ACCENT_ORANGE, linewidth=1.2,
                              label='Vy (Roll)')
    ax_vel_xy.legend(loc='upper left', fontsize=7, framealpha=0.3)
    val_text_vel_xy = ax_vel_xy.text(
        0.98, 0.95, '', transform=ax_vel_xy.transAxes,
        fontsize=8, ha='right', va='top', fontfamily='monospace',
        color=TEXT_COLOR)

    # =========================================================================
    # Bottom-Right: Vel Z
    # =========================================================================
    ax_vel_z.set_title('Vertical Velocity', color=DIM_TEXT, fontsize=10, pad=6)
    ax_vel_z.set_ylabel('Velocity (m/s)', fontsize=8)
    ax_vel_z.set_xlabel('Time (s)', fontsize=8)
    ax_vel_z.grid(True, alpha=0.2, linestyle=':')
    ax_vel_z.tick_params(labelsize=7)
    line_vz, = ax_vel_z.plot([], [], '-', color=ACCENT_CYAN, linewidth=1.2,
                             label='Vz')
    ax_vel_z.legend(loc='upper left', fontsize=7, framealpha=0.3)
    val_text_vel_z = ax_vel_z.text(
        0.98, 0.95, '', transform=ax_vel_z.transAxes,
        fontsize=8, ha='right', va='top', fontfamily='monospace',
        color=TEXT_COLOR)

    # History buffer
    h = {'t': [], 'px': [], 'py': [], 'pz': [], 'vx': [], 'vy': [], 'vz': []}
    t0 = [None]

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
                send_log_class_command(g_serial, LOG_CLASS_POSITION)
                g_logging_active = True
                btn_toggle.label.set_text('Stop Log')
                btn_toggle.color = BTN_RED
                btn_toggle.hovercolor = BTN_RED_HOV
                ax_toggle.set_facecolor(BTN_RED)

    btn_toggle.on_clicked(on_toggle)

    ax_reset = fig.add_axes([0.09, 0.005, 0.08, 0.04])
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

        px, py, pz, vx, vy, vz = latest

        now = time.time()
        if t0[0] is None:
            t0[0] = now
        t = now - t0[0]

        h['t'].append(t)
        h['px'].append(px)
        h['py'].append(py)
        h['pz'].append(pz)
        h['vx'].append(vx)
        h['vy'].append(vy)
        h['vz'].append(vz)
        if len(h['t']) > HISTORY_LEN:
            for k in h:
                h[k] = h[k][-HISTORY_LEN:]

        ta = np.array(h['t'])

        # =================================================================
        # Pos XY
        # =================================================================
        line_px.set_data(ta, h['px'])
        line_py.set_data(ta, h['py'])
        val_text_pos_xy.set_text(f'X:{px:+.3f}  Y:{py:+.3f}')
        if len(ta) > 1:
            ax_pos_xy.set_xlim(ta[0], ta[-1] + 0.1)
            d = np.array(h['px'] + h['py'])
            dmin, dmax = d.min(), d.max()
            m = max(0.1, (dmax - dmin) * 0.2)
            ax_pos_xy.set_ylim(dmin - m, dmax + m)
        updated.extend([line_px, line_py])

        # =================================================================
        # Pos Z
        # =================================================================
        line_pz.set_data(ta, h['pz'])
        val_text_pos_z.set_text(f'Z:{pz:+.3f}')
        if len(ta) > 1:
            ax_pos_z.set_xlim(ta[0], ta[-1] + 0.1)
            d = np.array(h['pz'])
            dmin, dmax = d.min(), d.max()
            m = max(0.1, (dmax - dmin) * 0.2)
            ax_pos_z.set_ylim(dmin - m, dmax + m)
        updated.append(line_pz)

        # =================================================================
        # Vel XY
        # =================================================================
        line_vx.set_data(ta, h['vx'])
        line_vy.set_data(ta, h['vy'])
        val_text_vel_xy.set_text(f'Vx:{vx:+.3f}  Vy:{vy:+.3f}')
        if len(ta) > 1:
            ax_vel_xy.set_xlim(ta[0], ta[-1] + 0.1)
            d = np.array(h['vx'] + h['vy'])
            dmin, dmax = d.min(), d.max()
            m = max(0.1, (dmax - dmin) * 0.2)
            ax_vel_xy.set_ylim(dmin - m, dmax + m)
        updated.extend([line_vx, line_vy])

        # =================================================================
        # Vel Z
        # =================================================================
        line_vz.set_data(ta, h['vz'])
        val_text_vel_z.set_text(f'Vz:{vz:+.3f}')
        if len(ta) > 1:
            ax_vel_z.set_xlim(ta[0], ta[-1] + 0.1)
            d = np.array(h['vz'])
            dmin, dmax = d.min(), d.max()
            m = max(0.1, (dmax - dmin) * 0.2)
            ax_vel_z.set_ylim(dmin - m, dmax + m)
        updated.append(line_vz)

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

    ani = FuncAnimation(fig, update, interval=33, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
