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
Mix Control Quadcopter Test Tool

Connects to the FC via serial, sends LOG_CLASS_MIX_CONTROL,
and displays real-time motor speeds.

Motor layout (top view, nose up):
  m1(FL,CW)    m2(FR,CCW)
       \\  ^  /
        \\ | /
         X-X
        /   \\
       /     \\
  m4(BL,CCW)   m3(BR,CW)

Layer 2 (coaxial): m5(FL,CCW) m6(FR,CW) m7(BR,CCW) m8(BL,CW)

Usage:
  python3 mix_control_quadcopter_test.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 19200
SEND_LOG_ID = 0x00

# Log class constants (match messages.h)
LOG_CLASS_NONE        = 0x00
LOG_CLASS_MIX_CONTROL = 0x11
DB_CMD_LOG_CLASS      = 0x03
DB_CMD_RESET          = 0x07

# --- Motor Parameters (must match mix_control.c) ---
MIN_SPEED = 150
MAX_SPEED = 1800

# --- Dark Theme Colors ---
BG_COLOR      = '#1e1e1e'
PANEL_COLOR   = '#252526'
TEXT_COLOR     = '#cccccc'
DIM_TEXT       = '#888888'
GRID_COLOR     = '#3c3c3c'
BAR_L1_COLOR   = '#5599ff'   # Layer 1 motors (m1-m4)
BAR_L2_COLOR   = '#ff9955'   # Layer 2 motors (m5-m8)
DRONE_COLOR    = '#555555'
NOSE_COLOR     = '#ff5555'
CLAMP_COLOR    = '#ff3333'
BTN_GREEN      = '#2d5a2d'
BTN_GREEN_HOV  = '#3d7a3d'
BTN_RED        = '#5a2d2d'
BTN_RED_HOV    = '#7a3d3d'

# --- Motor positions (top view, X=right, Y=forward) ---
MOTOR_POS = {1: (-1, 1), 2: (1, 1), 3: (1, -1), 4: (-1, -1)}
MOTOR_POS_L2 = {5: (-1, 1), 6: (1, 1), 7: (1, -1), 8: (-1, -1)}

MOTOR_LABELS_L1 = {1: 'M1\nFL CW', 2: 'M2\nFR CCW', 3: 'M3\nBR CW', 4: 'M4\nBL CCW'}
MOTOR_LABELS_L2 = {5: 'M5\nFL CCW', 6: 'M6\nFR CW', 7: 'M7\nBR CCW', 8: 'M8\nBL CW'}

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
    names = {0x00: 'NONE', 0x11: 'MIX_CONTROL'}
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


def serial_reader():
    """Background thread: reads DB frames from FC, parses 8-float payloads."""
    global g_serial
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

        while True:
            b1 = ser.read(1)
            if not b1: continue
            if b1[0] != 0x62 and b1[0] != 0x64: continue

            b2 = ser.read(1)
            if not b2: continue
            if not ((b1[0] == 0x64 and b2[0] == 0x62) or (b1[0] == 0x62 and b2[0] == 0x64)):
                continue

            id_byte = ser.read(1)
            if not id_byte: continue
            msg_id = id_byte[0]

            class_byte = ser.read(1)
            if not class_byte: continue

            len_bytes = ser.read(2)
            if len(len_bytes) < 2: continue
            length = int.from_bytes(len_bytes, 'little')
            if length > 1024: continue

            payload = ser.read(length)
            if len(payload) != length: continue
            _ = ser.read(2)  # checksum

            if msg_id == SEND_LOG_ID and length == 32:
                vals = struct.unpack('<8f', payload)
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

    fig = plt.figure(figsize=(14, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('Mix Control — Quadcopter', fontsize=16,
                 color=TEXT_COLOR, fontweight='bold', y=0.97)

    # --- Left: Motor diagram (top view) ---
    ax_diagram = fig.add_axes([0.03, 0.25, 0.43, 0.65])
    ax_diagram.set_facecolor(BG_COLOR)
    ax_diagram.set_aspect('equal')
    ax_diagram.set_xlim(-2.5, 2.5)
    ax_diagram.set_ylim(-2.5, 2.5)
    ax_diagram.set_xticks([])
    ax_diagram.set_yticks([])
    ax_diagram.set_title('Top View (nose up)', color=DIM_TEXT, fontsize=11)
    for spine in ax_diagram.spines.values():
        spine.set_visible(False)

    for pos in MOTOR_POS.values():
        ax_diagram.plot([0, pos[0]], [0, pos[1]], color=DRONE_COLOR, lw=3, zorder=1)
    ax_diagram.annotate('', xy=(0, 1.6), xytext=(0, 0.3),
                        arrowprops=dict(arrowstyle='->', color=NOSE_COLOR, lw=2.5))
    ax_diagram.text(0, 1.75, 'NOSE', ha='center', va='bottom',
                    color=NOSE_COLOR, fontsize=9, fontweight='bold')

    motor_circles_l1 = {}
    motor_speed_texts_l1 = {}
    motor_circles_l2 = {}
    motor_speed_texts_l2 = {}

    circle_r = 0.38
    for m, (x, y) in MOTOR_POS.items():
        c = plt.Circle((x, y), circle_r, fill=True, fc=BAR_L1_COLOR, ec='white',
                        lw=1.5, alpha=0.3, zorder=2)
        ax_diagram.add_patch(c)
        motor_circles_l1[m] = c
        ax_diagram.text(x, y + 0.08, MOTOR_LABELS_L1[m],
                        ha='center', va='center', fontsize=7, color=TEXT_COLOR, zorder=3)
        motor_speed_texts_l1[m] = ax_diagram.text(x, y - 0.28, '0',
                                                   ha='center', va='center', fontsize=10,
                                                   color='white', fontweight='bold', zorder=3)

    for m, (x, y) in MOTOR_POS_L2.items():
        c = plt.Circle((x, y), circle_r + 0.12, fill=False, ec=BAR_L2_COLOR,
                        lw=2, alpha=0.3, zorder=2, ls='--')
        ax_diagram.add_patch(c)
        motor_circles_l2[m] = c
        ax_diagram.text(x, y - 0.52, MOTOR_LABELS_L2[m],
                        ha='center', va='center', fontsize=6, color=BAR_L2_COLOR, alpha=0.7, zorder=3)
        motor_speed_texts_l2[m] = ax_diagram.text(x, y - 0.82, '0',
                                                    ha='center', va='center', fontsize=9,
                                                    color=BAR_L2_COLOR, fontweight='bold', zorder=3)

    # --- Right: Bar chart ---
    ax_bars = fig.add_axes([0.55, 0.25, 0.42, 0.65])
    ax_bars.set_facecolor(BG_COLOR)
    ax_bars.set_title('Motor Speeds', color=DIM_TEXT, fontsize=11)
    ax_bars.set_ylim(0, MAX_SPEED + 100)
    ax_bars.set_ylabel('Speed', color=DIM_TEXT)
    ax_bars.axhline(y=MIN_SPEED, color='#555555', ls='--', lw=1, label=f'MIN ({MIN_SPEED})')
    ax_bars.axhline(y=MAX_SPEED, color='#553333', ls='--', lw=1, label=f'MAX ({MAX_SPEED})')
    ax_bars.legend(loc='upper right', fontsize=8, facecolor=PANEL_COLOR, edgecolor=GRID_COLOR)

    bar_labels = ['M1\nFL', 'M2\nFR', 'M3\nBR', 'M4\nBL', 'M5\nFL', 'M6\nFR', 'M7\nBR', 'M8\nBL']
    bar_colors = [BAR_L1_COLOR]*4 + [BAR_L2_COLOR]*4
    x_pos = np.arange(8)
    bars = ax_bars.bar(x_pos, [0]*8, color=bar_colors, alpha=0.8, width=0.7)
    ax_bars.set_xticks(x_pos)
    ax_bars.set_xticklabels(bar_labels, fontsize=8)
    bar_value_texts = []
    for i in range(8):
        t = ax_bars.text(i, 10, '0', ha='center', va='bottom', fontsize=9,
                         color='white', fontweight='bold')
        bar_value_texts.append(t)

    # --- Status text ---
    status_text = fig.text(0.50, 0.01, '', ha='center', va='bottom', fontsize=9, color=DIM_TEXT)

    # Common update function: takes a dict {1..8: speed}
    def update_display(speeds):
        for i, m in enumerate(range(1, 9)):
            spd = speeds[m]
            bars[i].set_height(spd)
            bar_value_texts[i].set_text(str(spd))
            bar_value_texts[i].set_position((i, spd + 15))
            clamped = (spd <= MIN_SPEED and spd != 0) or spd >= MAX_SPEED
            if clamped:
                bars[i].set_alpha(1.0)
                bars[i].set_edgecolor(CLAMP_COLOR)
                bars[i].set_linewidth(2)
                bar_value_texts[i].set_color(CLAMP_COLOR)
            else:
                bars[i].set_alpha(0.8)
                bars[i].set_edgecolor('none')
                bars[i].set_linewidth(0)
                bar_value_texts[i].set_color('white')

        for m in range(1, 5):
            alpha = max(0.15, speeds[m] / MAX_SPEED) if MAX_SPEED > 0 else 0.15
            motor_circles_l1[m].set_alpha(alpha)
            motor_speed_texts_l1[m].set_text(str(speeds[m]))
            motor_speed_texts_l1[m].set_color(
                CLAMP_COLOR if speeds[m] >= MAX_SPEED else 'white')
        for m in range(5, 9):
            alpha = max(0.15, speeds[m] / MAX_SPEED) if MAX_SPEED > 0 else 0.15
            motor_circles_l2[m].set_alpha(alpha)
            motor_speed_texts_l2[m].set_text(str(speeds[m]))
            motor_speed_texts_l2[m].set_color(
                CLAMP_COLOR if speeds[m] >= MAX_SPEED else BAR_L2_COLOR)

    g_live_state = {'logging': False}

    ax_log_btn = fig.add_axes([0.03, 0.01, 0.12, 0.04])
    btn_log = Button(ax_log_btn, 'Start Log', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_log.label.set_color(TEXT_COLOR)
    btn_log.label.set_fontsize(8)

    def toggle_logging(event):
        global g_logging_active
        if not g_serial:
            status_text.set_text('No serial connection')
            status_text.set_color(CLAMP_COLOR)
            fig.canvas.draw_idle()
            return
        if g_live_state['logging']:
            send_log_class_command(g_serial, LOG_CLASS_NONE)
            g_live_state['logging'] = False
            btn_log.label.set_text('Start Log')
            btn_log.color = BTN_GREEN
            btn_log.hovercolor = BTN_GREEN_HOV
            status_text.set_text('Logging stopped')
            status_text.set_color(DIM_TEXT)
        else:
            send_log_class_command(g_serial, LOG_CLASS_MIX_CONTROL)
            g_live_state['logging'] = True
            btn_log.label.set_text('Stop Log')
            btn_log.color = BTN_RED
            btn_log.hovercolor = BTN_RED_HOV
            status_text.set_text('Logging active — waiting for data...')
            status_text.set_color('#55dd55')
        fig.canvas.draw_idle()

    btn_log.on_clicked(toggle_logging)

    # Reset FC button
    ax_reset_btn = fig.add_axes([0.16, 0.01, 0.10, 0.04])
    btn_reset = Button(ax_reset_btn, 'Reset FC', color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_reset.label.set_color(TEXT_COLOR)
    btn_reset.label.set_fontsize(8)

    def reset_fc(event):
        if g_serial and g_serial.is_open:
            send_reset_command(g_serial)
            g_live_state['logging'] = False
            btn_log.label.set_text('Start Log')
            btn_log.color = BTN_GREEN
            btn_log.hovercolor = BTN_GREEN_HOV
            fig.canvas.draw_idle()

    btn_reset.on_clicked(reset_fc)

    def live_update(frame):
        """Called by FuncAnimation to drain the serial queue."""
        updated = False
        latest = None
        while not data_queue.empty():
            latest = data_queue.get_nowait()
            updated = True
        if updated and latest is not None:
            speeds = {i+1: int(latest[i]) for i in range(8)}
            update_display(speeds)
            total = sum(speeds.values())
            status_text.set_text(
                f'M1={speeds[1]}  M2={speeds[2]}  M3={speeds[3]}  M4={speeds[4]}  '
                f'M5={speeds[5]}  M6={speeds[6]}  M7={speeds[7]}  M8={speeds[8]}  '
                f'sum={total}')
            status_text.set_color('#55dd55')
        return []

    # Initial empty display
    update_display({m: 0 for m in range(1, 9)})

    anim = FuncAnimation(fig, live_update, interval=100, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
