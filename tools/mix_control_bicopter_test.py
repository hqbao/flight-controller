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

"""
Mix Control Bicopter Test Tool

Connects to the FC via serial, sends LOG_CLASS_MIX_CONTROL,
and displays real-time motor speeds and servo angles.

Bicopter layout (top view, nose up):

   M1(L,CCW)     ^     M2(R,CW)
   S1(L tilt)    |     S2(R tilt)
       ----------+----------
                 |

Output array:
  [0] = left motor  (DShot 150..1800)
  [1] = right motor (DShot 150..1800)
  [2..3] = 0 (unused, TIM1 CH3-4 reserved for DShot)
  [4] = left servo  (PWM 1000..2000 µs)  — Port5 TIM2 CH1
  [5] = right servo (PWM 1000..2000 µs)  — Port6 TIM2 CH2
  [4..7] = 0 (unused)

Usage:
  python3 mix_control_bicopter_test.py
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 38400
SEND_LOG_ID = 0x00

# Log class constants (match messages.h)
LOG_CLASS_NONE         = 0x00
LOG_CLASS_HEART_BEAT   = 0x09
LOG_CLASS_MIX_CONTROL  = 0x11
DB_CMD_LOG_CLASS       = 0x03
DB_CMD_RESET          = 0x07

# --- Motor / Servo Parameters (must match bicopter.c) ---
MIN_SPEED    = 150
MAX_SPEED    = 1800
SERVO_MIN    = 1000
SERVO_MAX    = 2000
SERVO_CENTER = 1500

# --- Dark Theme Colors ---
BG_COLOR      = '#1e1e1e'
PANEL_COLOR   = '#252526'
TEXT_COLOR     = '#cccccc'
DIM_TEXT       = '#888888'
GRID_COLOR     = '#3c3c3c'
MOTOR_COLOR    = '#5599ff'
SERVO_COLOR    = '#ff9955'
BODY_COLOR     = '#555555'
NOSE_COLOR     = '#ff5555'
CLAMP_COLOR    = '#ff3333'
PROP_COLOR     = '#88bbff'
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
        send_log_class_command(ser, LOG_CLASS_HEART_BEAT)

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


def servo_to_angle_deg(pwm_us):
    """Convert servo PWM (1000-2000 µs) to tilt angle in degrees (-45 to +45)."""
    return (pwm_us - SERVO_CENTER) / (SERVO_MAX - SERVO_CENTER) * 45.0


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
    fig.suptitle('Mix Control \u2014 Bicopter', fontsize=16,
                 color=TEXT_COLOR, fontweight='bold', y=0.97)

    # ============================
    # LEFT: Top-view diagram
    # ============================
    ax_diagram = fig.add_axes([0.03, 0.25, 0.50, 0.65])
    ax_diagram.set_facecolor(BG_COLOR)
    ax_diagram.set_aspect('equal')
    ax_diagram.set_xlim(-3.5, 3.5)
    ax_diagram.set_ylim(-2.5, 3.0)
    ax_diagram.set_xticks([])
    ax_diagram.set_yticks([])
    ax_diagram.set_title('Top View (nose up)', color=DIM_TEXT, fontsize=11)
    for spine in ax_diagram.spines.values():
        spine.set_visible(False)

    # Body: horizontal beam (wing) + fuselage
    ax_diagram.plot([-2.5, 2.5], [0, 0], color=BODY_COLOR, lw=5, zorder=1, solid_capstyle='round')
    ax_diagram.plot([0, 0], [-1.5, 0.8], color=BODY_COLOR, lw=4, zorder=1, solid_capstyle='round')

    # Nose arrow
    ax_diagram.annotate('', xy=(0, 1.8), xytext=(0, 0.8),
                        arrowprops=dict(arrowstyle='->', color=NOSE_COLOR, lw=2.5))
    ax_diagram.text(0, 1.95, 'NOSE', ha='center', va='bottom',
                    color=NOSE_COLOR, fontsize=9, fontweight='bold')

    # Tail
    ax_diagram.plot(0, -1.5, 'o', color=BODY_COLOR, markersize=6, zorder=2)
    ax_diagram.text(0, -1.8, 'TAIL', ha='center', va='top',
                    color=DIM_TEXT, fontsize=7)

    # Motor mount positions (on the beam)
    MOUNT_L = (-2.5, 0)
    MOUNT_R = (2.5, 0)
    NACELLE_LEN = 0.6   # arm from pivot to motor center
    PROP_RADIUS = 0.8

    # Nacelle arms (lines from pivot to motor, rotate with servo)
    nacelle_l, = ax_diagram.plot([], [], color=MOTOR_COLOR, lw=4, zorder=2, solid_capstyle='round')
    nacelle_r, = ax_diagram.plot([], [], color=MOTOR_COLOR, lw=4, zorder=2, solid_capstyle='round')

    # Prop discs — from top-down, a tilted rotor looks like an ellipse
    # Width stays constant (PROP_RADIUS*2), height shrinks with cos(tilt)
    from matplotlib.patches import Ellipse
    prop_l = Ellipse(MOUNT_L, PROP_RADIUS * 2, PROP_RADIUS * 2,
                      fc=PROP_COLOR, ec='white', lw=1.5, alpha=0.15, zorder=3)
    prop_r = Ellipse(MOUNT_R, PROP_RADIUS * 2, PROP_RADIUS * 2,
                      fc=PROP_COLOR, ec='white', lw=1.5, alpha=0.15, zorder=3)
    ax_diagram.add_patch(prop_l)
    ax_diagram.add_patch(prop_r)

    # Motor center dots (move with nacelle)
    motor_dot_l, = ax_diagram.plot([], [], 'o', color=MOTOR_COLOR, markersize=7, zorder=4)
    motor_dot_r, = ax_diagram.plot([], [], 'o', color=MOTOR_COLOR, markersize=7, zorder=4)

    # Nacelle pivot markers (fixed)
    ax_diagram.plot(*MOUNT_L, 's', color=BODY_COLOR, markersize=7, zorder=4)
    ax_diagram.plot(*MOUNT_R, 's', color=BODY_COLOR, markersize=7, zorder=4)

    # Motor labels (static, above max prop extent)
    ax_diagram.text(MOUNT_L[0], MOUNT_L[1] + PROP_RADIUS + NACELLE_LEN + 0.2,
                    'M1 (L, CCW)', ha='center', va='bottom',
                    fontsize=8, color=MOTOR_COLOR, fontweight='bold', zorder=5)
    ax_diagram.text(MOUNT_R[0], MOUNT_R[1] + PROP_RADIUS + NACELLE_LEN + 0.2,
                    'M2 (R, CW)', ha='center', va='bottom',
                    fontsize=8, color=MOTOR_COLOR, fontweight='bold', zorder=5)

    # Speed value labels (updated live, follow motor center)
    speed_text_l = ax_diagram.text(MOUNT_L[0], MOUNT_L[1],
                                    '0', ha='center', va='center', fontsize=11,
                                    color='white', fontweight='bold', zorder=5)
    speed_text_r = ax_diagram.text(MOUNT_R[0], MOUNT_R[1],
                                    '0', ha='center', va='center', fontsize=11,
                                    color='white', fontweight='bold', zorder=5)

    # Servo text (below pivot)
    servo_text_l = ax_diagram.text(MOUNT_L[0], MOUNT_L[1] - PROP_RADIUS - NACELLE_LEN - 0.2,
                                    'S1: 1500 µs\n0.0°', ha='center', va='top',
                                    fontsize=8, color=SERVO_COLOR, zorder=5)
    servo_text_r = ax_diagram.text(MOUNT_R[0], MOUNT_R[1] - PROP_RADIUS - NACELLE_LEN - 0.2,
                                    'S2: 1500 µs\n0.0°', ha='center', va='top',
                                    fontsize=8, color=SERVO_COLOR, zorder=5)

    # ============================
    # RIGHT: Bar charts
    # ============================
    # Motor bar chart (top right)
    ax_motors = fig.add_axes([0.60, 0.55, 0.37, 0.35])
    ax_motors.set_facecolor(BG_COLOR)
    ax_motors.set_title('Motors (DShot)', color=MOTOR_COLOR, fontsize=10)
    ax_motors.set_ylim(0, MAX_SPEED + 200)
    ax_motors.set_ylabel('DShot value', color=DIM_TEXT, fontsize=9)
    ax_motors.axhline(y=MIN_SPEED, color='#555555', ls='--', lw=1)
    ax_motors.axhline(y=MAX_SPEED, color='#553333', ls='--', lw=1)
    ax_motors.text(1.6, MIN_SPEED + 30, f'MIN ({MIN_SPEED})', fontsize=7, color=DIM_TEXT)
    ax_motors.text(1.6, MAX_SPEED + 30, f'MAX ({MAX_SPEED})', fontsize=7, color='#885555')

    motor_x = [0, 1]
    motor_labels = ['M1\nLeft', 'M2\nRight']
    motor_bars = ax_motors.bar(motor_x, [0, 0], color=MOTOR_COLOR, alpha=0.8, width=0.6)
    ax_motors.set_xticks(motor_x)
    ax_motors.set_xticklabels(motor_labels, fontsize=9)
    motor_val_texts = []
    for i in range(2):
        txt = ax_motors.text(i, 10, '0', ha='center', va='bottom', fontsize=10,
                              color='white', fontweight='bold')
        motor_val_texts.append(txt)

    # Servo bar chart (bottom right)
    ax_servos = fig.add_axes([0.60, 0.12, 0.37, 0.35])
    ax_servos.set_facecolor(BG_COLOR)
    ax_servos.set_title('Servos (PWM)', color=SERVO_COLOR, fontsize=10)
    ax_servos.set_ylim(SERVO_MIN - 100, SERVO_MAX + 100)
    ax_servos.set_ylabel('PWM (µs)', color=DIM_TEXT, fontsize=9)
    ax_servos.axhline(y=SERVO_MIN, color='#555555', ls='--', lw=1)
    ax_servos.axhline(y=SERVO_CENTER, color='#556655', ls='--', lw=1)
    ax_servos.axhline(y=SERVO_MAX, color='#553333', ls='--', lw=1)
    ax_servos.text(1.6, SERVO_MIN + 15, f'MIN ({SERVO_MIN})', fontsize=7, color=DIM_TEXT)
    ax_servos.text(1.6, SERVO_CENTER + 15, f'CTR ({SERVO_CENTER})', fontsize=7, color='#669966')
    ax_servos.text(1.6, SERVO_MAX + 15, f'MAX ({SERVO_MAX})', fontsize=7, color='#885555')

    servo_x = [0, 1]
    servo_labels = ['S1\nLeft', 'S2\nRight']
    servo_bars = ax_servos.bar(servo_x, [SERVO_CENTER, SERVO_CENTER], color=SERVO_COLOR,
                                alpha=0.8, width=0.6)
    ax_servos.set_xticks(servo_x)
    ax_servos.set_xticklabels(servo_labels, fontsize=9)
    servo_val_texts = []
    for i in range(2):
        txt = ax_servos.text(i, SERVO_CENTER + 15, f'{SERVO_CENTER}', ha='center', va='bottom',
                              fontsize=10, color='white', fontweight='bold')
        servo_val_texts.append(txt)

    # --- Status text ---
    status_text = fig.text(0.50, 0.01, '', ha='center', va='bottom', fontsize=9, color=DIM_TEXT)

    def update_display(m1, m2, s1, s2):
        """Update all visual elements with new motor/servo values."""
        # --- Motor bars ---
        for i, spd in enumerate([m1, m2]):
            motor_bars[i].set_height(spd)
            motor_val_texts[i].set_text(str(spd))
            motor_val_texts[i].set_position((i, spd + 20))
            clamped = (spd <= MIN_SPEED and spd != 0) or spd >= MAX_SPEED
            if clamped:
                motor_bars[i].set_alpha(1.0)
                motor_bars[i].set_edgecolor(CLAMP_COLOR)
                motor_bars[i].set_linewidth(2)
                motor_val_texts[i].set_color(CLAMP_COLOR)
            else:
                motor_bars[i].set_alpha(0.8)
                motor_bars[i].set_edgecolor('none')
                motor_bars[i].set_linewidth(0)
                motor_val_texts[i].set_color('white')

        # --- Servo bars ---
        for i, pwm in enumerate([s1, s2]):
            servo_bars[i].set_height(pwm)
            servo_val_texts[i].set_text(str(pwm))
            servo_val_texts[i].set_position((i, pwm + 15))
            clamped = pwm <= SERVO_MIN or pwm >= SERVO_MAX
            if clamped:
                servo_bars[i].set_alpha(1.0)
                servo_bars[i].set_edgecolor(CLAMP_COLOR)
                servo_bars[i].set_linewidth(2)
                servo_val_texts[i].set_color(CLAMP_COLOR)
            else:
                servo_bars[i].set_alpha(0.8)
                servo_bars[i].set_edgecolor('none')
                servo_bars[i].set_linewidth(0)
                servo_val_texts[i].set_color('white')

        # --- Diagram: tilting nacelles + prop discs ---
        angle_l = servo_to_angle_deg(s1)
        angle_r = -servo_to_angle_deg(s2)  # S2 mirrored mount: negate for physical tilt
        rad_l = np.radians(angle_l)
        rad_r = np.radians(angle_r)

        # Nacelle arm endpoints (pivot → motor center)
        # Tilt is fore/aft (Y axis on screen). 0° = straight up from wing.
        # Positive angle tilts toward nose (+Y).
        mx_l = MOUNT_L[0]
        my_l = MOUNT_L[1] + NACELLE_LEN * np.sin(rad_l)
        nacelle_l.set_data([MOUNT_L[0], mx_l], [MOUNT_L[1], my_l])
        motor_dot_l.set_data([mx_l], [my_l])

        mx_r = MOUNT_R[0]
        my_r = MOUNT_R[1] + NACELLE_LEN * np.sin(rad_r)
        nacelle_r.set_data([MOUNT_R[0], mx_r], [MOUNT_R[1], my_r])
        motor_dot_r.set_data([mx_r], [my_r])

        # Prop ellipse: center follows motor, height shrinks with cos(tilt)
        # to show the disc foreshortening from top-down perspective
        prop_l.set_center((mx_l, my_l))
        prop_l.height = PROP_RADIUS * 2 * np.cos(rad_l)
        alpha_l = max(0.15, m1 / MAX_SPEED) if MAX_SPEED > 0 else 0.15
        prop_l.set_alpha(alpha_l)

        prop_r.set_center((mx_r, my_r))
        prop_r.height = PROP_RADIUS * 2 * np.cos(rad_r)
        alpha_r = max(0.15, m2 / MAX_SPEED) if MAX_SPEED > 0 else 0.15
        prop_r.set_alpha(alpha_r)

        # Speed text follows motor center
        speed_text_l.set_position((mx_l, my_l))
        speed_text_l.set_text(str(m1))
        speed_text_l.set_color(CLAMP_COLOR if m1 >= MAX_SPEED else 'white')

        speed_text_r.set_position((mx_r, my_r))
        speed_text_r.set_text(str(m2))
        speed_text_r.set_color(CLAMP_COLOR if m2 >= MAX_SPEED else 'white')

        # Servo text (show physical tilt angle)
        servo_text_l.set_text(f'S1: {s1} µs\n{angle_l:+.1f}°')
        servo_text_r.set_text(f'S2: {s2} µs\n{angle_r:+.1f}°')

    # --- Buttons ---
    g_live_state = {'logging': False}

    ax_log_btn = fig.add_axes([0.03, 0.01, 0.12, 0.04])
    btn_log = Button(ax_log_btn, 'Start Log', color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_log.label.set_color(TEXT_COLOR)
    btn_log.label.set_fontsize(8)

    def toggle_logging(event):
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
            status_text.set_text('Logging active \u2014 waiting for data...')
            status_text.set_color('#55dd55')
        fig.canvas.draw_idle()

    btn_log.on_clicked(toggle_logging)

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

            def _post_reset():
                time.sleep(2.0)
                if g_serial and g_serial.is_open:
                    g_serial.reset_input_buffer()
                    send_log_class_command(g_serial, LOG_CLASS_HEART_BEAT)

            threading.Thread(target=_post_reset, daemon=True).start()

    btn_reset.on_clicked(reset_fc)

    def live_update(frame):
        """Called by FuncAnimation to drain the serial queue."""
        updated = False
        latest = None
        while not data_queue.empty():
            latest = data_queue.get_nowait()
            updated = True
        if updated and latest is not None:
            m1 = int(latest[0])
            m2 = int(latest[1])
            s1 = int(latest[4])
            s2 = int(latest[5])
            update_display(m1, m2, s1, s2)
            angle_l = servo_to_angle_deg(s1)
            angle_r = -servo_to_angle_deg(s2)  # S2 mirrored
            status_text.set_text(
                f'M1={m1}  M2={m2}  S1={s1}\u00b5s ({angle_l:+.1f}\u00b0)  '
                f'S2={s2}\u00b5s ({angle_r:+.1f}\u00b0)')
            status_text.set_color('#55dd55')
        return []

    # Initial display
    update_display(0, 0, SERVO_CENTER, SERVO_CENTER)

    anim = FuncAnimation(fig, live_update, interval=100, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
