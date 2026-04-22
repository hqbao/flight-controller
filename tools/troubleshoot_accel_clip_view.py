import serial
import serial.tools.list_ports
import struct
import threading
import queue
import sys
import time
import matplotlib
matplotlib.use('macosx' if sys.platform == 'darwin' else 'TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation

"""
Accelerometer Clip / Range Diagnostic Viewer

Streams per-axis raw int16 LSB min/max + clip-count over a 1-second window
from the firmware troubleshoot module. Used to confirm whether the configured
accel full-scale range (currently AFS_2G = 2 g rail at INT16_MAX = 32767) is
being saturated under vibration / maneuvers.

Verdict guide (when motors are running):
  - clip_count > 0 on any axis  => SENSOR IS CLIPPING. Increase FS range.
  - |min| or |max| > 30000      => Within ~10% of rail. Risky. Increase FS.
  - |min| or |max| < 25000      => Comfortable margin. AFS_2G is OK.

At AFS_2G: 1 g = 16384 LSB. So 32440 LSB ~= 1.98 g.

Usage:
  python3 troubleshoot_accel_clip_view.py
"""

SERIAL_PORT = None
BAUD_RATE = 19200
SEND_LOG_ID = 0x00

LOG_CLASS_NONE                = 0x00
LOG_CLASS_HEART_BEAT          = 0x09
LOG_CLASS_TROUBLESHOOT_ACCEL  = 0x1D
DB_CMD_LOG_CLASS              = 0x03
DB_CMD_RESET                  = 0x07

PAYLOAD_SIZE = 20  # 3*2 (min) + 3*2 (max) + 3*2 (clip) + 2 (samples)
RAIL = 32767       # INT16 rail
LSB_PER_G_AT_2G = 16384.0
WARN_LSB = 30000   # ~1.83 g at AFS_2G

# Colors
BG_COLOR    = '#1e1e1e'
PANEL_COLOR = '#252526'
TEXT_COLOR  = '#cccccc'
DIM_TEXT    = '#888888'
GRID_COLOR  = '#3c3c3c'
BTN_GREEN     = '#2d5a2d'
BTN_GREEN_HOV = '#3d7a3d'
BTN_RED       = '#5a2d2d'
BTN_RED_HOV   = '#7a3d3d'
COLOR_X     = '#ff5555'
COLOR_Y     = '#55dd55'
COLOR_Z     = '#5599ff'
COLOR_RAIL  = '#ff3333'
COLOR_WARN  = '#ffaa44'

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

data_queue = queue.Queue()
g_serial = None
g_logging_active = False


def _build_frame(cmd_id, payload):
    msg_class = 0x00
    length = len(payload)
    header = struct.pack('<2sBBH', b'db', cmd_id, msg_class, length)
    cs = cmd_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
    for b in payload:
        cs += b
    cs &= 0xFFFF
    return header + payload + struct.pack('<H', cs)


def send_log_class_command(ser, log_class):
    ser.write(_build_frame(DB_CMD_LOG_CLASS, bytes([log_class])))
    ser.flush()
    names = {0x00: 'NONE', 0x09: 'HEART_BEAT', 0x1D: 'TROUBLESHOOT_ACCEL'}
    print(f"  \u2192 Log class: {names.get(log_class, f'0x{log_class:02X}')}")


def send_reset_command(ser):
    ser.write(_build_frame(DB_CMD_RESET, bytes([0x00])))
    ser.flush()
    print("  \u2192 Reset command sent")


def serial_reader():
    global g_serial
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
            _ = ser.read(1)
            len_bytes = ser.read(2)
            if len(len_bytes) < 2:
                continue
            length = int.from_bytes(len_bytes, 'little')
            if length > 1024:
                continue
            payload = ser.read(length)
            if len(payload) != length:
                continue
            _ = ser.read(2)

            if msg_id == SEND_LOG_ID and length == PAYLOAD_SIZE:
                vals = struct.unpack('<hhh hhh HHH H', payload)
                # min[3], max[3], clip[3], samples
                data_queue.put(vals)
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


def main():
    global g_logging_active

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

    fig = plt.figure(figsize=(13, 8))
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle('Accel Clip Diagnostic \u2014 raw INT16 LSB (AFS_2G \u2192 \u00b132767 = \u00b12 g)',
                 fontsize=12, color=TEXT_COLOR, fontweight='bold', y=0.97)

    gs = fig.add_gridspec(2, 1, height_ratios=[1.4, 1.0], hspace=0.35,
                          left=0.08, right=0.97, top=0.91, bottom=0.13)

    ax_range = fig.add_subplot(gs[0])
    ax_clip  = fig.add_subplot(gs[1])

    axes_labels = ['X (Right)', 'Y (Forward)', 'Z (Down)']
    axes_colors = [COLOR_X, COLOR_Y, COLOR_Z]
    x_pos = [0, 1, 2]

    # Range plot: vertical bars from min to max per axis, with rail markers
    ax_range.set_facecolor(BG_COLOR)
    ax_range.grid(True, axis='y', alpha=0.25)
    ax_range.axhline( RAIL, color=COLOR_RAIL, lw=1.2, ls='--', alpha=0.9, label=f'\u00b1Rail ({RAIL})')
    ax_range.axhline(-RAIL, color=COLOR_RAIL, lw=1.2, ls='--', alpha=0.9)
    ax_range.axhline( WARN_LSB, color=COLOR_WARN, lw=1.0, ls=':', alpha=0.7, label=f'\u00b1Warn ({WARN_LSB})')
    ax_range.axhline(-WARN_LSB, color=COLOR_WARN, lw=1.0, ls=':', alpha=0.7)
    ax_range.axhline(0, color=DIM_TEXT, lw=0.5, alpha=0.5)

    range_bars = []
    range_labels = []
    for i, (lbl, c) in enumerate(zip(axes_labels, axes_colors)):
        bar = ax_range.bar(i, 0, bottom=0, width=0.55, color=c, alpha=0.75,
                           edgecolor=c, linewidth=1.5)
        range_bars.append(bar[0])
        # Numeric annotation
        txt = ax_range.text(i, 0, '', ha='center', va='center',
                            fontsize=9, fontfamily='monospace', color=TEXT_COLOR,
                            bbox=dict(facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                                      boxstyle='round,pad=0.3', alpha=0.85))
        range_labels.append(txt)
    ax_range.set_xticks(x_pos)
    ax_range.set_xticklabels(axes_labels, color=TEXT_COLOR, fontsize=10)
    ax_range.set_ylabel('Raw LSB (1 g \u2248 16384)', color=TEXT_COLOR)
    ax_range.set_ylim(-RAIL * 1.15, RAIL * 1.15)
    ax_range.legend(loc='lower right', fontsize=8, facecolor=PANEL_COLOR, edgecolor=GRID_COLOR)

    # Clip count plot
    ax_clip.set_facecolor(BG_COLOR)
    ax_clip.grid(True, axis='y', alpha=0.25)
    clip_bars = ax_clip.bar(x_pos, [0, 0, 0], width=0.55,
                            color=axes_colors, alpha=0.75,
                            edgecolor=axes_colors, linewidth=1.5)
    clip_labels = []
    for i in range(3):
        txt = ax_clip.text(i, 0, '0', ha='center', va='bottom',
                           fontsize=10, fontfamily='monospace', color=TEXT_COLOR,
                           fontweight='bold')
        clip_labels.append(txt)
    ax_clip.set_xticks(x_pos)
    ax_clip.set_xticklabels(axes_labels, color=TEXT_COLOR, fontsize=10)
    ax_clip.set_ylabel('Clipped samples in last 1 s window', color=TEXT_COLOR)
    ax_clip.set_ylim(0, 10)

    # Status / verdict text
    verdict_text = fig.text(0.5, 0.06,
                            'Press Start Log to begin streaming...',
                            ha='center', fontsize=10, color=DIM_TEXT,
                            fontfamily='monospace',
                            bbox=dict(facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                                      boxstyle='round,pad=0.4', alpha=0.85))

    # Buttons
    ax_toggle = fig.add_axes([0.66, 0.015, 0.10, 0.04])
    btn_toggle = Button(ax_toggle, 'Start Log',
                        color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_toggle.label.set_color(TEXT_COLOR)
    btn_toggle.label.set_fontsize(9)

    ax_reset = fig.add_axes([0.78, 0.015, 0.10, 0.04])
    btn_reset = Button(ax_reset, 'Reset FC',
                       color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_reset.label.set_color(TEXT_COLOR)
    btn_reset.label.set_fontsize(9)

    def on_toggle(event):
        global g_logging_active
        if not (g_serial and g_serial.is_open):
            print("  \u2717 No serial connection")
            return
        if g_logging_active:
            send_log_class_command(g_serial, LOG_CLASS_HEART_BEAT)
            g_logging_active = False
            btn_toggle.label.set_text('Start Log')
            btn_toggle.color = BTN_GREEN
            btn_toggle.hovercolor = BTN_GREEN_HOV
        else:
            send_log_class_command(g_serial, LOG_CLASS_TROUBLESHOOT_ACCEL)
            g_logging_active = True
            btn_toggle.label.set_text('Stop Log')
            btn_toggle.color = BTN_RED
            btn_toggle.hovercolor = BTN_RED_HOV
        fig.canvas.draw_idle()

    def on_reset(event):
        if g_serial and g_serial.is_open:
            send_reset_command(g_serial)

    btn_toggle.on_clicked(on_toggle)
    btn_reset.on_clicked(on_reset)

    # State for the last frame
    state = {'mins': [0, 0, 0], 'maxs': [0, 0, 0],
             'clips': [0, 0, 0], 'samples': 0}

    def update(_frame):
        drained = False
        while not data_queue.empty():
            try:
                vals = data_queue.get_nowait()
                state['mins']    = list(vals[0:3])
                state['maxs']    = list(vals[3:6])
                state['clips']   = list(vals[6:9])
                state['samples'] = vals[9]
                drained = True
            except queue.Empty:
                break

        if not drained:
            return []

        artists = []
        # Update range bars (from min to max)
        for i in range(3):
            mn = state['mins'][i]
            mx = state['maxs'][i]
            height = mx - mn
            range_bars[i].set_y(mn)
            range_bars[i].set_height(height)
            # Color border red if clipping or >= warn
            extreme = max(abs(mn), abs(mx))
            if state['clips'][i] > 0:
                range_bars[i].set_edgecolor(COLOR_RAIL)
                range_bars[i].set_linewidth(3.0)
            elif extreme >= WARN_LSB:
                range_bars[i].set_edgecolor(COLOR_WARN)
                range_bars[i].set_linewidth(2.5)
            else:
                range_bars[i].set_edgecolor(axes_colors[i])
                range_bars[i].set_linewidth(1.5)
            # Label in middle of bar
            mid = (mn + mx) / 2
            range_labels[i].set_position((i, mid))
            range_labels[i].set_text(
                f'min: {mn:+6d}\nmax: {mx:+6d}\n({mn/LSB_PER_G_AT_2G:+.2f} \u2192 {mx/LSB_PER_G_AT_2G:+.2f} g)'
            )
            artists.extend([range_bars[i], range_labels[i]])

        # Update clip bars
        max_clip = max(max(state['clips']), 5)
        ax_clip.set_ylim(0, max_clip * 1.25)
        for i in range(3):
            c = state['clips'][i]
            clip_bars[i].set_height(c)
            clip_labels[i].set_position((i, c))
            clip_labels[i].set_text(f'{c}')
            if c > 0:
                clip_bars[i].set_edgecolor(COLOR_RAIL)
                clip_bars[i].set_linewidth(3.0)
                clip_labels[i].set_color(COLOR_RAIL)
            else:
                clip_bars[i].set_edgecolor(axes_colors[i])
                clip_bars[i].set_linewidth(1.5)
                clip_labels[i].set_color(TEXT_COLOR)
            artists.extend([clip_bars[i], clip_labels[i]])

        # Verdict
        any_clip = any(c > 0 for c in state['clips'])
        any_warn = any(max(abs(state['mins'][i]), abs(state['maxs'][i])) >= WARN_LSB
                       for i in range(3))
        smp = state['samples']
        if any_clip:
            verdict = (f'\u2717 CLIPPING DETECTED  X:{state["clips"][0]}  '
                       f'Y:{state["clips"][1]}  Z:{state["clips"][2]}  '
                       f'(of {smp} samples)  \u2192 raise FS range (AFS_8G or AFS_16G)')
            color = COLOR_RAIL
        elif any_warn:
            verdict = (f'\u26a0 NEAR RAIL (>{WARN_LSB} LSB \u2248 1.83 g)  '
                       f'samples={smp}  \u2192 increase FS range recommended')
            color = COLOR_WARN
        else:
            ext = max(max(abs(state['mins'][i]), abs(state['maxs'][i])) for i in range(3))
            verdict = (f'\u2713 OK  peak |LSB|={ext}  ({ext/LSB_PER_G_AT_2G:.2f} g)  '
                       f'samples={smp}  \u2192 AFS_2G has comfortable margin')
            color = '#55dd55'
        verdict_text.set_text(verdict)
        verdict_text.set_color(color)
        artists.append(verdict_text)

        return artists

    anim = FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
