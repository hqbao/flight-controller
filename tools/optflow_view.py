"""
Optflow Viewer
==============

Live view of body-frame horizontal velocity from the downward optical-flow
camera (= flow_raw · range, no derotation) overlaid with the ESKF's current
velocity prediction. When camera and ESKF agree the lines track; when they
diverge the filter is rejecting the camera (or vice-versa).

  green = v_meas  (m/s, body) — what the camera reports this frame
  blue  = v_pred  (m/s, body) — ESKF current estimate

Wire format (8 floats / 32 B, LOG_CLASS_OPTFLOW = 0x22):
    [0..1] flow_raw  (rad/s, body)        — not plotted
    [2..3] v_meas    (m/s,  body)         ← plotted (green)
    [4..5] v_pred    (m/s,  body)         ← plotted (blue)
    [6]    range     (m)                  — status only
    [7]    clarity   (0..~100)            ← plotted
"""

import math
import queue
import struct
import sys
import threading
import time
from collections import deque

import matplotlib
import serial
import serial.tools.list_ports

matplotlib.use("macosx" if sys.platform == "darwin" else "TkAgg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402
from matplotlib.widgets import Button  # noqa: E402


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


# --- Wire protocol ---
BAUD_RATE = 38400
SEND_LOG_ID = 0x00
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET = 0x07
DB_CMD_CHIP_ID = 0x09

LOG_CLASS_NONE = 0x00
LOG_CLASS_HEART_BEAT = 0x09
LOG_CLASS_OPTFLOW = 0x22

FRAME_SIZE = 32           # 8 floats
FRAME_FORMAT = "<8f"

# --- UI palette (shared with other tools/*.py viewers) ---
BG_COLOR = "#1e1e1e"
PANEL_COLOR = "#252526"
TEXT_COLOR = "#cccccc"
DIM_TEXT = "#888888"
GRID_COLOR = "#3c3c3c"
ACCENT_BLUE = "#5599ff"
ACCENT_GREEN = "#55cc55"
ACCENT_RED = "#ff5555"
ACCENT_ORANGE = "#ff9955"
ACCENT_YELLOW = "#ffdd55"
ACCENT_PURPLE = "#aa66ff"
BTN_GREEN = "#2d5a2d"
BTN_GREEN_HOV = "#3d7a3d"
BTN_RED = "#5a2d2d"
BTN_RED_HOV = "#7a3d3d"

WINDOW_S = 15.0
RATE_HZ = 25


# --- Serial autodetect ---
SERIAL_PORT = None
for port, desc, _ in sorted(serial.tools.list_ports.comports()):
    if any(x in port for x in ("usbmodem", "usbserial", "SLAB_USBtoUART",
                               "ttyACM", "ttyUSB", "COM")):
        SERIAL_PORT = port
        print(f"Auto-selected: {port} ({desc})")
        break
if not SERIAL_PORT:
    print("No compatible serial port found.")


# --- Shared state ---
data_queue = queue.Queue(maxsize=1)
g_serial = None
g_logging_active = False
g_chip_id = None
g_rx_frame_count = 0


def push_latest(vals):
    try:
        data_queue.get_nowait()
    except queue.Empty:
        pass
    try:
        data_queue.put_nowait(vals)
    except queue.Full:
        pass


def _send_db(ser, msg_id, payload_byte):
    msg_class = 0x00
    length = 1
    header = struct.pack("<2sBBH", b"db", msg_id, msg_class, length)
    cs = (msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
          + payload_byte) & 0xFFFF
    ser.write(header + bytes([payload_byte]) + struct.pack("<H", cs))
    ser.flush()


def send_log_class(ser, log_class):
    _send_db(ser, DB_CMD_LOG_CLASS, log_class)


def send_reset(ser):
    _send_db(ser, DB_CMD_RESET, 0x00)


def send_chip_id(ser):
    _send_db(ser, DB_CMD_CHIP_ID, 0x00)


def serial_reader():
    """Same non-blocking parser pattern as the other tools/*.py viewers."""
    global g_serial, g_chip_id, g_rx_frame_count
    if not SERIAL_PORT:
        return
    HEADER_SIZE = 6
    CHKSUM_SIZE = 2
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(b"\x00" * 32)
        ser.flush()
        time.sleep(0.05)
        g_serial = ser
        send_chip_id(ser)
        send_log_class(ser, LOG_CLASS_HEART_BEAT)

        buf = bytearray()
        while True:
            n = ser.in_waiting
            if n:
                buf.extend(ser.read(n))
            else:
                time.sleep(0.001)

            while True:
                idx = buf.find(b"db")
                if idx < 0:
                    if len(buf) > 1:
                        del buf[:-1]
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < HEADER_SIZE:
                    break
                length = buf[4] | (buf[5] << 8)
                if length > 1024:
                    del buf[:2]
                    continue
                frame_size = HEADER_SIZE + length + CHKSUM_SIZE
                if len(buf) < frame_size:
                    break
                msg_id = buf[2]
                payload = bytes(buf[HEADER_SIZE:HEADER_SIZE + length])
                del buf[:frame_size]

                if msg_id == SEND_LOG_ID and length == 8 and g_chip_id is None:
                    g_chip_id = payload[:8].hex().upper()
                    print(f"Chip ID: {g_chip_id}")
                elif msg_id == SEND_LOG_ID and length == FRAME_SIZE:
                    vals = struct.unpack(FRAME_FORMAT, payload)
                    # Flow can legitimately be NaN before the first frame.
                    # Only require clarity to be finite.
                    if math.isfinite(vals[7]):
                        g_rx_frame_count += 1
                        push_latest(vals)
    except Exception as e:
        print(f"Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


def main():
    threading.Thread(target=serial_reader, daemon=True).start()

    plt.style.use("dark_background")
    plt.rcParams.update({
        "figure.facecolor": BG_COLOR,
        "axes.facecolor": BG_COLOR,
        "axes.edgecolor": GRID_COLOR,
        "axes.labelcolor": TEXT_COLOR,
        "text.color": TEXT_COLOR,
        "xtick.color": DIM_TEXT,
        "ytick.color": DIM_TEXT,
        "grid.color": GRID_COLOR,
    })

    fig = plt.figure(figsize=screen_fit_figsize(14, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.canvas.manager.set_window_title("Optflow Viewer")
    fig.suptitle("Optflow — v_meas (camera, green) vs v_pred (ESKF, blue), m/s",
                 fontsize=14, color=TEXT_COLOR, fontweight="bold", y=0.99)

    # Four stacked strips: body X (m/s), body Y (m/s), range (m), clarity.
    gs = fig.add_gridspec(4, 1, left=0.07, right=0.97, top=0.93, bottom=0.18,
                          hspace=0.28, height_ratios=[3, 3, 1, 1])
    ax_x = fig.add_subplot(gs[0, 0])
    ax_y = fig.add_subplot(gs[1, 0], sharex=ax_x)
    ax_r = fig.add_subplot(gs[2, 0], sharex=ax_x)
    ax_q = fig.add_subplot(gs[3, 0], sharex=ax_x)

    for axp in (ax_x, ax_y, ax_r, ax_q):
        axp.set_facecolor(BG_COLOR)
        axp.grid(True, color=GRID_COLOR, alpha=0.4, linewidth=0.5)
        for sp in axp.spines.values():
            sp.set_color(GRID_COLOR)
        axp.set_autoscaley_on(True)

    for axp in (ax_x, ax_y):
        axp.axhline(0.0, color=GRID_COLOR, lw=0.6)

    ax_x.set_ylabel("body X (m/s)", color=TEXT_COLOR, fontsize=9)
    ax_y.set_ylabel("body Y (m/s)", color=TEXT_COLOR, fontsize=9)
    ax_r.set_ylabel("range (m)",    color=TEXT_COLOR, fontsize=9)
    ax_q.set_ylabel("clarity",      color=TEXT_COLOR, fontsize=9)
    ax_q.set_xlabel("time (s)",     color=TEXT_COLOR, fontsize=9)
    plt.setp(ax_x.get_xticklabels(), visible=False)
    plt.setp(ax_y.get_xticklabels(), visible=False)
    plt.setp(ax_r.get_xticklabels(), visible=False)

    # Buffers
    maxlen = int(WINDOW_S * RATE_HZ) + 8
    t_buf   = deque(maxlen=maxlen)
    vmeas_x = deque(maxlen=maxlen); vmeas_y = deque(maxlen=maxlen)
    vpred_x = deque(maxlen=maxlen); vpred_y = deque(maxlen=maxlen)
    range_q = deque(maxlen=maxlen)
    clarity = deque(maxlen=maxlen)

    l_vmeas_x, = ax_x.plot([], [], color=ACCENT_GREEN, lw=1.4,
                           label="v_meas (camera)")
    l_vpred_x, = ax_x.plot([], [], color=ACCENT_BLUE,  lw=1.6, alpha=0.9,
                           label="v_pred (ESKF)")
    l_vmeas_y, = ax_y.plot([], [], color=ACCENT_GREEN, lw=1.4,
                           label="v_meas (camera)")
    l_vpred_y, = ax_y.plot([], [], color=ACCENT_BLUE,  lw=1.6, alpha=0.9,
                           label="v_pred (ESKF)")
    l_range,   = ax_r.plot([], [], color=ACCENT_ORANGE, lw=1.2,
                           label="range")
    l_clarity, = ax_q.plot([], [], color=ACCENT_PURPLE, lw=1.2,
                           label="clarity")

    for axp in (ax_x, ax_y):
        axp.legend(loc="upper right", fontsize=8, framealpha=0.3,
                   facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                   labelcolor=TEXT_COLOR, ncol=2)

    # --- Bottom info row -----------------------------------------------
    chip_text   = fig.text(0.02, 0.06, "", fontsize=8, ha="left",
                           color=DIM_TEXT)
    fps_text    = fig.text(0.50, 0.06, "", fontsize=8, ha="center",
                           color=DIM_TEXT)
    status_text = fig.text(0.50, 0.10, "", fontsize=9, ha="center",
                           color=TEXT_COLOR, family="monospace")

    t0 = [None]
    draw_count = [0]
    last_rx_count = [0]
    last_fps_t = [time.time()]

    # --- Buttons ---
    ax_toggle = fig.add_axes([0.76, 0.02, 0.10, 0.04])
    btn_toggle = Button(ax_toggle, "Start Log",
                        color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_toggle.label.set_color(TEXT_COLOR)
    btn_toggle.label.set_fontsize(8)

    def on_toggle(_event):
        global g_logging_active
        if not (g_serial and g_serial.is_open):
            return
        if g_logging_active:
            send_log_class(g_serial, LOG_CLASS_NONE)
            g_logging_active = False
            btn_toggle.label.set_text("Start Log")
            btn_toggle.color = BTN_GREEN
            btn_toggle.hovercolor = BTN_GREEN_HOV
            ax_toggle.set_facecolor(BTN_GREEN)
        else:
            send_log_class(g_serial, LOG_CLASS_OPTFLOW)
            g_logging_active = True
            btn_toggle.label.set_text("Stop Log")
            btn_toggle.color = BTN_RED
            btn_toggle.hovercolor = BTN_RED_HOV
            ax_toggle.set_facecolor(BTN_RED)

    btn_toggle.on_clicked(on_toggle)

    ax_reset = fig.add_axes([0.87, 0.02, 0.10, 0.04])
    btn_reset = Button(ax_reset, "Reset FC",
                       color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_reset.label.set_color(TEXT_COLOR)
    btn_reset.label.set_fontsize(8)

    def on_reset(_event):
        global g_logging_active
        if not (g_serial and g_serial.is_open):
            return
        send_reset(g_serial)
        g_logging_active = False
        btn_toggle.label.set_text("Start Log")
        btn_toggle.color = BTN_GREEN
        btn_toggle.hovercolor = BTN_GREEN_HOV
        ax_toggle.set_facecolor(BTN_GREEN)
        for d in (t_buf, vmeas_x, vmeas_y, vpred_x, vpred_y, range_q, clarity):
            d.clear()
        t0[0] = None

        def _after():
            time.sleep(2.0)
            if g_serial and g_serial.is_open:
                g_serial.reset_input_buffer()
                send_log_class(g_serial, LOG_CLASS_HEART_BEAT)
        threading.Thread(target=_after, daemon=True).start()

    btn_reset.on_clicked(on_reset)

    ax_clear = fig.add_axes([0.65, 0.02, 0.10, 0.04])
    btn_clear = Button(ax_clear, "Clear",
                       color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_clear.label.set_color(TEXT_COLOR)
    btn_clear.label.set_fontsize(8)

    def on_clear(_event):
        for d in (t_buf, vmeas_x, vmeas_y, vpred_x, vpred_y, range_q, clarity):
            d.clear()
        t0[0] = None
        for axp in (ax_x, ax_y, ax_r, ax_q):
            axp.relim()
            axp.set_autoscaley_on(True)
            axp.autoscale_view(scalex=False, scaley=True)

    btn_clear.on_clicked(on_clear)

    def update(_):
        latest = None
        while True:
            try:
                latest = data_queue.get_nowait()
            except queue.Empty:
                break

        now = time.time()
        if latest is not None:
            draw_count[0] += 1
        if now - last_fps_t[0] >= 1.0:
            elapsed = now - last_fps_t[0]
            rx_count = g_rx_frame_count
            rx_hz = (rx_count - last_rx_count[0]) / elapsed
            draw_hz = draw_count[0] / elapsed
            fps_text.set_text(f"RX {rx_hz:.0f} Hz | Draw {draw_hz:.0f} Hz")
            last_rx_count[0] = rx_count
            draw_count[0] = 0
            last_fps_t[0] = now

        if latest is None:
            return ()

        if t0[0] is None:
            t0[0] = now
        t = now - t0[0]

        v_meas_x, v_meas_y = latest[2], latest[3]
        v_pred_x, v_pred_y = latest[4], latest[5]
        rng                = latest[6]
        clr                = latest[7]

        t_buf.append(t)
        vmeas_x.append(v_meas_x); vmeas_y.append(v_meas_y)
        vpred_x.append(v_pred_x); vpred_y.append(v_pred_y)
        range_q.append(rng)
        clarity.append(clr)

        while t_buf and (t - t_buf[0]) > WINDOW_S:
            t_buf.popleft()
            for d in (vmeas_x, vmeas_y, vpred_x, vpred_y, range_q, clarity):
                d.popleft()

        ts = list(t_buf)
        l_vmeas_x.set_data(ts, list(vmeas_x))
        l_vpred_x.set_data(ts, list(vpred_x))
        l_vmeas_y.set_data(ts, list(vmeas_y))
        l_vpred_y.set_data(ts, list(vpred_y))
        l_range.set_data(ts, list(range_q))
        l_clarity.set_data(ts, list(clarity))

        if ts:
            x_lo = max(0.0, ts[-1] - WINDOW_S)
            x_hi = max(WINDOW_S, ts[-1])
            ax_x.set_xlim(x_lo, x_hi)
            for axp in (ax_x, ax_y, ax_r, ax_q):
                axp.relim()
                axp.set_autoscaley_on(True)
                axp.autoscale_view(scalex=False, scaley=True)

        # NaN-safe formatting for the status row.
        def _fmt(v):
            return f"{v:+6.3f}" if math.isfinite(v) else "  nan "

        status_text.set_text(
            f"X v_meas={_fmt(v_meas_x)}  v_pred={_fmt(v_pred_x)}  "
            f"| Y v_meas={_fmt(v_meas_y)}  v_pred={_fmt(v_pred_y)}  m/s"
            f"  | range={rng:5.2f} m  clarity={clr:5.1f}"
        )
        if g_chip_id is not None:
            chip_text.set_text(f"Chip {g_chip_id}")
        return ()

    _anim = FuncAnimation(fig, update, interval=20, blit=False,
                          cache_frame_data=False)

    def on_close(_):
        if g_serial and g_serial.is_open:
            try:
                send_log_class(g_serial, LOG_CLASS_NONE)
            except Exception:
                pass

    fig.canvas.mpl_connect("close_event", on_close)
    plt.show()


if __name__ == "__main__":
    main()
