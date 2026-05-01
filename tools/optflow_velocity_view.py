"""
Optical-Flow Velocity Viewer
=============================

Shows the body-frame horizontal velocity (vx, vy) measured by the DOWN-pointing
optical-flow camera overlaid on the ESKF-predicted body velocity. Useful when
bringing up the optflow → fusion6 path: confirms sign convention, range gating,
and that the estimator tracks measured velocity during translation.

The UP camera is not shown here — it has no range finder so it is not fused
(see state_estimation.c on_optflow()).

Wire format (6 floats / 24 B, LOG_CLASS_VEL_FUSION = 0x20):
    [0..1]  v_meas DOWN  (vx, vy, body, m/s)
    [2..3]  v_pred       (vx, vy from current ESKF, body, m/s)
    [4]     clarity DOWN
    [5]     range DOWN (m)
"""

import math
import queue
import struct
import sys
import threading
import time
from collections import deque

import matplotlib
import numpy as np
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
LOG_CLASS_VEL_FUSION = 0x20

VEL_FRAME_SIZE = 24            # 6 floats
VEL_FRAME_FORMAT = "<6f"

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
BTN_COLOR = "#333333"
BTN_HOVER = "#444444"

WINDOW_S = 10.0      # rolling time window
RATE_HZ = 25         # log stream rate (informational)

BOX_HALF_M = 2.0     # ±2 m position view; recenter when current pos escapes
TRAIL_LEN  = 800     # ~32 s at 25 Hz
ARROW_SCALE = 0.5    # 1 m/s drawn as 0.5 m on the 2D chart

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
    global g_serial, g_chip_id, g_rx_frame_count
    if not SERIAL_PORT:
        return
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.write(b"\x00" * 32)
        ser.flush()
        time.sleep(0.05)
        g_serial = ser
        send_chip_id(ser)
        send_log_class(ser, LOG_CLASS_HEART_BEAT)

        while True:
            b1 = ser.read(1)
            if not b1 or b1[0] not in (0x62, 0x64):
                continue
            b2 = ser.read(1)
            if not b2:
                continue
            if not ((b1[0] == 0x64 and b2[0] == 0x62)
                    or (b1[0] == 0x62 and b2[0] == 0x64)):
                continue
            id_byte = ser.read(1)
            class_byte = ser.read(1)
            len_bytes = ser.read(2)
            if not (id_byte and class_byte and len(len_bytes) == 2):
                continue
            msg_id = id_byte[0]
            length = int.from_bytes(len_bytes, "little")
            if length > 1024:
                continue
            payload = ser.read(length)
            if len(payload) != length:
                continue
            _ = ser.read(2)  # checksum (unverified)
            if msg_id == SEND_LOG_ID and length == 8 and g_chip_id is None:
                g_chip_id = payload[:8].hex().upper()
                print(f"Chip ID: {g_chip_id}")
            elif msg_id == SEND_LOG_ID and length == VEL_FRAME_SIZE:
                vals = struct.unpack(VEL_FRAME_FORMAT, payload)
                if all(math.isfinite(v) for v in vals):
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

    fig = plt.figure(figsize=screen_fit_figsize(16, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.canvas.manager.set_window_title("Optflow Velocity Viewer")
    fig.suptitle("State Estimation — Optical-Flow Body Velocity",
                 fontsize=14, color=TEXT_COLOR, fontweight="bold", y=0.99)

    # Left column: 4 stacked time-series strips (vx, vy, clarity, range).
    # Right column: 2D top-down body-frame position with velocity arrows.
    gs = fig.add_gridspec(4, 2, left=0.05, right=0.97, top=0.94, bottom=0.14,
                          hspace=0.30, wspace=0.18,
                          height_ratios=[3, 3, 1.4, 1.4],
                          width_ratios=[1.4, 1.0])
    ax_vx = fig.add_subplot(gs[0, 0])
    ax_vy = fig.add_subplot(gs[1, 0], sharex=ax_vx)
    ax_q  = fig.add_subplot(gs[2, 0], sharex=ax_vx)
    ax_r  = fig.add_subplot(gs[3, 0], sharex=ax_vx)
    ax_xy = fig.add_subplot(gs[:, 1])

    for axp in (ax_vx, ax_vy, ax_q, ax_r):
        axp.set_facecolor(BG_COLOR)
        axp.grid(True, color=GRID_COLOR, alpha=0.4, linewidth=0.5)
        for sp in axp.spines.values():
            sp.set_color(GRID_COLOR)

    ax_vx.set_ylabel("vx body (m/s)", color=TEXT_COLOR, fontsize=9)
    ax_vy.set_ylabel("vy body (m/s)", color=TEXT_COLOR, fontsize=9)
    ax_q.set_ylabel("clarity",        color=TEXT_COLOR, fontsize=9)
    ax_r.set_ylabel("range (m)",      color=TEXT_COLOR, fontsize=9)
    ax_r.set_xlabel("time (s)",       color=TEXT_COLOR, fontsize=9)
    plt.setp(ax_vx.get_xticklabels(), visible=False)
    plt.setp(ax_vy.get_xticklabels(), visible=False)
    plt.setp(ax_q.get_xticklabels(),  visible=False)

    # Y axes start in autoscale mode; matplotlib will compute limits from the
    # first batch of data. Note: calling set_ylim() at any point turns off
    # the autoscale flag, so we must re-enable it explicitly in the update
    # loop with set_autoscaley_on(True) before relim()/autoscale_view().
    for axp in (ax_vx, ax_vy, ax_q, ax_r):
        axp.set_autoscaley_on(True)

    # Rolling buffers (one slot per expected RX frame; over-allocating just
    # multiplies the per-redraw cost without showing more data).
    t_buf      = deque(maxlen=int(WINDOW_S * RATE_HZ) + 8)
    vx_down    = deque(maxlen=t_buf.maxlen)
    vy_down    = deque(maxlen=t_buf.maxlen)
    vx_pred    = deque(maxlen=t_buf.maxlen)
    vy_pred    = deque(maxlen=t_buf.maxlen)
    q_down     = deque(maxlen=t_buf.maxlen)
    r_down     = deque(maxlen=t_buf.maxlen)

    # Lines
    l_vx_down, = ax_vx.plot([], [], color=ACCENT_GREEN,  lw=1.4, label="DOWN meas")
    l_vx_pred, = ax_vx.plot([], [], color=ACCENT_BLUE,   lw=1.8, alpha=0.85,
                            label="ESKF pred")
    l_vy_down, = ax_vy.plot([], [], color=ACCENT_GREEN,  lw=1.4, label="DOWN meas")
    l_vy_pred, = ax_vy.plot([], [], color=ACCENT_BLUE,   lw=1.8, alpha=0.85,
                            label="ESKF pred")
    l_q_down,  = ax_q.plot([],  [], color=ACCENT_GREEN,  lw=1.2, label="DOWN")
    l_r_down,  = ax_r.plot([],  [], color=ACCENT_GREEN,  lw=1.2, label="DOWN")

    for axp in (ax_vx, ax_vy):
        axp.legend(loc="upper right", fontsize=7, framealpha=0.3,
                   facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                   labelcolor=TEXT_COLOR)
    for axp in (ax_q, ax_r):
        axp.legend(loc="upper right", fontsize=6, framealpha=0.3,
                   facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                   labelcolor=TEXT_COLOR)

    # --- 2D body-frame position view ---
    # Convention: body X (forward) is the VERTICAL axis (up = forward),
    # body Y (right) is the HORIZONTAL axis. Matches a top-down chase view.
    ax_xy.set_facecolor(BG_COLOR)
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.set_xlim(-BOX_HALF_M, BOX_HALF_M)
    ax_xy.set_ylim(-BOX_HALF_M, BOX_HALF_M)
    ax_xy.grid(True, color=GRID_COLOR, alpha=0.4, linewidth=0.5)
    ax_xy.axhline(0, color=GRID_COLOR, lw=0.6)
    ax_xy.axvline(0, color=GRID_COLOR, lw=0.6)
    ax_xy.set_xlabel("body y — right (m)",   color=TEXT_COLOR, fontsize=9)
    ax_xy.set_ylabel("body x — forward (m)", color=TEXT_COLOR, fontsize=9)
    for sp in ax_xy.spines.values():
        sp.set_color(GRID_COLOR)
    # Box outline at ±BOX_HALF_M (visualises recenter trigger).
    ax_xy.plot([-BOX_HALF_M, BOX_HALF_M, BOX_HALF_M, -BOX_HALF_M, -BOX_HALF_M],
               [-BOX_HALF_M, -BOX_HALF_M, BOX_HALF_M, BOX_HALF_M, -BOX_HALF_M],
               color=GRID_COLOR, lw=0.8, linestyle="--")
    trail_x = deque(maxlen=TRAIL_LEN)
    trail_y = deque(maxlen=TRAIL_LEN)
    trail_line, = ax_xy.plot([], [], color=ACCENT_GREEN, lw=1.2, alpha=0.8,
                             label="path (∫v_meas dt)")
    cur_marker, = ax_xy.plot([], [], "o", color=ACCENT_GREEN, markersize=8,
                             markeredgecolor="white", markeredgewidth=0.8)
    arrow_meas, = ax_xy.plot([], [], color=ACCENT_GREEN, lw=2.2,
                             label="v_meas (m/s)")
    arrow_pred, = ax_xy.plot([], [], color=ACCENT_BLUE, lw=2.2, alpha=0.85,
                             label="v_pred ESKF")
    ax_xy.legend(loc="upper right", fontsize=7, framealpha=0.3,
                 facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                 labelcolor=TEXT_COLOR)
    pos_state = {"px": 0.0, "py": 0.0, "last_t": None, "recenter": 0}

    # --- Bottom info row -----------------------------------------------
    # Layout (y in figure coords):
    #   y=0.085  status line (truth: pos / v_meas / v_pred / clarity / range)
    #   y=0.045  chip id  (left)         |   fps (right of status)
    #   y=0.018  buttons:  [Start Log] [Reset FC]
    chip_text   = fig.text(0.02, 0.045, "", fontsize=8, ha="left",  color=DIM_TEXT)
    fps_text    = fig.text(0.50, 0.045, "", fontsize=8, ha="center", color=DIM_TEXT)
    status_text = fig.text(0.50, 0.085, "", fontsize=8, ha="center",
                           color=TEXT_COLOR, family="monospace")

    t0 = [None]   # set on first sample so the X axis starts at 0
    draw_count = [0]
    last_rx_count = [0]
    last_fps_t = [time.time()]

    # --- Buttons ---
    ax_toggle = fig.add_axes([0.76, 0.018, 0.10, 0.035])
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
            send_log_class(g_serial, LOG_CLASS_VEL_FUSION)
            g_logging_active = True
            btn_toggle.label.set_text("Stop Log")
            btn_toggle.color = BTN_RED
            btn_toggle.hovercolor = BTN_RED_HOV
            ax_toggle.set_facecolor(BTN_RED)

    btn_toggle.on_clicked(on_toggle)

    ax_reset = fig.add_axes([0.87, 0.018, 0.10, 0.035])
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
        # Clear plot history.
        for d in (t_buf, vx_down, vy_down,
                  vx_pred, vy_pred, q_down, r_down):
            d.clear()
        trail_x.clear(); trail_y.clear()
        pos_state["px"] = 0.0; pos_state["py"] = 0.0
        pos_state["last_t"] = None
        pos_state["recenter"] = 0
        t0[0] = None

        def _after():
            time.sleep(2.0)
            if g_serial and g_serial.is_open:
                g_serial.reset_input_buffer()
                send_log_class(g_serial, LOG_CLASS_HEART_BEAT)
        threading.Thread(target=_after, daemon=True).start()

    btn_reset.on_clicked(on_reset)

    # Clear button: wipes all chart history without touching the FC.
    ax_clear = fig.add_axes([0.65, 0.018, 0.10, 0.035])
    btn_clear = Button(ax_clear, "Clear",
                       color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_clear.label.set_color(TEXT_COLOR)
    btn_clear.label.set_fontsize(8)

    def on_clear(_event):
        for d in (t_buf, vx_down, vy_down,
                  vx_pred, vy_pred, q_down, r_down):
            d.clear()
        trail_x.clear(); trail_y.clear()
        pos_state["px"] = 0.0; pos_state["py"] = 0.0
        pos_state["last_t"] = None
        pos_state["recenter"] = 0
        t0[0] = None
        # Reset y-limits so autoscale starts fresh.
        for axp in (ax_vx, ax_vy, ax_q, ax_r):
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

        # --- 2D position integration (body-frame, no yaw rotation) ---
        if pos_state["last_t"] is None:
            dt_pos = 0.0
        else:
            dt_pos = max(0.0, min(0.2, now - pos_state["last_t"]))
        pos_state["last_t"] = now
        pos_state["px"] += latest[0] * dt_pos
        pos_state["py"] += latest[1] * dt_pos
        if (abs(pos_state["px"]) > BOX_HALF_M
                or abs(pos_state["py"]) > BOX_HALF_M):
            pos_state["px"] = 0.0
            pos_state["py"] = 0.0
            trail_x.clear(); trail_y.clear()
            pos_state["recenter"] += 1
        # Plot horizontal=py (right), vertical=px (forward) so body X is up.
        trail_x.append(pos_state["py"])
        trail_y.append(pos_state["px"])
        trail_line.set_data(list(trail_x), list(trail_y))
        cur_marker.set_data([pos_state["py"]], [pos_state["px"]])
        arrow_meas.set_data(
            [pos_state["py"], pos_state["py"] + latest[1] * ARROW_SCALE],
            [pos_state["px"], pos_state["px"] + latest[0] * ARROW_SCALE],
        )
        arrow_pred.set_data(
            [pos_state["py"], pos_state["py"] + latest[3] * ARROW_SCALE],
            [pos_state["px"], pos_state["px"] + latest[2] * ARROW_SCALE],
        )

        t_buf.append(t)
        vx_down.append(latest[0]); vy_down.append(latest[1])
        vx_pred.append(latest[2]); vy_pred.append(latest[3])
        q_down.append(latest[4])
        r_down.append(latest[5])

        # Trim to rolling window.
        while t_buf and (t - t_buf[0]) > WINDOW_S:
            t_buf.popleft()
            for d in (vx_down, vy_down, vx_pred, vy_pred,
                      q_down, r_down):
                d.popleft()

        ts = list(t_buf)
        l_vx_down.set_data(ts, list(vx_down))
        l_vx_pred.set_data(ts, list(vx_pred))
        l_vy_down.set_data(ts, list(vy_down))
        l_vy_pred.set_data(ts, list(vy_pred))
        l_q_down.set_data(ts,  list(q_down))
        l_r_down.set_data(ts,  list(r_down))

        if ts:
            x_lo = max(0.0, ts[-1] - WINDOW_S)
            x_hi = max(WINDOW_S, ts[-1])
            ax_vx.set_xlim(x_lo, x_hi)

            # True autoscale on Y for each strip. set_autoscaley_on(True) is
            # required because matplotlib silently disables the autoscale
            # flag on any axis whose limits have ever been set explicitly
            # (e.g. by the Clear button or an earlier autoscale_view call
            # — yes, autoscale_view itself sets the flag off after running).
            for axp in (ax_vx, ax_vy, ax_q, ax_r):
                axp.relim()
                axp.set_autoscaley_on(True)
                axp.autoscale_view(scalex=False, scaley=True)

        status_text.set_text(
            f"pos=({pos_state['px']:+.2f}, {pos_state['py']:+.2f}) m  "
            f"recenter#{pos_state['recenter']:<2d} │ "
            f"v_meas=({latest[0]:+.2f}, {latest[1]:+.2f}) m/s │ "
            f"v_pred=({latest[2]:+.2f}, {latest[3]:+.2f}) m/s │ "
            f"clarity={latest[4]:>3.0f}  range={latest[5]:.2f} m"
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
