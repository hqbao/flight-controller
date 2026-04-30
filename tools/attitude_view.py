"""
Attitude Vector Viewer
======================

3D real-time visualization of the state_estimation fusion vectors. Identical
in spirit to the legacy attitude_estimation_view.py, adapted for the new
unified ESKF estimator.

Vectors (all in body NED, m/s², drawn at unit length normalised by g):
  Red    \u2014 Measured Accel       (latest accel sample, includes gravity)
  Blue   \u2014 Attitude (predicted)  (R(q)^T \u00b7 [0,0,-g], from current quaternion)
  Green  \u2014 Linear Accel          (gravity-removed body-frame acceleration)

At rest (level), red and blue should overlap pointing to (0, 0, -1) in body
NED, and green should be near zero. Drift between red and blue is the
fusion residual; if it grows the estimator is diverging.

Wire format: LOG_CLASS_ATTITUDE = 0x03 (state_estimation @ 50 Hz)
  9 \u00d7 float32 = 36 bytes
    [0..2] accel_meas_body
    [3..5] gravity_pred_body  (sign-flipped to match accel convention)
    [6..8] accel_linear_body

Usage:
  cd flight-controller/tools
  python3 attitude_view.py
"""

import math
import queue
import struct
import sys
import threading
import time

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
LOG_CLASS_ATTITUDE = 0x03

ATTITUDE_FRAME_SIZE = 36     # 9 floats
GRAVITY_MSS = 9.80665        # m/s² — used to normalise vectors for display

# --- UI palette ---
BG_COLOR = "#1e1e1e"
PANEL_COLOR = "#252526"
TEXT_COLOR = "#cccccc"
DIM_TEXT = "#888888"
GRID_COLOR = "#3c3c3c"
ACCENT_BLUE = "#5599ff"
ACCENT_GREEN = "#55cc55"
ACCENT_RED = "#ff5555"
ACCENT_ORANGE = "#ff9955"
BTN_GREEN = "#2d5a2d"
BTN_GREEN_HOV = "#3d7a3d"
BTN_RED = "#5a2d2d"
BTN_RED_HOV = "#7a3d3d"
BTN_COLOR = "#333333"
BTN_HOVER = "#444444"

COLOR_MEAS = ACCENT_RED      # measured accel
COLOR_PRED = ACCENT_BLUE     # attitude-predicted gravity
COLOR_LINEAR = ACCENT_GREEN  # linear accel

# --- Serial autodetect ---
SERIAL_PORT = None
print("Scanning for serial ports...")
for port, desc, _ in sorted(serial.tools.list_ports.comports()):
    if any(x in port for x in ("usbmodem", "usbserial", "SLAB_USBtoUART",
                               "ttyACM", "ttyUSB", "COM")):
        SERIAL_PORT = port
        print(f"  \u2713 Auto-selected: {port} ({desc})")
        break
    else:
        print(f"  \u00b7 Skipped: {port} ({desc})")
if not SERIAL_PORT:
    print("  \u2717 No compatible serial port found.")

# --- Globals ---
data_queue = queue.Queue(maxsize=1)
g_serial = None
g_logging_active = False
g_chip_id = None
g_rx_frame_count = 0


def push_latest_sample(vals):
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
    payload = bytes([payload_byte])
    header = struct.pack("<2sBBH", b"db", msg_id, msg_class, length)
    cs = (msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
          + payload_byte) & 0xFFFF
    ser.write(header + payload + struct.pack("<H", cs))
    ser.flush()


def send_log_class_command(ser, log_class):
    _send_db(ser, DB_CMD_LOG_CLASS, log_class)
    names = {0x00: "NONE", 0x03: "ATTITUDE", 0x09: "HEART_BEAT"}
    print(f"  \u2192 Log class: {names.get(log_class, f'0x{log_class:02X}')}")


def send_reset_command(ser):
    _send_db(ser, DB_CMD_RESET, 0x00)
    print("  \u2192 Reset command sent")


def send_chip_id_request(ser):
    _send_db(ser, DB_CMD_CHIP_ID, 0x00)
    print("  \u2192 Chip ID request sent")


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
        print(f"  \u2713 Connected to {SERIAL_PORT}")
        send_chip_id_request(ser)
        send_log_class_command(ser, LOG_CLASS_HEART_BEAT)

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
            _ = ser.read(2)  # checksum (not validated)

            if msg_id == SEND_LOG_ID and length == 8 and g_chip_id is None:
                g_chip_id = payload[:8].hex().upper()
                print(f"  \u2713 Chip ID: {g_chip_id}")
            elif msg_id == SEND_LOG_ID and length == ATTITUDE_FRAME_SIZE:
                vals = struct.unpack("<9f", payload)
                if all(math.isfinite(v) for v in vals):
                    g_rx_frame_count += 1
                    push_latest_sample(vals)
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
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
    fig.canvas.manager.set_window_title("Attitude Vector Viewer")
    fig.suptitle("State Estimation \u2014 Attitude Fusion Vectors",
                 fontsize=14, color=TEXT_COLOR, fontweight="bold", y=0.99)

    # --- 3D viewport (NED body frame) ---
    ax = fig.add_axes([0.02, 0.12, 0.62, 0.82], projection="3d")
    ax.set_facecolor(BG_COLOR)
    ax.set_box_aspect((1, 1, 1))
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor(GRID_COLOR)
    ax.yaxis.pane.set_edgecolor(GRID_COLOR)
    ax.zaxis.pane.set_edgecolor(GRID_COLOR)
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_zlim(-1.5, 1.5)
    ax.invert_yaxis()
    ax.set_xlabel("X — Forward (Pitch)", fontsize=9, labelpad=6)
    ax.set_ylabel("Y — Right (Roll)", fontsize=9, labelpad=6)
    ax.set_zlabel("–Z (Up)", fontsize=9, labelpad=6)
    ax.view_init(elev=0, azim=180)
    ax.tick_params(labelsize=7)

    # Reference unit sphere
    u = np.linspace(0, 2 * np.pi, 24)
    v = np.linspace(0, np.pi, 12)
    xs = np.outer(np.cos(u), np.sin(v))
    ys = np.outer(np.sin(u), np.sin(v))
    zs = np.outer(np.ones_like(u), np.cos(v))
    ax.plot_wireframe(xs, ys, zs, color=GRID_COLOR, alpha=0.08, linewidth=0.3)

    # Origin guide axes
    g = 1.2
    ax.plot([-g, g], [0, 0], [0, 0], color=GRID_COLOR, lw=0.5, alpha=0.3)
    ax.plot([0, 0], [-g, g], [0, 0], color=GRID_COLOR, lw=0.5, alpha=0.3)
    ax.plot([0, 0], [0, 0], [-g, g], color=GRID_COLOR, lw=0.5, alpha=0.3)

    # Vectors
    line_meas, = ax.plot([0, 0], [0, 0], [0, 0],
                         color=COLOR_MEAS, lw=3, label="Measured Accel")
    head_meas, = ax.plot([0], [0], [0], color=COLOR_MEAS, marker="o", markersize=8)

    line_pred, = ax.plot([0, 0], [0, 0], [0, 0],
                         color=COLOR_PRED, lw=3, label="Attitude (pred g)")
    head_pred, = ax.plot([0], [0], [0], color=COLOR_PRED, marker="o", markersize=8)

    line_lin, = ax.plot([0, 0], [0, 0], [0, 0],
                        color=COLOR_LINEAR, lw=2, label="Linear Accel", alpha=0.85)
    head_lin, = ax.plot([0], [0], [0], color=COLOR_LINEAR, marker="o", markersize=6)

    ax.legend(loc="upper left", fontsize=8, framealpha=0.3,
              facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # --- Data panel ---
    ax_data = fig.add_axes([0.66, 0.12, 0.33, 0.82])
    ax_data.set_facecolor(PANEL_COLOR)
    ax_data.patch.set_alpha(0.7)
    ax_data.set_xlim(0, 1)
    ax_data.set_ylim(0, 1)
    ax_data.set_xticks([])
    ax_data.set_yticks([])
    for s in ax_data.spines.values():
        s.set_edgecolor(GRID_COLOR)
        s.set_alpha(0.3)

    _mono = dict(fontfamily="monospace", transform=ax_data.transAxes)
    _hdr = dict(fontsize=9, color=DIM_TEXT, fontweight="bold", ha="left", **_mono)
    _val = dict(fontsize=10, fontweight="bold", ha="left", **_mono)

    ax_data.text(0.06, 0.95, "MEASURED ACCEL  (m/s\u00b2)", **_hdr)
    ax_data.plot([0.04, 0.96], [0.943, 0.943], color=COLOR_MEAS, lw=1.0,
                 alpha=0.5, transform=ax_data.transAxes, clip_on=False)
    t_meas_x = ax_data.text(0.06, 0.91, "X(Fwd): +0.000", color=COLOR_MEAS, **_val)
    t_meas_y = ax_data.text(0.06, 0.87, "Y(Rgt): +0.000", color=COLOR_MEAS, **_val)
    t_meas_z = ax_data.text(0.06, 0.83, "Z(Dwn): +0.000", color=COLOR_MEAS, **_val)
    t_meas_m = ax_data.text(0.06, 0.79, "|v|:     0.000", color=DIM_TEXT, **_val)

    ax_data.text(0.06, 0.72, "ATTITUDE (predicted g, m/s\u00b2)", **_hdr)
    ax_data.plot([0.04, 0.96], [0.713, 0.713], color=COLOR_PRED, lw=1.0,
                 alpha=0.5, transform=ax_data.transAxes, clip_on=False)
    t_pred_x = ax_data.text(0.06, 0.68, "X(Fwd): +0.000", color=COLOR_PRED, **_val)
    t_pred_y = ax_data.text(0.06, 0.64, "Y(Rgt): +0.000", color=COLOR_PRED, **_val)
    t_pred_z = ax_data.text(0.06, 0.60, "Z(Dwn): +0.000", color=COLOR_PRED, **_val)
    t_pred_m = ax_data.text(0.06, 0.56, "|v|:     0.000", color=DIM_TEXT, **_val)

    ax_data.text(0.06, 0.49, "LINEAR ACCEL  (m/s\u00b2)", **_hdr)
    ax_data.plot([0.04, 0.96], [0.483, 0.483], color=COLOR_LINEAR, lw=1.0,
                 alpha=0.5, transform=ax_data.transAxes, clip_on=False)
    t_lin_x = ax_data.text(0.06, 0.45, "X(Fwd): +0.000", color=COLOR_LINEAR, **_val)
    t_lin_y = ax_data.text(0.06, 0.41, "Y(Rgt): +0.000", color=COLOR_LINEAR, **_val)
    t_lin_z = ax_data.text(0.06, 0.37, "Z(Dwn): +0.000", color=COLOR_LINEAR, **_val)
    t_lin_m = ax_data.text(0.06, 0.33, "|v|:     0.000", color=DIM_TEXT, **_val)

    ax_data.text(0.06, 0.26, "FUSION RESIDUAL", **_hdr)
    ax_data.plot([0.04, 0.96], [0.253, 0.253], color=GRID_COLOR, lw=0.5,
                 alpha=0.4, transform=ax_data.transAxes, clip_on=False)
    t_err = ax_data.text(0.06, 0.22, "angle(meas, pred): 0.00\u00b0",
                         color=ACCENT_ORANGE, **_val)
    t_tilt = ax_data.text(0.06, 0.18, "tilt from level:    0.00\u00b0",
                          color=TEXT_COLOR, **_val)

    # --- View buttons ---
    views = {"Top": (90, 180), "Front": (0, 0), "Back": (0, 180),
             "Left": (0, 90), "Right": (0, -90)}
    view_btns = []
    for i, (label, (elev, azim)) in enumerate(views.items()):
        bx = fig.add_axes([0.02 + i * 0.065, 0.05, 0.06, 0.035])
        b = Button(bx, label, color=BTN_COLOR, hovercolor=BTN_HOVER)
        b.label.set_color(TEXT_COLOR)
        b.label.set_fontsize(7)
        b.on_clicked(lambda event, e=elev, a=azim: ax.view_init(elev=e, azim=a))
        view_btns.append(b)

    # --- Status bar ---
    chip_text = fig.text(0.96, 0.035, "Chip ID: ---", fontsize=7,
                         ha="right", color=DIM_TEXT)
    fps_text = fig.text(0.96, 0.015, "", fontsize=8, ha="right", color=DIM_TEXT)

    ax_toggle = fig.add_axes([0.74, 0.005, 0.10, 0.04])
    btn_toggle = Button(ax_toggle, "Start Log",
                        color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_toggle.label.set_color(TEXT_COLOR)
    btn_toggle.label.set_fontsize(8)

    def on_toggle(event):
        global g_logging_active
        if not (g_serial and g_serial.is_open):
            return
        if g_logging_active:
            send_log_class_command(g_serial, LOG_CLASS_NONE)
            g_logging_active = False
            btn_toggle.label.set_text("Start Log")
            btn_toggle.color = BTN_GREEN
            btn_toggle.hovercolor = BTN_GREEN_HOV
            ax_toggle.set_facecolor(BTN_GREEN)
        else:
            send_log_class_command(g_serial, LOG_CLASS_ATTITUDE)
            g_logging_active = True
            btn_toggle.label.set_text("Stop Log")
            btn_toggle.color = BTN_RED
            btn_toggle.hovercolor = BTN_RED_HOV
            ax_toggle.set_facecolor(BTN_RED)

    btn_toggle.on_clicked(on_toggle)

    ax_reset = fig.add_axes([0.85, 0.005, 0.10, 0.04])
    btn_reset = Button(ax_reset, "Reset FC",
                       color=BTN_RED, hovercolor=BTN_RED_HOV)
    btn_reset.label.set_color(TEXT_COLOR)
    btn_reset.label.set_fontsize(8)

    def on_reset(event):
        global g_logging_active
        if not (g_serial and g_serial.is_open):
            return
        send_reset_command(g_serial)
        g_logging_active = False
        btn_toggle.label.set_text("Start Log")
        btn_toggle.color = BTN_GREEN
        btn_toggle.hovercolor = BTN_GREEN_HOV
        ax_toggle.set_facecolor(BTN_GREEN)

        def _after():
            time.sleep(2.0)
            if g_serial and g_serial.is_open:
                g_serial.reset_input_buffer()
                send_log_class_command(g_serial, LOG_CLASS_HEART_BEAT)

        threading.Thread(target=_after, daemon=True).start()

    btn_reset.on_clicked(on_reset)

    # --- Animation ---
    draw_count = [0]
    last_rx_count = [0]
    last_fps = [time.time()]

    def update(_):
        latest = None
        while True:
            try:
                latest = data_queue.get_nowait()
            except queue.Empty:
                break
        if latest is None:
            return ()

        v_meas = np.array(latest[0:3])
        v_pred = np.array(latest[3:6])
        v_lin = np.array(latest[6:9])

        # Display vectors normalised by g so |v|≈1 at rest.
        d_meas = v_meas / GRAVITY_MSS
        d_pred = v_pred / GRAVITY_MSS
        d_lin = v_lin / GRAVITY_MSS

        line_meas.set_data([0, d_meas[0]], [0, d_meas[1]])
        line_meas.set_3d_properties([0, d_meas[2]])
        head_meas.set_data([d_meas[0]], [d_meas[1]])
        head_meas.set_3d_properties([d_meas[2]])

        line_pred.set_data([0, d_pred[0]], [0, d_pred[1]])
        line_pred.set_3d_properties([0, d_pred[2]])
        head_pred.set_data([d_pred[0]], [d_pred[1]])
        head_pred.set_3d_properties([d_pred[2]])

        line_lin.set_data([0, d_lin[0]], [0, d_lin[1]])
        line_lin.set_3d_properties([0, d_lin[2]])
        head_lin.set_data([d_lin[0]], [d_lin[1]])
        head_lin.set_3d_properties([d_lin[2]])

        m_meas = float(np.linalg.norm(v_meas))
        m_pred = float(np.linalg.norm(v_pred))
        m_lin = float(np.linalg.norm(v_lin))

        t_meas_x.set_text(f"X(Fwd):{v_meas[0]:+8.3f}")
        t_meas_y.set_text(f"Y(Rgt):{v_meas[1]:+8.3f}")
        t_meas_z.set_text(f"Z(Dwn):{v_meas[2]:+8.3f}")
        t_meas_m.set_text(f"|v|:    {m_meas:8.3f}")

        t_pred_x.set_text(f"X(Fwd):{v_pred[0]:+8.3f}")
        t_pred_y.set_text(f"Y(Rgt):{v_pred[1]:+8.3f}")
        t_pred_z.set_text(f"Z(Dwn):{v_pred[2]:+8.3f}")
        t_pred_m.set_text(f"|v|:    {m_pred:8.3f}")

        t_lin_x.set_text(f"X(Fwd):{v_lin[0]:+8.3f}")
        t_lin_y.set_text(f"Y(Rgt):{v_lin[1]:+8.3f}")
        t_lin_z.set_text(f"Z(Dwn):{v_lin[2]:+8.3f}")
        t_lin_m.set_text(f"|v|:    {m_lin:8.3f}")

        # Angle between measured and predicted gravity = fusion residual.
        denom = max(m_meas * m_pred, 1e-9)
        cos_ang = float(np.clip(np.dot(v_meas, v_pred) / denom, -1.0, 1.0))
        err_deg = math.degrees(math.acos(cos_ang))
        t_err.set_text(f"angle(meas, pred): {err_deg:6.2f}\u00b0")

        # Tilt from level: angle between predicted gravity and earth +Z = body
        # frame [0,0,-g] (since accel reads -g at rest). So compare to (0,0,-1).
        if m_pred > 1e-3:
            cos_t = float(np.clip(np.dot(v_pred, [0, 0, -1]) / m_pred, -1.0, 1.0))
            tilt = math.degrees(math.acos(cos_t))
        else:
            tilt = 0.0
        t_tilt.set_text(f"tilt from level:   {tilt:6.2f}\u00b0")

        # Chip ID + rates
        out = (line_meas, head_meas, line_pred, head_pred, line_lin, head_lin,
               t_meas_x, t_meas_y, t_meas_z, t_meas_m,
               t_pred_x, t_pred_y, t_pred_z, t_pred_m,
               t_lin_x, t_lin_y, t_lin_z, t_lin_m,
               t_err, t_tilt)

        if g_chip_id is not None:
            chip_text.set_text(f"Chip ID: {g_chip_id}")

        now = time.time()
        draw_count[0] += 1
        if now - last_fps[0] >= 1.0:
            elapsed = now - last_fps[0]
            rx_count = g_rx_frame_count
            rx_hz = (rx_count - last_rx_count[0]) / elapsed
            draw_hz = draw_count[0] / elapsed
            fps_text.set_text(f"RX {rx_hz:.0f} Hz | Draw {draw_hz:.0f} Hz")
            last_rx_count[0] = rx_count
            draw_count[0] = 0
            last_fps[0] = now

        return out

    _anim = FuncAnimation(fig, update, interval=20, blit=False,
                          cache_frame_data=False)

    def on_close(_):
        if g_serial and g_serial.is_open:
            try:
                send_log_class_command(g_serial, LOG_CLASS_NONE)
            except Exception:
                pass

    fig.canvas.mpl_connect("close_event", on_close)
    plt.show()


if __name__ == "__main__":
    main()
