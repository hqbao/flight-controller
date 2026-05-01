"""
Mag Diagnostic Viewer
=====================

Visualises body-frame magnetometer diagnostics from state_estimation. The
current firmware does not apply a magnetometer EKF update; this tool keeps the
mapped compass vector, diagnostic predicted field, and attitude overlay visible
while the next heading/mag method is developed.

Wire format (13 floats = 52 B):
    [0..2]   m_meas  (body NED, unit vector, post hard/soft iron)
    [3..5]   m_pred  = R(q)^T \u00b7 m_ned_unit  (body NED, unit)
    [6]      NIS  (0 while mag update is disabled)
    [7]      r_scale  (0 while mag update is disabled)
    [8..10]  roll, pitch, yaw  (deg)
    [11]     reject reason  (0=applied, 1=nonfinite, 2=dir-gate,
                             3=singular-S, 4=singular-S-infl,
                             5=accel-conflict, 6=disabled)
    [12]     reserved (0 while mag update is disabled)

Panels:
    1. 3D diagnostic scene
             - Blue   : body forward axis in earth frame (R(q) \u00b7 [1,0,0])
             - Purple : attitude/predicted accel vector reconstructed as
                                    R(q)^T \u00b7 [0,0,-1], matching attitude_view.py's
                                    quat_to_accel(q,g)/g convention
             - Red    : measured mag rotated to earth (R(q) \u00b7 m_meas)
             - Orange : measured mag horizontal projection in earth frame
             - Green  : configured reference m_ned_unit
         Healthy = red overlays green. Purple matches attitude_view.py.
     2. yaw_est, mag yaw, and field-vector angular error in degrees. Yaw
         traces are unwrapped for display so ±180° boundary crossings do not
         look like estimator jumps.
    3. Measured magnetic inclination vs configured reference
    4. Quaternion/local-frame 3D view: earth axes expressed in local body frame

Status bar: RX Hz / Draw Hz, mean |y|, mean NIS, R-scale, mag status,
decl/incl.
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
LOG_CLASS_MAG_FUSION = 0x1F

MAG_FRAME_SIZE_OLD = 44       # 11 floats, before reject-reason debug field
MAG_FRAME_SIZE_REJECT = 48    # 12 floats, reject reason included
MAG_FRAME_SIZE = 52           # 13 floats, status + reserved field
MAG_FRAME_FORMATS = {
    MAG_FRAME_SIZE_OLD: "<11f",
    MAG_FRAME_SIZE_REJECT: "<12f",
    MAG_FRAME_SIZE: "<13f",
}

# --- Reference field (must match firmware diagnostic constants) ---
DECL_DEG = -0.6
INCL_DEG = 27.5

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
ACCENT_YELLOW = "#ffdd55"
BTN_GREEN = "#2d5a2d"
BTN_GREEN_HOV = "#3d7a3d"
BTN_RED = "#5a2d2d"
BTN_RED_HOV = "#7a3d3d"
BTN_COLOR = "#333333"
BTN_HOVER = "#444444"

NIS_THRESH_3D = 11.345
REJECT_REASONS = {
    0: "applied",
    1: "nonfinite",
    2: "dir-gate",
    3: "singular-S",
    4: "singular-S-infl",
    5: "accel-conflict",
    6: "disabled",
}

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
    names = {0x00: "NONE", 0x09: "HEART_BEAT", 0x1F: "MAG_DIAG"}
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
            elif msg_id == SEND_LOG_ID and length in MAG_FRAME_FORMATS:
                vals = struct.unpack(MAG_FRAME_FORMATS[length], payload)
                if all(math.isfinite(v) for v in vals):
                    g_rx_frame_count += 1
                    push_latest_sample(vals)
    except Exception as e:
        print(f"  \u2717 Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


# --- Math helpers ---

def rot_zyx(roll, pitch, yaw):
    """Body-to-earth rotation matrix. Inputs in radians, Tait-Bryan ZYX."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr],
    ])


def rot_zyx_no_yaw(roll, pitch):
    return rot_zyx(roll, pitch, 0.0)


def unwrap_deg(values):
    """Return a continuous degree series for display-only yaw plots."""
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return arr
    return np.rad2deg(np.unwrap(np.deg2rad(arr)))


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
    fig.canvas.manager.set_window_title("Mag Diagnostic Viewer")
    fig.suptitle("State Estimation \u2014 Magnetometer Diagnostics",
                 fontsize=14, color=TEXT_COLOR, fontweight="bold", y=0.99)

    # --- Reference NED unit vector from configured decl/incl ---
    decl = math.radians(DECL_DEG)
    incl = math.radians(INCL_DEG)
    m_ned_unit = np.array([math.cos(incl)*math.cos(decl),
                            math.cos(incl)*math.sin(decl),
                            math.sin(incl)])

    # --- Panel 1: 3D mag scene (top-left) ---
    ax3d = fig.add_axes([0.03, 0.53, 0.44, 0.40], projection="3d")
    ax3d.set_facecolor(BG_COLOR)
    ax3d.set_box_aspect((1, 1, 1))
    for pane in (ax3d.xaxis.pane, ax3d.yaxis.pane, ax3d.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor(GRID_COLOR)
    ax3d.set_xlim(-1.2, 1.2)
    ax3d.set_ylim(-1.2, 1.2)
    ax3d.set_zlim(-1.2, 1.2)
    ax3d.invert_yaxis()
    ax3d.set_xlabel("X \u2014 North", fontsize=8, labelpad=4)
    ax3d.set_ylabel("Y \u2014 East", fontsize=8, labelpad=4)
    ax3d.set_zlabel("\u2013Z (Up)", fontsize=8, labelpad=4)
    ax3d.view_init(elev=0, azim=180)
    ax3d.tick_params(labelsize=6)

    # Reference unit sphere
    u = np.linspace(0, 2*np.pi, 24)
    v = np.linspace(0, np.pi, 12)
    xs = np.outer(np.cos(u), np.sin(v))
    ys = np.outer(np.sin(u), np.sin(v))
    zs = np.outer(np.ones_like(u), np.cos(v))
    ax3d.plot_wireframe(xs, ys, zs, color=GRID_COLOR, alpha=0.08, linewidth=0.3)
    g = 1.1
    ax3d.plot([-g, g], [0, 0], [0, 0], color=GRID_COLOR, lw=0.5, alpha=0.3)
    ax3d.plot([0, 0], [-g, g], [0, 0], color=GRID_COLOR, lw=0.5, alpha=0.3)
    ax3d.plot([0, 0], [0, 0], [-g, g], color=GRID_COLOR, lw=0.5, alpha=0.3)

    # Static reference field (green, configured WMM)
    ax3d.plot([0, m_ned_unit[0]], [0, m_ned_unit[1]], [0, m_ned_unit[2]],
              color=ACCENT_GREEN, lw=2.5, label="m_ned (ref)")
    ax3d.scatter([m_ned_unit[0]], [m_ned_unit[1]], [m_ned_unit[2]],
                 color=ACCENT_GREEN, s=40)

    line_nose, = ax3d.plot([0, 1], [0, 0], [0, 0],
                            color=ACCENT_BLUE, lw=2.5, label="Nose (body X)")
    head_nose, = ax3d.plot([1], [0], [0], color=ACCENT_BLUE, marker="o", ms=6)

    # Attitude vector = reconstructed quat_to_accel(q, g)/g, matching
    # attitude_view.py's "Attitude (pred g)" convention.
    line_att, = ax3d.plot([0, 0], [0, 0], [0, -1],
                          color="#aa66ff", lw=2.5, label="Attitude (pred g)")
    head_att, = ax3d.plot([0], [0], [-1], color="#aa66ff", marker="o", ms=6)

    line_mag, = ax3d.plot([0, 0], [0, 0], [0, 0],
                          color=ACCENT_RED, lw=2.5, label="R\u00b7m_meas")
    head_mag, = ax3d.plot([0], [0], [0], color=ACCENT_RED, marker="o", ms=6)

    line_tilt, = ax3d.plot([0, 0], [0, 0], [0, 0],
                           color=ACCENT_ORANGE, lw=2.0, ls="--",
                           label="tilt-comp mag")
    head_tilt, = ax3d.plot([0], [0], [0], color=ACCENT_ORANGE, marker="o", ms=5)

    ax3d.legend(loc="upper left", fontsize=7, framealpha=0.3,
                facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # --- View buttons (face views) ---
    view_axes = [ax3d]

    def set_3d_view(elev, azim):
        for ax_view in view_axes:
            ax_view.view_init(elev=elev, azim=azim)
        fig.canvas.draw_idle()

    views = {"Top": (90, 180), "Front": (0, 0), "Back": (0, 180),
             "Left": (0, 90), "Right": (0, -90), "Iso": (18, -60)}
    view_btns = []
    for i, (label, (elev, azim)) in enumerate(views.items()):
        bx = fig.add_axes([0.03 + i * 0.055, 0.485, 0.05, 0.025])
        b = Button(bx, label, color=BTN_COLOR, hovercolor=BTN_HOVER)
        b.label.set_color(TEXT_COLOR)
        b.label.set_fontsize(7)
        b.on_clicked(lambda event, e=elev, a=azim: set_3d_view(e, a))
        view_btns.append(b)

    # --- Panel 2: yaw tracking + field-vector angular error (bottom-left) ---
    ax_err = fig.add_axes([0.05, 0.10, 0.42, 0.30])
    ax_err.set_facecolor(BG_COLOR)
    ax_err.set_title("Yaw tracking / magnetic field angle error",
                   fontsize=10, color=TEXT_COLOR, pad=4)
    ax_err.set_xlim(-30, 0)
    ax_err.set_ylim(-180, 180)
    ax_err.set_xlabel("seconds (rolling)", fontsize=8)
    ax_err.set_ylabel("degrees", fontsize=8)
    ax_err.grid(True, alpha=0.2)
    ax_err.axhline(0, color=GRID_COLOR, lw=0.6)
    ax_err.axhline(10, color=GRID_COLOR, lw=0.5, ls="--", alpha=0.45)
    ax_err.axhline(-10, color=GRID_COLOR, lw=0.5, ls="--", alpha=0.45)
    ax_err.axhline(90, color=GRID_COLOR, lw=0.5, ls=":", alpha=0.35)
    ax_err.axhline(-90, color=GRID_COLOR, lw=0.5, ls=":", alpha=0.35)
    yaw_est_line, = ax_err.plot([], [], color=ACCENT_BLUE, lw=1.3,
                                label="yaw_est")
    mag_yaw_line, = ax_err.plot([], [], color=ACCENT_ORANGE, lw=1.1,
                                label="mag yaw")
    field_err_line, = ax_err.plot([], [], color=ACCENT_RED, lw=1.0, alpha=0.85,
                                  label="angle(R·m, m_ref)")
    ax_err.legend(loc="upper right", fontsize=7, framealpha=0.4,
                facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # --- Panel 3: magnetic inclination check (bottom-right) ---
    ax_incl = fig.add_axes([0.55, 0.10, 0.42, 0.30])
    ax_incl.set_facecolor(BG_COLOR)
    ax_incl.set_title("Magnetic inclination check",
                     fontsize=10, color=TEXT_COLOR, pad=4)
    ax_incl.set_xlim(-30, 0)
    ax_incl.set_ylim(-90, 90)
    ax_incl.set_xlabel("seconds (rolling)", fontsize=8)
    ax_incl.set_ylabel("inclination (deg, +down)", fontsize=8, color=ACCENT_YELLOW)
    ax_incl.tick_params(axis="y", labelcolor=ACCENT_YELLOW)
    ax_incl.grid(True, alpha=0.2)
    ax_incl.axhline(INCL_DEG, color=ACCENT_GREEN, lw=1.0, ls="--",
                   label=f"ref {INCL_DEG:+.1f}\u00b0")
    ax_incl.axhline(0, color=GRID_COLOR, lw=0.5)
    incl_line, = ax_incl.plot([], [], color=ACCENT_YELLOW, lw=1.2,
                              label="measured R·m inclination")
    ax_incl.legend(loc="upper left", fontsize=7, framealpha=0.4,
                   facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # --- Panel 4: Quaternion/local frame view (top-right) ---
    ax_q = fig.add_axes([0.53, 0.53, 0.44, 0.40], projection="3d")
    view_axes.append(ax_q)
    ax_q.set_facecolor(BG_COLOR)
    ax_q.set_title("Quaternion view: earth frame in local/body axes",
                   fontsize=10, color=TEXT_COLOR, pad=4)
    ax_q.set_box_aspect((1, 1, 1))
    for pane in (ax_q.xaxis.pane, ax_q.yaxis.pane, ax_q.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor(GRID_COLOR)
    ax_q.set_xlim(-1.2, 1.2)
    ax_q.set_ylim(-1.2, 1.2)
    ax_q.set_zlim(-1.2, 1.2)
    ax_q.invert_yaxis()
    ax_q.set_xlabel("X — Forward", fontsize=8, labelpad=4)
    ax_q.set_ylabel("Y — Right", fontsize=8, labelpad=4)
    ax_q.set_zlabel("–Z (Up)", fontsize=8, labelpad=4)
    ax_q.view_init(elev=0, azim=180)
    ax_q.tick_params(labelsize=6)

    # Fixed local/body reference axes.
    ax_q.plot([0, 1], [0, 0], [0, 0], color=ACCENT_BLUE, lw=1.2,
              alpha=0.45, label="Body X/Fwd")
    ax_q.plot([0, 0], [0, 1], [0, 0], color="#aa66ff", lw=1.2,
              alpha=0.45, label="Body Y/Right")
    ax_q.plot([0, 0], [0, 0], [0, -1], color=GRID_COLOR, lw=1.2,
              alpha=0.75, label="Body Up (-Z)")

    # Earth frame axes rotated into the local/body frame. Earth Up is the same
    # vector as attitude_view.py's predicted accelerometer direction.
    q_north_line, = ax_q.plot([0, 1], [0, 0], [0, 0], color=ACCENT_GREEN,
                               lw=2.3, label="Earth N")
    q_north_head, = ax_q.plot([1], [0], [0], color=ACCENT_GREEN, marker="o", ms=5)
    q_east_line, = ax_q.plot([0, 0], [0, 1], [0, 0], color=ACCENT_ORANGE,
                              lw=2.0, label="Earth E")
    q_east_head, = ax_q.plot([0], [1], [0], color=ACCENT_ORANGE, marker="o", ms=4)
    q_up_line, = ax_q.plot([0, 0], [0, 0], [0, -1], color="#cccccc",
                            lw=2.2, label="Earth Up / pred g")
    q_up_head, = ax_q.plot([0], [0], [-1], color="#cccccc", marker="o", ms=4)
    q_text = ax_q.text2D(0.02, 0.92, "", transform=ax_q.transAxes,
                         fontsize=8, color=DIM_TEXT)
    ax_q.legend(loc="upper right", fontsize=6, framealpha=0.35,
                facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # --- Status bar ---
    chip_text = fig.text(0.02, 0.025,
                         f"decl={DECL_DEG:+.1f}\u00b0  incl={INCL_DEG:+.1f}\u00b0",
                         fontsize=8, ha="left", color=DIM_TEXT)
    stats_text = fig.text(0.50, 0.025, "",
                          fontsize=8, ha="center", color=DIM_TEXT)
    fps_text = fig.text(0.74, 0.025, "", fontsize=8, ha="right", color=DIM_TEXT)

    # --- Buttons ---
    ax_toggle = fig.add_axes([0.76, 0.018, 0.10, 0.035])
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
            send_log_class_command(g_serial, LOG_CLASS_MAG_FUSION)
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

    # --- Rolling buffers (30 s @ 25 Hz = 750 samples) ---
    BUF_LEN = 1000
    t_buf  = deque(maxlen=BUF_LEN)
    yaw_est_buf = deque(maxlen=BUF_LEN)
    mag_yaw_buf = deque(maxlen=BUF_LEN)
    yaw_err_buf = deque(maxlen=BUF_LEN)
    field_err_buf = deque(maxlen=BUF_LEN)
    incl_buf = deque(maxlen=BUF_LEN)
    y_norm_buf = deque(maxlen=BUF_LEN)
    nis_buf = deque(maxlen=BUF_LEN)
    rs_buf  = deque(maxlen=BUF_LEN)
    last_status = {
        "rscale": 0.0,
        "reject_text": "n/a",
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "yaw_deg": 0.0,
    }

    t_start = time.time()
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

        now = time.time()
        if latest is not None:
            m_meas = np.array(latest[0:3])
            m_pred = np.array(latest[3:6])
            nis    = float(latest[6])
            rscale = float(latest[7])
            roll   = math.radians(latest[8])
            pitch  = math.radians(latest[9])
            yaw    = math.radians(latest[10])
            reject_reason = int(round(latest[11])) if len(latest) > 11 else 0
            reject_text = REJECT_REASONS.get(reject_reason, f"code-{reject_reason}")
            last_status.update({
                "rscale": rscale,
                "reject_text": reject_text,
                "roll_deg": latest[8],
                "pitch_deg": latest[9],
                "yaw_deg": latest[10],
            })

            # 3D scene vectors. Mag/nose are earth-frame diagnostics. The
            # attitude vector reconstructs attitude_view.py's pred-g vector:
            # quat_to_accel(q,g)/g = R(q)^T · [0,0,-1].
            R = rot_zyx(roll, pitch, yaw)
            R_lvl = rot_zyx_no_yaw(roll, pitch)
            nose = R @ np.array([1.0, 0.0, 0.0])
            att = R.T @ np.array([0.0, 0.0, -1.0])
            earth_n_body = R.T @ np.array([1.0, 0.0, 0.0])
            earth_e_body = R.T @ np.array([0.0, 1.0, 0.0])
            mag_e = R @ m_meas
            # Visualization tilt-comp = horizontal projection of mag_e in earth
            # frame (rotates with body so it lines up with the m_ned reference
            # arrow only when yaw is correct).
            mag_lvl = mag_e.copy()
            mag_lvl[2] = 0.0
            n = np.linalg.norm(mag_lvl)
            if n > 1e-6:
                mag_lvl = mag_lvl / n
            # Body-level frame mag (yaw stripped) — used for the yaw_meas
            # heading calculation below.
            mag_body_lvl = R_lvl @ m_meas

            line_nose.set_data([0, nose[0]], [0, nose[1]])
            line_nose.set_3d_properties([0, nose[2]])
            head_nose.set_data([nose[0]], [nose[1]])
            head_nose.set_3d_properties([nose[2]])

            line_att.set_data([0, att[0]], [0, att[1]])
            line_att.set_3d_properties([0, att[2]])
            head_att.set_data([att[0]], [att[1]])
            head_att.set_3d_properties([att[2]])

            line_mag.set_data([0, mag_e[0]], [0, mag_e[1]])
            line_mag.set_3d_properties([0, mag_e[2]])
            head_mag.set_data([mag_e[0]], [mag_e[1]])
            head_mag.set_3d_properties([mag_e[2]])

            line_tilt.set_data([0, mag_lvl[0]], [0, mag_lvl[1]])
            line_tilt.set_3d_properties([0, mag_lvl[2]])
            head_tilt.set_data([mag_lvl[0]], [mag_lvl[1]])
            head_tilt.set_3d_properties([mag_lvl[2]])

            q_north_line.set_data([0, earth_n_body[0]], [0, earth_n_body[1]])
            q_north_line.set_3d_properties([0, earth_n_body[2]])
            q_north_head.set_data([earth_n_body[0]], [earth_n_body[1]])
            q_north_head.set_3d_properties([earth_n_body[2]])

            q_east_line.set_data([0, earth_e_body[0]], [0, earth_e_body[1]])
            q_east_line.set_3d_properties([0, earth_e_body[2]])
            q_east_head.set_data([earth_e_body[0]], [earth_e_body[1]])
            q_east_head.set_3d_properties([earth_e_body[2]])

            q_up_line.set_data([0, att[0]], [0, att[1]])
            q_up_line.set_3d_properties([0, att[2]])
            q_up_head.set_data([att[0]], [att[1]])
            q_up_head.set_3d_properties([att[2]])

            # Time-series buffers
            t_rel = now - t_start
            t_buf.append(t_rel)
            y = m_meas - m_pred

            # Yaw: estimated comes from frame; measured = level-frame mag yaw
            #   yaw_meas = atan2(-mag_body_lvl.y, mag_body_lvl.x) - declination
            yaw_meas = math.atan2(-mag_body_lvl[1], mag_body_lvl[0]) - decl
            yaw_meas_deg = math.degrees(math.atan2(math.sin(yaw_meas),
                                                    math.cos(yaw_meas)))
            yaw_err = math.degrees(math.atan2(math.sin(yaw - yaw_meas),
                                              math.cos(yaw - yaw_meas)))
            mag_e_norm = max(float(np.linalg.norm(mag_e)), 1e-9)
            field_dot = float(np.clip(np.dot(mag_e, m_ned_unit) / mag_e_norm,
                                      -1.0, 1.0))
            field_err = math.degrees(math.acos(field_dot))
            incl_meas = math.degrees(math.asin(float(np.clip(mag_e[2] / mag_e_norm,
                                                           -1.0, 1.0))))

            y_norm_buf.append(float(np.linalg.norm(y)))
            yaw_est_buf.append(latest[10])
            mag_yaw_buf.append(yaw_meas_deg)
            yaw_err_buf.append(yaw_err)
            field_err_buf.append(field_err)
            incl_buf.append(incl_meas)
            nis_buf.append(nis); rs_buf.append(rscale)

            q_text.set_text(
                f"yaw={latest[10]:+.1f}\u00b0  mag={yaw_meas_deg:+.1f}\u00b0  err={yaw_err:+.1f}\u00b0")

        # Always re-draw the time-series with the rolling window relative to now.
        if t_buf:
            t_arr = np.array(t_buf) - (now - t_start)
            # Trim to last 30 s for display only.
            mask = t_arr >= -30.0
            t_v = t_arr[mask]
            yaw_est_v = unwrap_deg(np.array(yaw_est_buf)[mask])
            mag_yaw_v = unwrap_deg(np.array(mag_yaw_buf)[mask])
            field_err_v = np.array(field_err_buf)[mask]
            yaw_est_line.set_data(t_v, yaw_est_v)
            mag_yaw_line.set_data(t_v, mag_yaw_v)
            field_err_line.set_data(t_v, field_err_v)
            incl_line.set_data(t_v, np.array(incl_buf)[mask])
            if len(t_v):
                y_min = float(np.nanmin([np.nanmin(yaw_est_v), np.nanmin(mag_yaw_v), np.nanmin(field_err_v), -10.0]))
                y_max = float(np.nanmax([np.nanmax(yaw_est_v), np.nanmax(mag_yaw_v), np.nanmax(field_err_v), 10.0]))
                pad = max(15.0, 0.08 * (y_max - y_min))
                ax_err.set_ylim(y_min - pad, y_max + pad)

            # Stats on last 5 s
            mask5 = t_arr >= -5.0
            if mask5.any():
                mag_y = float(np.mean(np.array(y_norm_buf)[mask5]))
                mean_nis = float(np.mean(np.array(nis_buf)[mask5]))
                mean_yaw_err = float(np.mean(np.abs(np.array(yaw_err_buf)[mask5])))
                stats_text.set_text(
                    f"|y|={mag_y:.3f}  yaw_err={mean_yaw_err:.1f}\u00b0  NIS={mean_nis:.2f}  "
                    f"R={last_status['rscale']:.2f}  mag={last_status['reject_text']}  "
                    f"r={last_status['roll_deg']:+.1f}\u00b0 p={last_status['pitch_deg']:+.1f}\u00b0 "
                    f"y={last_status['yaw_deg']:+.1f}\u00b0")

        if g_chip_id is not None:
            chip_text.set_text(
                f"Chip {g_chip_id}  decl={DECL_DEG:+.1f}\u00b0  incl={INCL_DEG:+.1f}\u00b0")

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

        return ()

    _anim = FuncAnimation(fig, update, interval=40, blit=False,
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
