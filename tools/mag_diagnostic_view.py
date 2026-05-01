"""
Mag Diagnostic Viewer (minimal)
================================

Single 3D panel showing, in the earth NED frame:
    * Body axes from the ESKF roll/pitch/yaw
    * Raw measured mag vector R(q)·m_meas (red)
    * Tilt-compensated mag (orange) = horizontal projection of the raw
      vector onto the N-E plane

Wire format (7 floats / 28 B, LOG_CLASS_MAG_FUSION = 0x1F):
    [0..2]  m_meas  (body NED, unit)
    [3..5]  roll, pitch, yaw  (deg)
    [6]     mag_heading_deg  (decl-corrected, [-180, 180])
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
LOG_CLASS_MAG_FUSION = 0x1F

MAG_FRAME_SIZE = 28            # 7 floats
MAG_FRAME_FORMAT = "<7f"

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
    global g_serial, g_chip_id
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
            elif msg_id == SEND_LOG_ID and length == MAG_FRAME_SIZE:
                vals = struct.unpack(MAG_FRAME_FORMAT, payload)
                if all(math.isfinite(v) for v in vals):
                    push_latest(vals)
    except Exception as e:
        print(f"Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


def quat_from_euler_zyx(roll, pitch, yaw):
    """Body→earth rotation matrix from Tait-Bryan ZYX."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr],
    ])


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

    fig = plt.figure(figsize=screen_fit_figsize(10, 10))
    fig.patch.set_facecolor(BG_COLOR)
    fig.canvas.manager.set_window_title("Mag Diagnostic Viewer")
    fig.suptitle("State Estimation \u2014 Magnetometer Diagnostics",
                 fontsize=14, color=TEXT_COLOR, fontweight="bold", y=0.99)

    ax = fig.add_axes([0.05, 0.10, 0.90, 0.83], projection="3d")
    ax.set_facecolor(BG_COLOR)
    ax.set_box_aspect((1, 1, 1))
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor(GRID_COLOR)
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)
    # Earth NED frame, back-camera (azim=180, elev=0): invert_yaxis() puts
    # +Y (East / pilot's right) on the viewer's right; invert_zaxis() puts
    # +Z (Down) at the bottom of the screen. Body axes are drawn as
    # R(q)·e_i so they tilt with the body (roll left = body axes tilt left).
    ax.invert_yaxis()
    ax.invert_zaxis()
    ax.set_xlabel("X \u2014 North", fontsize=8, labelpad=4)
    ax.set_ylabel("Y \u2014 East",  fontsize=8, labelpad=4)
    ax.set_zlabel("Z \u2014 Down",  fontsize=8, labelpad=4)
    ax.view_init(elev=0, azim=180)  # default = Back view
    ax.tick_params(labelsize=6)

    # Reference unit sphere
    u = np.linspace(0, 2*np.pi, 24)
    v = np.linspace(0, np.pi, 12)
    xs = np.outer(np.cos(u), np.sin(v))
    ys = np.outer(np.sin(u), np.sin(v))
    zs = np.outer(np.ones_like(u), np.cos(v))
    ax.plot_wireframe(xs, ys, zs, color=GRID_COLOR, alpha=0.08, linewidth=0.3)

    # Static earth-frame axes (faded reference: NED).
    ax.plot([0, 1], [0, 0], [0, 0], color="#9a9a9a", lw=1.2, alpha=0.85, label="Earth N")
    ax.plot([0, 0], [0, 1], [0, 0], color="#9a9a9a", lw=1.2, alpha=0.85, label="Earth E")
    ax.plot([0, 0], [0, 0], [0, 1], color="#9a9a9a", lw=1.2, alpha=0.85, label="Earth Down")

    # Body axes expressed in earth frame (R(q) · e_i). These tilt the same
    # direction the body tilts — roll left makes them tilt left on screen.
    body_x_line, = ax.plot([0, 1], [0, 0], [0, 0], color=ACCENT_BLUE,
                            lw=2.2, label="Body X (Forward)")
    body_x_head, = ax.plot([1], [0], [0], color=ACCENT_BLUE, marker="o", ms=5)
    body_y_line, = ax.plot([0, 0], [0, 1], [0, 0], color=ACCENT_PURPLE,
                            lw=1.8, label="Body Y (Right)")
    body_y_head, = ax.plot([0], [1], [0], color=ACCENT_PURPLE, marker="o", ms=4)
    body_z_line, = ax.plot([0, 0], [0, 0], [0, 1], color=ACCENT_GREEN,
                            lw=2.0, label="Body Z (Down)")
    body_z_head, = ax.plot([0], [0], [1], color=ACCENT_GREEN, marker="o", ms=4)

    # Mag vectors in earth frame. Raw = R·m_meas (full mag, tilts with the
    # local field vector). Tilt-comp = horizontal projection of raw onto the
    # N-E plane (z=0) — points toward magnetic north regardless of attitude.
    raw_line,  = ax.plot([0, 0], [0, 0], [0, 0], color=ACCENT_RED, lw=2.5,
                         label="Raw mag (R·m_meas)")
    raw_head,  = ax.plot([0], [0], [0], color=ACCENT_RED, marker="o", ms=6)
    tilt_line, = ax.plot([0, 0], [0, 0], [0, 0], color=ACCENT_ORANGE, lw=2.5,
                         label="Tilt-comp mag (N-E plane)")
    tilt_head, = ax.plot([0], [0], [0], color=ACCENT_ORANGE, marker="o", ms=6)

    ax.legend(loc="upper left", fontsize=7, framealpha=0.3,
              facecolor=PANEL_COLOR, edgecolor=GRID_COLOR, labelcolor=TEXT_COLOR)

    # --- View buttons (6 faces) ---
    views = {"Top": (90, 180), "Front": (0, 0), "Back": (0, 180),
             "Left": (0, 90), "Right": (0, -90), "Iso": (18, -60)}
    view_btns = []
    for i, (label, (elev, azim)) in enumerate(views.items()):
        bx = fig.add_axes([0.03 + i * 0.065, 0.07, 0.06, 0.035])
        b = Button(bx, label, color=BTN_COLOR, hovercolor=BTN_HOVER)
        b.label.set_color(TEXT_COLOR)
        b.label.set_fontsize(7)
        b.on_clicked(lambda event, e=elev, a=azim: ax.view_init(elev=e, azim=a))
        view_btns.append(b)

    chip_text = fig.text(0.02, 0.025, "", fontsize=8, ha="left", color=DIM_TEXT)
    status_text = fig.text(0.50, 0.025, "", fontsize=8, ha="center", color=DIM_TEXT)

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
            send_log_class(g_serial, LOG_CLASS_MAG_FUSION)
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

        def _after():
            time.sleep(2.0)
            if g_serial and g_serial.is_open:
                g_serial.reset_input_buffer()
                send_log_class(g_serial, LOG_CLASS_HEART_BEAT)
        threading.Thread(target=_after, daemon=True).start()

    btn_reset.on_clicked(on_reset)

    def update(_):
        latest = None
        while True:
            try:
                latest = data_queue.get_nowait()
            except queue.Empty:
                break
        if latest is None:
            return ()

        m_meas = np.array(latest[0:3])
        roll   = math.radians(latest[3])
        pitch  = math.radians(latest[4])
        yaw    = math.radians(latest[5])
        heading_deg = float(latest[6])

        # Render in EARTH frame (NED). Body axes = R(q) · e_i so they tilt
        # the same direction the body tilts.
        R = quat_from_euler_zyx(roll, pitch, yaw)
        b_x_e = R @ np.array([1.0, 0.0, 0.0])
        b_y_e = R @ np.array([0.0, 1.0, 0.0])
        b_z_e = R @ np.array([0.0, 0.0, 1.0])
        # Mag in earth frame.
        raw_e = R @ m_meas
        # Tilt-compensated mag in earth frame = horizontal projection of
        # raw_e (z forced to 0). This vector lies on the N-E plane and
        # points toward magnetic north regardless of roll/pitch — that's
        # exactly what "tilt compensation" means visually.
        # NOTE: do NOT reconstruct it as Rz(yaw)·m_lvl — that just recovers
        # raw_e (because firmware already computed m_lvl = Rz(-yaw)·R·m),
        # making the two arrows overlap.
        tilt_e = np.array([raw_e[0], raw_e[1], 0.0])
        n = np.linalg.norm(tilt_e)
        if n > 1e-6:
            tilt_e = tilt_e * (np.linalg.norm(raw_e) / n)

        for line, head, vec in (
            (raw_line,  raw_head,  raw_e),
            (tilt_line, tilt_head, tilt_e),
            (body_x_line, body_x_head, b_x_e),
            (body_y_line, body_y_head, b_y_e),
            (body_z_line, body_z_head, b_z_e),
        ):
            line.set_data([0, vec[0]], [0, vec[1]])
            line.set_3d_properties([0, vec[2]])
            head.set_data([vec[0]], [vec[1]])
            head.set_3d_properties([vec[2]])

        status_text.set_text(
            f"r={latest[3]:+.1f}°  p={latest[4]:+.1f}°  "
            f"yaw={latest[5]:+.1f}°  mag_heading={heading_deg:+.1f}°"
        )
        if g_chip_id is not None:
            chip_text.set_text(f"Chip {g_chip_id}")
        return ()

    _anim = FuncAnimation(fig, update, interval=40, blit=False,
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
