"""
Baro Fusion Viewer
==================

Validates the barometric altitude → fusion6 path. Streams a focused diagnostic
that lets you confirm sign convention (NED +Z = Down, baro reports +up so the
FC negates), the snap-on-first-frame behaviour, and that the ESKF p.z tracks
the raw measurement during a hand lift / drop.

Wire format (5 floats / 20 B, LOG_CLASS_BARO_FUSION = 0x21):
    [0] z_meas       (NED metres on p.z; +down, after FC sign flip)
    [1] p_z          (current ESKF p.z, NED, m)
    [2] v_z          (current ESKF v.z, NED, m/s)
    [3] innovation   (z_meas − p_z, m; non-zero even when sanity-gated)
    [4] baro_ok      (1.0 if FUSION6_HF_BARO_OK set, else 0.0)

Display convention: the strips show **altitude above startup = -p_z** so
"up = up" on screen, regardless of NED sign. The numeric status row shows
the raw NED values for debugging.
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
LOG_CLASS_BARO_FUSION = 0x21

BARO_FRAME_SIZE = 20            # 5 floats
BARO_FRAME_FORMAT = "<5f"

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
BTN_GREEN = "#2d5a2d"
BTN_GREEN_HOV = "#3d7a3d"
BTN_RED = "#5a2d2d"
BTN_RED_HOV = "#7a3d3d"

WINDOW_S = 30.0
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
    """Same non-blocking parser pattern as optflow_velocity_view.py."""
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
                elif msg_id == SEND_LOG_ID and length == BARO_FRAME_SIZE:
                    vals = struct.unpack(BARO_FRAME_FORMAT, payload)
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

    fig = plt.figure(figsize=screen_fit_figsize(13, 8))
    fig.patch.set_facecolor(BG_COLOR)
    fig.canvas.manager.set_window_title("Baro Fusion Viewer")
    fig.suptitle("State Estimation — Barometric Altitude Fusion",
                 fontsize=14, color=TEXT_COLOR, fontweight="bold", y=0.99)

    # 3 stacked strips:
    #   1. altitude (z_meas vs p_z, both flipped to +up)
    #   2. v.z (NED, +down)  — caller reads sign on screen
    #   3. innovation (m), with ±50 m sanity gate guides
    gs = fig.add_gridspec(3, 1, left=0.07, right=0.97, top=0.94, bottom=0.18,
                          hspace=0.30, height_ratios=[3, 2, 2])
    ax_alt = fig.add_subplot(gs[0, 0])
    ax_vz  = fig.add_subplot(gs[1, 0], sharex=ax_alt)
    ax_inn = fig.add_subplot(gs[2, 0], sharex=ax_alt)

    for axp in (ax_alt, ax_vz, ax_inn):
        axp.set_facecolor(BG_COLOR)
        axp.grid(True, color=GRID_COLOR, alpha=0.4, linewidth=0.5)
        for sp in axp.spines.values():
            sp.set_color(GRID_COLOR)
        axp.set_autoscaley_on(True)

    ax_alt.set_ylabel("altitude (m, +up)", color=TEXT_COLOR, fontsize=9)
    ax_vz.set_ylabel("v.z NED (m/s, +down)", color=TEXT_COLOR, fontsize=9)
    ax_inn.set_ylabel("innovation (m)", color=TEXT_COLOR, fontsize=9)
    ax_inn.set_xlabel("time (s)", color=TEXT_COLOR, fontsize=9)
    plt.setp(ax_alt.get_xticklabels(), visible=False)
    plt.setp(ax_vz.get_xticklabels(),  visible=False)

    # Innovation strip: skip the ±50 m sanity-gate guides as static lines
    # — they would force autoscale to include ±50 m on every redraw and
    # squash the actual sub-metre innovation flat. The [GATED] tag in the
    # status row already calls out violations, and we still autoscale
    # purely from the innovation data below.
    ax_inn.axhline(0.0, color=GRID_COLOR, lw=0.6)
    ax_vz.axhline( 0.0, color=GRID_COLOR, lw=0.6)

    # Buffers
    maxlen = int(WINDOW_S * RATE_HZ) + 8
    t_buf   = deque(maxlen=maxlen)
    z_meas  = deque(maxlen=maxlen)   # +up after sign flip for display
    p_z     = deque(maxlen=maxlen)   # +up after sign flip for display
    v_z     = deque(maxlen=maxlen)   # raw NED (+down)
    innov   = deque(maxlen=maxlen)   # raw NED metres (z_meas - p_z, both +down)
    ok_flag = deque(maxlen=maxlen)

    # Lines
    l_z_meas, = ax_alt.plot([], [], color=ACCENT_GREEN, lw=1.4,
                            label="baro z_meas")
    l_p_z,    = ax_alt.plot([], [], color=ACCENT_BLUE,  lw=1.8, alpha=0.9,
                            label="ESKF p.z")
    l_v_z,    = ax_vz.plot([],  [], color=ACCENT_YELLOW, lw=1.4,
                           label="ESKF v.z")
    l_innov,  = ax_inn.plot([], [], color=ACCENT_ORANGE, lw=1.4,
                            label="z_meas − p.z")

    for axp in (ax_alt, ax_vz, ax_inn):
        axp.legend(loc="upper right", fontsize=7, framealpha=0.3,
                   facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                   labelcolor=TEXT_COLOR)

    # --- Bottom info row -----------------------------------------------
    chip_text   = fig.text(0.02, 0.06, "", fontsize=8, ha="left",
                           color=DIM_TEXT)
    fps_text    = fig.text(0.50, 0.06, "", fontsize=8, ha="center",
                           color=DIM_TEXT)
    status_text = fig.text(0.50, 0.10, "", fontsize=9, ha="center",
                           color=TEXT_COLOR, family="monospace")
    ok_lamp     = fig.text(0.95, 0.06, "BARO_OK", fontsize=9, ha="right",
                           color=DIM_TEXT, fontweight="bold")

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
            send_log_class(g_serial, LOG_CLASS_BARO_FUSION)
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
        for d in (t_buf, z_meas, p_z, v_z, innov, ok_flag):
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
        for d in (t_buf, z_meas, p_z, v_z, innov, ok_flag):
            d.clear()
        t0[0] = None
        for axp in (ax_alt, ax_vz, ax_inn):
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

        z_meas_ned = latest[0]   # NED, +down
        p_z_ned    = latest[1]   # NED, +down
        v_z_ned    = latest[2]   # NED, +down
        innov_m    = latest[3]   # NED, +down (z_meas - p_z)
        baro_ok    = latest[4] >= 0.5

        # Display altitude with +up convention so visual matches intuition.
        alt_meas_up = -z_meas_ned
        p_z_up      = -p_z_ned

        t_buf.append(t)
        z_meas.append(alt_meas_up)
        p_z.append(p_z_up)
        v_z.append(v_z_ned)
        innov.append(innov_m)
        ok_flag.append(1.0 if baro_ok else 0.0)

        while t_buf and (t - t_buf[0]) > WINDOW_S:
            t_buf.popleft()
            for d in (z_meas, p_z, v_z, innov, ok_flag):
                d.popleft()

        ts = list(t_buf)
        l_z_meas.set_data(ts, list(z_meas))
        l_p_z.set_data(ts, list(p_z))
        l_v_z.set_data(ts, list(v_z))
        l_innov.set_data(ts, list(innov))

        if ts:
            x_lo = max(0.0, ts[-1] - WINDOW_S)
            x_hi = max(WINDOW_S, ts[-1])
            ax_alt.set_xlim(x_lo, x_hi)
            for axp in (ax_alt, ax_vz, ax_inn):
                axp.relim()
                axp.set_autoscaley_on(True)
                axp.autoscale_view(scalex=False, scaley=True)

        # OK lamp colour
        ok_lamp.set_color(ACCENT_GREEN if baro_ok else ACCENT_RED)

        # Highlight innovation in red if it's outside ±50 m (i.e. would be
        # rejected by the FC's BARO_SANITY_GATE_M).
        gated = abs(innov_m) > 50.0
        innov_str = f"{innov_m:+.3f}"
        if gated:
            innov_str += " [GATED]"

        status_text.set_text(
            f"z_meas={z_meas_ned:+8.3f} m  "
            f"p.z={p_z_ned:+8.3f} m  "
            f"v.z={v_z_ned:+6.3f} m/s  "
            f"innov={innov_str}"
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
