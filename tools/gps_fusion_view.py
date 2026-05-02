"""
GPS Fusion Viewer
=================

Validates the GPS pos/vel → fusion6 path (NED-frame scalar updates added
in robotkit/fusion6.c). Streams raw GPS measurements alongside the current
ESKF state so you can confirm:

  * the lazy NED origin captured (raw GPS pos starts near 0, 0, 0);
  * ESKF position snaps onto GPS within a few seconds of GPS_OK;
  * walking outdoors traces matching ground tracks;
  * GPS_OK lamp toggles in step with quality-gate transitions.

Wire format (14 floats / 56 B, LOG_CLASS_GPS_FUSION = 0x23, @ 5 Hz):
    [0..2]   gps_pos_ned   (m,   NED, lazy-origin-relative; NaN = no fix)
    [3..5]   gps_vel_ned   (m/s, NED;                       NaN = no fix)
    [6..8]   eskf_p        (m,   NED)
    [9..11]  eskf_v        (m/s, NED)
    [12]     gps_ok        (1.0 if FUSION6_HF_GPS_OK set, else 0.0)
    [13]     num_sv        (last reported satellite count, as float)

Display convention: GPS in green, ESKF in cyan/blue. The 2D plot uses
East on X and North on Y so "up the screen" = North. The Down strip is
NED-native (positive = down); altitude-above-origin can be eyeballed as
the negation.
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
LOG_CLASS_GPS_FUSION = 0x23

GPS_FRAME_SIZE = 56            # 14 floats
GPS_FRAME_FORMAT = "<14f"

# --- UI palette (shared with other tools/*.py viewers) ---
BG_COLOR = "#1e1e1e"
PANEL_COLOR = "#252526"
TEXT_COLOR = "#cccccc"
DIM_TEXT = "#888888"
GRID_COLOR = "#3c3c3c"
ACCENT_BLUE = "#5599ff"
ACCENT_CYAN = "#55ddff"
ACCENT_GREEN = "#55cc55"
ACCENT_RED = "#ff5555"
ACCENT_ORANGE = "#ff9955"
ACCENT_YELLOW = "#ffdd55"
BTN_GREEN = "#2d5a2d"
BTN_GREEN_HOV = "#3d7a3d"
BTN_RED = "#5a2d2d"
BTN_RED_HOV = "#7a3d3d"

WINDOW_S = 60.0       # 60 s of history (5 Hz → 300 samples per strip)
RATE_HZ = 5
TRACK_LEN = 1500      # 5 min of position history on the 2D ground track


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
    """Same non-blocking parser pattern as baro_fusion_view.py."""
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
                elif msg_id == SEND_LOG_ID and length == GPS_FRAME_SIZE:
                    vals = struct.unpack(GPS_FRAME_FORMAT, payload)
                    # ESKF fields (6..11) and the flags (12,13) must be
                    # finite; GPS fields (0..5) may be NaN before first fix.
                    if all(math.isfinite(v) for v in vals[6:]):
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

    fig = plt.figure(figsize=screen_fit_figsize(15, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.canvas.manager.set_window_title("GPS Fusion Viewer")
    fig.suptitle("State Estimation — GPS Position & Velocity Fusion",
                 fontsize=14, color=TEXT_COLOR, fontweight="bold", y=0.99)

    # Layout: ground track on the left, 6 strips on the right.
    gs = fig.add_gridspec(6, 2, left=0.06, right=0.97, top=0.94, bottom=0.18,
                          hspace=0.55, wspace=0.20, width_ratios=[3, 4])
    ax_track = fig.add_subplot(gs[:, 0])
    ax_pn = fig.add_subplot(gs[0, 1])
    ax_pe = fig.add_subplot(gs[1, 1], sharex=ax_pn)
    ax_pd = fig.add_subplot(gs[2, 1], sharex=ax_pn)
    ax_vn = fig.add_subplot(gs[3, 1], sharex=ax_pn)
    ax_ve = fig.add_subplot(gs[4, 1], sharex=ax_pn)
    ax_vd = fig.add_subplot(gs[5, 1], sharex=ax_pn)

    strip_axes = (ax_pn, ax_pe, ax_pd, ax_vn, ax_ve, ax_vd)
    strip_labels = ("p.N (m)", "p.E (m)", "p.D (m)",
                    "v.N (m/s)", "v.E (m/s)", "v.D (m/s)")
    for axp, lab in zip(strip_axes, strip_labels):
        axp.set_facecolor(BG_COLOR)
        axp.grid(True, color=GRID_COLOR, alpha=0.4, linewidth=0.5)
        for sp in axp.spines.values():
            sp.set_color(GRID_COLOR)
        axp.set_autoscaley_on(True)
        axp.set_ylabel(lab, color=TEXT_COLOR, fontsize=8)
        axp.axhline(0.0, color=GRID_COLOR, lw=0.5)
        axp.tick_params(labelsize=7)
        plt.setp(axp.get_xticklabels(), visible=False)
    ax_vd.set_xlabel("time (s)", color=TEXT_COLOR, fontsize=9)
    plt.setp(ax_vd.get_xticklabels(), visible=True)

    # 2D ground track
    ax_track.set_facecolor(BG_COLOR)
    ax_track.grid(True, color=GRID_COLOR, alpha=0.4, linewidth=0.5)
    for sp in ax_track.spines.values():
        sp.set_color(GRID_COLOR)
    ax_track.set_aspect("equal", adjustable="datalim")
    ax_track.set_xlabel("East (m)", color=TEXT_COLOR, fontsize=9)
    ax_track.set_ylabel("North (m)", color=TEXT_COLOR, fontsize=9)
    ax_track.set_title("Ground track (NED, origin = first GPS fix)",
                       color=TEXT_COLOR, fontsize=10)
    ax_track.axhline(0.0, color=GRID_COLOR, lw=0.6)
    ax_track.axvline(0.0, color=GRID_COLOR, lw=0.6)

    # Lines / scatters
    l_track_gps,  = ax_track.plot([], [], color=ACCENT_GREEN, lw=0,
                                  marker="o", markersize=3, alpha=0.8,
                                  label="GPS")
    l_track_eskf, = ax_track.plot([], [], color=ACCENT_CYAN, lw=1.4,
                                  alpha=0.9, label="ESKF")
    ax_track.legend(loc="upper right", fontsize=8, framealpha=0.3,
                    facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                    labelcolor=TEXT_COLOR)

    strip_lines_gps = []
    strip_lines_eskf = []
    for axp in strip_axes:
        lg, = axp.plot([], [], color=ACCENT_GREEN, lw=0, marker=".",
                       markersize=3, alpha=0.85, label="GPS")
        le, = axp.plot([], [], color=ACCENT_CYAN, lw=1.3, alpha=0.9,
                       label="ESKF")
        strip_lines_gps.append(lg)
        strip_lines_eskf.append(le)
    ax_pn.legend(loc="upper right", fontsize=7, framealpha=0.3,
                 facecolor=PANEL_COLOR, edgecolor=GRID_COLOR,
                 labelcolor=TEXT_COLOR)

    # Buffers
    maxlen = int(WINDOW_S * RATE_HZ) + 8
    t_buf       = deque(maxlen=maxlen)
    gps_pos     = [deque(maxlen=maxlen) for _ in range(3)]   # N,E,D
    gps_vel     = [deque(maxlen=maxlen) for _ in range(3)]
    eskf_pos    = [deque(maxlen=maxlen) for _ in range(3)]
    eskf_vel    = [deque(maxlen=maxlen) for _ in range(3)]

    # Track buffers (longer history, only finite GPS samples)
    track_gps_n = deque(maxlen=TRACK_LEN)
    track_gps_e = deque(maxlen=TRACK_LEN)
    track_eskf_n = deque(maxlen=TRACK_LEN * 2)  # ESKF arrives every frame
    track_eskf_e = deque(maxlen=TRACK_LEN * 2)

    # --- Bottom info row -----------------------------------------------
    chip_text   = fig.text(0.02, 0.06, "", fontsize=8, ha="left",
                           color=DIM_TEXT)
    fps_text    = fig.text(0.50, 0.06, "", fontsize=8, ha="center",
                           color=DIM_TEXT)
    status_text = fig.text(0.50, 0.10, "", fontsize=9, ha="center",
                           color=TEXT_COLOR, family="monospace")
    ok_lamp     = fig.text(0.95, 0.06, "GPS_OK", fontsize=9, ha="right",
                           color=DIM_TEXT, fontweight="bold")
    sv_text     = fig.text(0.85, 0.06, "", fontsize=9, ha="right",
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
            send_log_class(g_serial, LOG_CLASS_GPS_FUSION)
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

    def _clear_all():
        for d in (t_buf, *gps_pos, *gps_vel, *eskf_pos, *eskf_vel,
                  track_gps_n, track_gps_e, track_eskf_n, track_eskf_e):
            d.clear()
        t0[0] = None

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
        _clear_all()

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
        _clear_all()
        for axp in (ax_track, *strip_axes):
            axp.relim()
            axp.set_autoscaley_on(True)
            axp.autoscale_view()

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
            fps_text.set_text(f"RX {rx_hz:.1f} Hz | Draw {draw_hz:.0f} Hz")
            last_rx_count[0] = rx_count
            draw_count[0] = 0
            last_fps_t[0] = now

        if latest is None:
            return ()

        if t0[0] is None:
            t0[0] = now
        t = now - t0[0]

        gps_n, gps_e, gps_d = latest[0], latest[1], latest[2]
        gps_vn, gps_ve, gps_vd = latest[3], latest[4], latest[5]
        ep_n, ep_e, ep_d = latest[6], latest[7], latest[8]
        ev_n, ev_e, ev_d = latest[9], latest[10], latest[11]
        gps_ok = latest[12] >= 0.5
        num_sv = int(round(latest[13]))

        t_buf.append(t)
        gps_pos[0].append(gps_n);  gps_pos[1].append(gps_e);  gps_pos[2].append(gps_d)
        gps_vel[0].append(gps_vn); gps_vel[1].append(gps_ve); gps_vel[2].append(gps_vd)
        eskf_pos[0].append(ep_n);  eskf_pos[1].append(ep_e);  eskf_pos[2].append(ep_d)
        eskf_vel[0].append(ev_n);  eskf_vel[1].append(ev_e);  eskf_vel[2].append(ev_d)

        # Track buffers — only push GPS when finite, ESKF always.
        if math.isfinite(gps_n) and math.isfinite(gps_e):
            track_gps_n.append(gps_n)
            track_gps_e.append(gps_e)
        track_eskf_n.append(ep_n)
        track_eskf_e.append(ep_e)

        while t_buf and (t - t_buf[0]) > WINDOW_S:
            t_buf.popleft()
            for d in (*gps_pos, *gps_vel, *eskf_pos, *eskf_vel):
                d.popleft()

        ts = list(t_buf)
        for i in range(3):
            strip_lines_gps[i].set_data(ts, list(gps_pos[i]))
            strip_lines_eskf[i].set_data(ts, list(eskf_pos[i]))
        for i in range(3):
            strip_lines_gps[3 + i].set_data(ts, list(gps_vel[i]))
            strip_lines_eskf[3 + i].set_data(ts, list(eskf_vel[i]))

        # 2D ground track — note East on X, North on Y.
        l_track_gps.set_data(list(track_gps_e),  list(track_gps_n))
        l_track_eskf.set_data(list(track_eskf_e), list(track_eskf_n))

        if ts:
            x_lo = max(0.0, ts[-1] - WINDOW_S)
            x_hi = max(WINDOW_S, ts[-1])
            ax_pn.set_xlim(x_lo, x_hi)
            for axp in strip_axes:
                axp.relim()
                axp.set_autoscaley_on(True)
                axp.autoscale_view(scalex=False, scaley=True)

        # Track autoscale (small padding so a stationary point is still visible).
        if track_eskf_n:
            ax_track.relim()
            ax_track.autoscale_view()

        ok_lamp.set_color(ACCENT_GREEN if gps_ok else ACCENT_RED)
        sv_text.set_color(ACCENT_GREEN if num_sv >= 6 else
                          (ACCENT_ORANGE if num_sv >= 3 else ACCENT_RED))
        sv_text.set_text(f"sv={num_sv:2d}")

        if math.isfinite(gps_n):
            status_text.set_text(
                f"GPS  N={gps_n:+8.2f}  E={gps_e:+8.2f}  D={gps_d:+8.2f}  "
                f"|  ESKF N={ep_n:+8.2f}  E={ep_e:+8.2f}  D={ep_d:+8.2f}"
            )
        else:
            status_text.set_text(
                f"GPS  (no fix)                                   "
                f"|  ESKF N={ep_n:+8.2f}  E={ep_e:+8.2f}  D={ep_d:+8.2f}"
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
