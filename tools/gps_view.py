"""
gps_view.py — Real-time GPS dashboard for the flight-controller telemetry link.

Streams LOG_CLASS_GPS frames (gps_log_t, 48 bytes) over the dblink UART/USB link
and renders:

  • 2-D position map      — local NED scatter/trail referenced to the first fix
  • Altitude bar chart    — current MSL altitude with min/max history shown
  • Velocity line chart   — vN, vE, vD and 2-D ground speed
  • Sat / fix info panel  — num_sv, fix type, h_acc, v_acc, p_dop, reliability
  • Sat-count time series — useful while tuning antenna placement

Frame layout (LOG_CLASS_GPS = 0x1E, 48 bytes, host-side struct gps_log_t):
  [0..3]   float lat_deg
  [4..7]   float lon_deg
  [8..11]  float alt_msl_m
  [12..15] float vel_n_mps
  [16..19] float vel_e_mps
  [20..23] float vel_d_mps
  [24..27] float g_speed_mps
  [28..31] float head_mot_deg
  [32..35] float h_acc_m
  [36..39] float v_acc_m
  [40..41] uint16 p_dop  (× 0.01)
  [42]     uint8  num_sv
  [43]     uint8  fix_type
  [44]     uint8  flags
  [45]     uint8  reliable
  [46..47] uint16 _pad

Usage:
  python3 tools/gps_view.py
"""

import math
import queue
import struct
import sys
import threading
import time

import numpy as np
import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("macosx" if sys.platform == "darwin" else "TkAgg")
import matplotlib.pyplot as plt   # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402
from matplotlib.widgets import Button  # noqa: E402


# --- Protocol constants --------------------------------------------------

BAUD_RATE       = 19200
SEND_LOG_ID     = 0x00
DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET    = 0x07
LOG_CLASS_NONE  = 0x00
LOG_CLASS_HEART_BEAT = 0x09
LOG_CLASS_GPS   = 0x1E

GPS_FRAME_SIZE  = 48      # struct gps_log_t
GPS_STRUCT_FMT  = "<10fHBBBBH"  # 10 floats + u16 + 4×u8 + u16 pad

HISTORY_LEN     = 600     # ~60 s at 10 Hz

# Earth radius for the lat/lon → metres approximation around the home fix.
EARTH_R_M = 6_378_137.0


# --- UI palette (matches every other tool) ------------------------------

BG_COLOR     = "#1e1e1e"
PANEL_COLOR  = "#252526"
TEXT_COLOR   = "#cccccc"
DIM_TEXT     = "#888888"
GRID_COLOR   = "#3c3c3c"
ACCENT_BLUE  = "#5599ff"
ACCENT_GREEN = "#55cc55"
ACCENT_RED   = "#ff5555"
ACCENT_ORANGE = "#ff9955"
ACCENT_YELLOW = "#ffcc55"
ACCENT_CYAN   = "#55cccc"
ACCENT_MAG    = "#cc55cc"
BTN_GREEN     = "#2d5a2d"
BTN_GREEN_HOV = "#3d7a3d"
BTN_RED       = "#5a2d2d"
BTN_RED_HOV   = "#7a3d3d"

FIX_NAMES = {0: "NO FIX", 1: "DR ONLY", 2: "2D FIX", 3: "3D FIX",
             4: "GNSS+DR", 5: "TIME ONLY"}


# --- Auto-detect serial port --------------------------------------------

def find_serial_port() -> str | None:
    print("Scanning for serial ports...")
    for port, desc, _ in sorted(serial.tools.list_ports.comports()):
        if any(t in port for t in ("usbmodem", "usbserial", "SLAB_USBtoUART",
                                    "ttyACM", "ttyUSB", "COM")):
            print(f"  \u2713 Auto-selected: {port} ({desc})")
            return port
        print(f"  \u00b7 Skipped: {port} ({desc})")
    print("  \u2717 No compatible serial port found.")
    return None


SERIAL_PORT = find_serial_port()


# --- Global state -------------------------------------------------------

data_queue: queue.Queue = queue.Queue()
g_serial: serial.Serial | None = None
g_logging_active = False


def _build_db_frame(cmd_id: int, payload: bytes) -> bytes:
    msg_class = 0x00
    length = len(payload)
    header = struct.pack("<2sBBH", b"db", cmd_id, msg_class, length)
    cs = (cmd_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
          + sum(payload)) & 0xFFFF
    return header + payload + struct.pack("<H", cs)


def send_log_class(ser: serial.Serial, cls: int) -> None:
    ser.write(_build_db_frame(DB_CMD_LOG_CLASS, bytes([cls])))
    ser.flush()
    names = {LOG_CLASS_NONE: "NONE", LOG_CLASS_HEART_BEAT: "HEART_BEAT",
             LOG_CLASS_GPS: "GPS"}
    print(f"  \u2192 Log class: {names.get(cls, f'0x{cls:02X}')}")


def send_reset(ser: serial.Serial) -> None:
    ser.write(_build_db_frame(DB_CMD_RESET, bytes([0x00])))
    ser.flush()
    print("  \u2192 Reset command sent")


def serial_reader() -> None:
    global g_serial
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
        send_log_class(ser, LOG_CLASS_HEART_BEAT)

        while True:
            b1 = ser.read(1)
            if not b1:
                continue
            if b1[0] not in (0x62, 0x64):
                continue
            b2 = ser.read(1)
            if not b2:
                continue
            if not ((b1[0] == 0x64 and b2[0] == 0x62) or
                    (b1[0] == 0x62 and b2[0] == 0x64)):
                continue

            id_b = ser.read(1)
            cls_b = ser.read(1)
            len_b = ser.read(2)
            if len(id_b) < 1 or len(cls_b) < 1 or len(len_b) < 2:
                continue
            length = int.from_bytes(len_b, "little")
            if length > 1024:
                continue
            payload = ser.read(length)
            if len(payload) != length:
                continue
            _ck = ser.read(2)

            if id_b[0] == SEND_LOG_ID and length == GPS_FRAME_SIZE:
                vals = struct.unpack(GPS_STRUCT_FMT, payload)
                data_queue.put(vals)
    except Exception as exc:
        print(f"  \u2717 Serial error: {exc}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


# --- Helpers ------------------------------------------------------------

def latlon_to_local_m(lat_deg: float, lon_deg: float,
                      ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """Equirectangular projection — accurate within a few cm out to several km."""
    lat_r = math.radians(lat_deg)
    ref_r = math.radians(ref_lat)
    dlat = math.radians(lat_deg - ref_lat)
    dlon = math.radians(lon_deg - ref_lon)
    north_m = dlat * EARTH_R_M
    east_m  = dlon * EARTH_R_M * math.cos(0.5 * (lat_r + ref_r))
    return north_m, east_m


# --- Main UI ------------------------------------------------------------

def main() -> None:
    threading.Thread(target=serial_reader, daemon=True).start()

    plt.style.use("dark_background")
    plt.rcParams.update({
        "figure.facecolor": BG_COLOR,
        "axes.facecolor":   BG_COLOR,
        "axes.edgecolor":   GRID_COLOR,
        "axes.labelcolor":  TEXT_COLOR,
        "text.color":       TEXT_COLOR,
        "xtick.color":      DIM_TEXT,
        "ytick.color":      DIM_TEXT,
        "grid.color":       GRID_COLOR,
    })

    fig = plt.figure(figsize=(15, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle("GPS Dashboard — ZED-F9P (LOG_CLASS_GPS = 0x1E)",
                 fontsize=15, color=TEXT_COLOR, fontweight="bold", y=0.97)

    # ---- Layout (manual axes, matching the rest of the toolset) ----
    ax_map     = fig.add_axes([0.05, 0.36, 0.42, 0.55])
    ax_alt     = fig.add_axes([0.50, 0.62, 0.07, 0.29])
    ax_vel     = fig.add_axes([0.62, 0.62, 0.36, 0.29])
    ax_sats    = fig.add_axes([0.62, 0.36, 0.36, 0.20])
    ax_info    = fig.add_axes([0.50, 0.06, 0.20, 0.27])
    ax_metrics = fig.add_axes([0.05, 0.06, 0.42, 0.22])
    ax_btn1    = fig.add_axes([0.74, 0.21, 0.11, 0.05])
    ax_btn2    = fig.add_axes([0.86, 0.21, 0.11, 0.05])
    ax_btn3    = fig.add_axes([0.74, 0.14, 0.11, 0.05])
    ax_btn4    = fig.add_axes([0.86, 0.14, 0.11, 0.05])
    ax_status  = fig.add_axes([0.74, 0.06, 0.23, 0.06])

    # ---- 2-D position map ----
    ax_map.set_title("Position Map (local NED, metres)",
                     color=DIM_TEXT, fontsize=11)
    ax_map.set_xlabel("East (m)")
    ax_map.set_ylabel("North (m)")
    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.grid(True, alpha=0.3)
    ax_map.axhline(y=0, color=GRID_COLOR, lw=1)
    ax_map.axvline(x=0, color=GRID_COLOR, lw=1)
    line_trail, = ax_map.plot([], [], color=ACCENT_BLUE, lw=1.0, alpha=0.7,
                              label="Trail")
    pt_home = ax_map.scatter([0], [0], c=ACCENT_GREEN, s=80, marker="*",
                             zorder=5, label="Home")
    pt_now = ax_map.scatter([0], [0], c=ACCENT_RED, s=60, zorder=6,
                             label="Now")
    arrow_vel = ax_map.annotate("", xy=(0, 0), xytext=(0, 0),
                                arrowprops=dict(arrowstyle="->",
                                                color=ACCENT_YELLOW, lw=2))
    ax_map.legend(loc="upper right", fontsize=8, facecolor=PANEL_COLOR,
                  edgecolor=GRID_COLOR)

    # ---- Altitude bar ----
    ax_alt.set_title("Altitude\n(MSL m)", color=DIM_TEXT, fontsize=10)
    ax_alt.set_xticks([])
    ax_alt.grid(True, axis="y", alpha=0.3)
    bar_alt = ax_alt.bar([0], [0], color=ACCENT_ORANGE, width=0.6)
    txt_alt = ax_alt.text(0, 0.5, "—", ha="center", va="center",
                          color=TEXT_COLOR, fontsize=11, fontfamily="monospace",
                          transform=ax_alt.transAxes)
    line_alt_min = ax_alt.axhline(y=0, color=ACCENT_BLUE, lw=0.8, ls="--",
                                  alpha=0.6)
    line_alt_max = ax_alt.axhline(y=0, color=ACCENT_RED, lw=0.8, ls="--",
                                  alpha=0.6)

    # ---- Velocity time series ----
    ax_vel.set_title("Velocity (m/s)", color=DIM_TEXT, fontsize=11)
    ax_vel.set_xlim(0, HISTORY_LEN)
    ax_vel.set_ylim(-3, 3)
    ax_vel.grid(True, alpha=0.3)
    ax_vel.axhline(y=0, color=GRID_COLOR, lw=1)
    line_vN, = ax_vel.plot([], [], color=ACCENT_RED,    lw=1.4, label="vN")
    line_vE, = ax_vel.plot([], [], color=ACCENT_GREEN,  lw=1.4, label="vE")
    line_vD, = ax_vel.plot([], [], color=ACCENT_BLUE,   lw=1.4, label="vD")
    line_gs, = ax_vel.plot([], [], color=ACCENT_YELLOW, lw=1.4, label="gnd")
    ax_vel.legend(loc="upper right", fontsize=8, facecolor=PANEL_COLOR,
                  edgecolor=GRID_COLOR, ncol=4)

    # ---- Sat-count time series ----
    ax_sats.set_title("Satellites used in fix", color=DIM_TEXT, fontsize=11)
    ax_sats.set_xlim(0, HISTORY_LEN)
    ax_sats.set_ylim(0, 30)
    ax_sats.grid(True, alpha=0.3)
    line_sats, = ax_sats.plot([], [], color=ACCENT_CYAN, lw=1.4)
    ax_sats.set_xlabel("Samples")
    ax_sats.set_ylabel("count")

    # ---- Info panel ----
    ax_info.set_facecolor(PANEL_COLOR)
    ax_info.set_xticks([])
    ax_info.set_yticks([])
    ax_info.set_title("Live Fix", color=DIM_TEXT, fontsize=11)
    for sp in ax_info.spines.values():
        sp.set_edgecolor(GRID_COLOR)
    info_text = ax_info.text(0.05, 0.95, "Waiting for GPS frames…",
                             fontsize=10, fontfamily="monospace",
                             color=TEXT_COLOR, va="top",
                             transform=ax_info.transAxes)

    # ---- Metrics panel (accuracy + heading) ----
    ax_metrics.set_facecolor(PANEL_COLOR)
    ax_metrics.set_xticks([])
    ax_metrics.set_yticks([])
    ax_metrics.set_title("Accuracy & Heading", color=DIM_TEXT, fontsize=11)
    for sp in ax_metrics.spines.values():
        sp.set_edgecolor(GRID_COLOR)
    metrics_text = ax_metrics.text(0.03, 0.92, "—", fontsize=11,
                                   fontfamily="monospace", color=TEXT_COLOR,
                                   va="top", transform=ax_metrics.transAxes)

    # ---- Status bar ----
    ax_status.set_facecolor(PANEL_COLOR)
    ax_status.set_xticks([])
    ax_status.set_yticks([])
    for sp in ax_status.spines.values():
        sp.set_edgecolor(GRID_COLOR)
    status_text = ax_status.text(0.5, 0.5, "Idle", ha="center", va="center",
                                 color=DIM_TEXT, fontsize=10,
                                 fontfamily="monospace",
                                 transform=ax_status.transAxes)

    # ---- Buttons ----
    btn_start = Button(ax_btn1, "Start Log", color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_stop  = Button(ax_btn2, "Stop Log",  color=BTN_RED,   hovercolor=BTN_RED_HOV)
    btn_reset = Button(ax_btn3, "Reset FC",  color=BTN_RED,   hovercolor=BTN_RED_HOV)
    btn_clear = Button(ax_btn4, "Clear",     color="#3c3c3c", hovercolor="#555555")

    # ---- Buffers ----
    hist_n = np.zeros(HISTORY_LEN)
    hist_e = np.zeros(HISTORY_LEN)
    hist_vN = np.zeros(HISTORY_LEN)
    hist_vE = np.zeros(HISTORY_LEN)
    hist_vD = np.zeros(HISTORY_LEN)
    hist_gs = np.zeros(HISTORY_LEN)
    hist_sat = np.zeros(HISTORY_LEN)
    x_axis = np.arange(HISTORY_LEN)
    samples = [0]
    home = [None]    # [lat, lon] of first reliable fix
    alt_min = [None]
    alt_max = [None]
    last_frame_time = [time.time()]

    def reset_buffers() -> None:
        hist_n[:]   = 0
        hist_e[:]   = 0
        hist_vN[:]  = 0
        hist_vE[:]  = 0
        hist_vD[:]  = 0
        hist_gs[:]  = 0
        hist_sat[:] = 0
        samples[0]  = 0
        home[0]     = None
        alt_min[0]  = None
        alt_max[0]  = None

    def shift_in(buf: np.ndarray, value: float) -> None:
        buf[:-1] = buf[1:]
        buf[-1]  = value

    def update(_frame):
        drained = False
        last = None
        while not data_queue.empty():
            try:
                vals = data_queue.get_nowait()
            except queue.Empty:
                break
            drained = True
            last = vals
            (lat_deg, lon_deg, alt, vN, vE, vD, gs, head,
             h_acc, v_acc, p_dop_raw, num_sv, fix_type, flags, reliable, _pad) = vals

            # Lock the home position on the first reliable 3-D fix.
            if home[0] is None and reliable:
                home[0] = (lat_deg, lon_deg)

            if home[0] is not None:
                north_m, east_m = latlon_to_local_m(lat_deg, lon_deg,
                                                    home[0][0], home[0][1])
            else:
                north_m, east_m = 0.0, 0.0

            shift_in(hist_n,   north_m)
            shift_in(hist_e,   east_m)
            shift_in(hist_vN,  vN)
            shift_in(hist_vE,  vE)
            shift_in(hist_vD,  vD)
            shift_in(hist_gs,  gs)
            shift_in(hist_sat, num_sv)

            samples[0] += 1
            alt_min[0] = alt if alt_min[0] is None else min(alt_min[0], alt)
            alt_max[0] = alt if alt_max[0] is None else max(alt_max[0], alt)
            last_frame_time[0] = time.time()

        if drained and last is not None:
            (lat_deg, lon_deg, alt, vN, vE, vD, gs, head,
             h_acc, v_acc, p_dop_raw, num_sv, fix_type, flags, reliable, _pad) = last
            p_dop = p_dop_raw * 0.01

            n_valid = min(samples[0], HISTORY_LEN)
            if n_valid > 0:
                line_trail.set_data(hist_e[-n_valid:], hist_n[-n_valid:])
                pt_now.set_offsets([[hist_e[-1], hist_n[-1]]])

                # Velocity arrow on the map (2 s look-ahead)
                arrow_vel.xy = (hist_e[-1] + vE * 2.0,
                                hist_n[-1] + vN * 2.0)
                arrow_vel.set_position((hist_e[-1], hist_n[-1]))

                # Auto-fit map (square, with a small margin)
                xs = hist_e[-n_valid:]
                ys = hist_n[-n_valid:]
                cx = 0.5 * (xs.min() + xs.max())
                cy = 0.5 * (ys.min() + ys.max())
                half = max(2.0,
                           0.6 * max(xs.max() - xs.min(), ys.max() - ys.min()))
                ax_map.set_xlim(cx - half, cx + half)
                ax_map.set_ylim(cy - half, cy + half)

                # Velocity series
                line_vN.set_data(x_axis, hist_vN)
                line_vE.set_data(x_axis, hist_vE)
                line_vD.set_data(x_axis, hist_vD)
                line_gs.set_data(x_axis, hist_gs)
                v_max = max(2.0,
                            float(np.max(np.abs(np.stack([hist_vN, hist_vE,
                                                          hist_vD, hist_gs])))))
                ax_vel.set_ylim(-v_max * 1.2, v_max * 1.2)

                # Sats
                line_sats.set_data(x_axis, hist_sat)
                ax_sats.set_ylim(0, max(15, float(hist_sat.max()) + 2))

            # Altitude bar
            bar_alt[0].set_height(alt)
            txt_alt.set_text(f"{alt:7.2f} m")
            if alt_min[0] is not None and alt_max[0] is not None:
                lo = min(alt_min[0], alt) - 1.0
                hi = max(alt_max[0], alt) + 1.0
                if hi - lo < 4.0:
                    mid = 0.5 * (lo + hi)
                    lo, hi = mid - 2.0, mid + 2.0
                ax_alt.set_ylim(lo, hi)
                line_alt_min.set_ydata([alt_min[0]])
                line_alt_max.set_ydata([alt_max[0]])

            # Info text
            fix_name = FIX_NAMES.get(int(fix_type), f"?{int(fix_type)}")
            rel_marker = "RELIABLE" if reliable else "unreliable"
            info_text.set_text(
                f"Fix:    {fix_name}\n"
                f"Sats:   {int(num_sv):d}\n"
                f"Status: {rel_marker}\n"
                f"Flags:  0x{int(flags):02X}\n"
                f"\n"
                f"Lat:    {lat_deg:11.7f}°\n"
                f"Lon:    {lon_deg:11.7f}°\n"
                f"Alt:    {alt:8.2f} m\n"
            )
            metrics_text.set_text(
                f"H acc: {h_acc:6.2f} m       Heading: {head:6.1f}°\n"
                f"V acc: {v_acc:6.2f} m       pDOP:    {p_dop:6.2f}\n"
                f"\n"
                f"vN: {vN:+6.2f}   vE: {vE:+6.2f}   vD: {vD:+6.2f}  m/s\n"
                f"Ground speed:           {gs:6.2f}  m/s\n"
                f"\n"
                f"Samples: {samples[0]:5d}      "
                f"Alt range: [{(alt_min[0] or 0):.2f}, {(alt_max[0] or 0):.2f}] m"
            )

        # Status (refreshes even when no new frames arrive)
        age = time.time() - last_frame_time[0]
        if not g_logging_active:
            status_text.set_text("Logging stopped — press Start Log")
            status_text.set_color(DIM_TEXT)
        elif samples[0] == 0:
            status_text.set_text("Waiting for first GPS frame…")
            status_text.set_color(ACCENT_YELLOW)
        elif age > 2.0:
            status_text.set_text(f"No frame for {age:4.1f}s — check antenna / wiring")
            status_text.set_color(ACCENT_RED)
        else:
            status_text.set_text(f"Streaming  ({samples[0]} samples)")
            status_text.set_color(ACCENT_GREEN)

        return (line_trail, pt_now, arrow_vel, line_vN, line_vE, line_vD,
                line_gs, line_sats, bar_alt[0], info_text, metrics_text,
                status_text)

    # ---- Button callbacks ----
    def on_start(_event) -> None:
        global g_logging_active
        if g_serial and g_serial.is_open:
            send_log_class(g_serial, LOG_CLASS_GPS)
            g_logging_active = True

    def on_stop(_event) -> None:
        global g_logging_active
        if g_serial and g_serial.is_open:
            send_log_class(g_serial, LOG_CLASS_NONE)
            g_logging_active = False

    def on_reset(_event) -> None:
        if g_serial and g_serial.is_open:
            send_reset(g_serial)

            def _post_reset() -> None:
                time.sleep(2.0)
                if g_serial and g_serial.is_open:
                    g_serial.reset_input_buffer()
                    send_log_class(g_serial, LOG_CLASS_HEART_BEAT)

            threading.Thread(target=_post_reset, daemon=True).start()

    def on_clear(_event) -> None:
        reset_buffers()

    btn_start.on_clicked(on_start)
    btn_stop.on_clicked(on_stop)
    btn_reset.on_clicked(on_reset)
    btn_clear.on_clicked(on_clear)

    _ani = FuncAnimation(fig, update, interval=80, blit=False,
                         cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
