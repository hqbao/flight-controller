"""
ESKF Matrix Viewer
==================

Live heatmap of one of the four 15-state ESKF matrices: P, F, K, H. Only
ONE matrix streams at a time — pick with the buttons at the bottom. The FC
emits the selected matrix row-by-row at ~1 Hz; this tool reassembles each
snapshot and renders it with a diverging colormap centered at zero.

  P (15×15)   covariance, always valid post-init
  F (15×15)   last predict-step error-state Jacobian
  K (15×m)    last update Kalman gain   (m = 1, 2 or 3)
  H (m×15)    last update measurement Jacobian (m = 1, 2 or 3)

Per-row wire payload (12 B header + 4·cols B data, max 12+60 = 72 B):
  uint8  matrix_id     0=P, 1=F, 2=K, 3=H
  uint8  rows
  uint8  cols
  uint8  row_idx
  uint8  update_type   fusion6_update_id_t (K/H only); 0xFF for P/F
  uint8  pad
  uint16 seq           LE — same value for every row of one snapshot
  uint32 t_ms          LE
  float32 row[cols]    LE, row-major

Side panel shows the matrix's update-type tag, min / max / Frobenius norm,
seq counter, and a rolling Frobenius-norm strip at the bottom.
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
from matplotlib.colors import SymLogNorm, Normalize  # noqa: E402
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

LOG_CLASS_NONE       = 0x00
LOG_CLASS_HEART_BEAT = 0x09
LOG_CLASS_ESKF_P     = 0x24
LOG_CLASS_ESKF_F     = 0x25
LOG_CLASS_ESKF_K     = 0x26
LOG_CLASS_ESKF_H     = 0x27

MATRIX_NAMES   = {0: "P", 1: "F", 2: "K", 3: "H"}
MATRIX_LOG_CLS = {
    "P": LOG_CLASS_ESKF_P,
    "F": LOG_CLASS_ESKF_F,
    "K": LOG_CLASS_ESKF_K,
    "H": LOG_CLASS_ESKF_H,
}

UPDATE_TYPE_NAMES = {
    0:    "ACCEL",
    1:    "MAG",
    2:    "VEL_XY_BODY",
    3:    "BARO",
    4:    "GPS_POS",
    5:    "GPS_VEL",
    0xFF: "—",
}

# 15-state ESKF index labels (matches fusion6 layout).
STATE_LABELS = [
    "δp.x",  "δp.y",  "δp.z",
    "δv.x",  "δv.y",  "δv.z",
    "δθ.x",  "δθ.y",  "δθ.z",
    "δb_a.x","δb_a.y","δb_a.z",
    "δb_g.x","δb_g.y","δb_g.z",
]


# --- UI palette ---
BG_COLOR     = "#1e1e1e"
PANEL_COLOR  = "#252526"
TEXT_COLOR   = "#cccccc"
DIM_TEXT     = "#888888"
GRID_COLOR   = "#3c3c3c"
ACCENT_BLUE  = "#5599ff"
BTN_GREEN    = "#2d5a2d"
BTN_GREEN_HOV= "#3d7a3d"
BTN_RED      = "#5a2d2d"
BTN_RED_HOV  = "#7a3d3d"
BTN_BLUE     = "#2d4a6a"
BTN_BLUE_HOV = "#3d6a8a"


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
g_serial = None
g_chip_id = None
g_active_matrix = "P"          # one of "P","F","K","H"; updated by buttons
g_logging_active = False
g_rx_frame_count = 0
g_rx_snapshot_count = 0

# Reassembled snapshots (one per matrix label). Each entry:
#   { "rows": int, "cols": int, "update_type": int, "seq": int,
#     "t_ms": int, "row_idx": int, "data": np.ndarray (rows, cols) }
# Buffers are persistent: rows arrive one-per-frame at ~25 Hz and overwrite
# the previous snapshot in place. A new snapshot is signalled by row_idx=0
# (not by the seq counter), so partial snapshots render immediately and a
# dropped row leaves its previous value visible until the next pass.
g_latest = {k: None for k in MATRIX_NAMES.values()}
g_latest_lock = threading.Lock()

# Per-matrix in-progress assembly buffer (keyed by name). Persistent across
# snapshots — see g_latest comment above.
_assembly = {k: {"seq": None, "rows": 0, "cols": 0, "update_type": 0xFF,
                 "t_ms": 0, "data": None} for k in MATRIX_NAMES.values()}


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


def _parse_eskf_row(payload):
    """Decode one row-frame; update the persistent per-matrix buffer and
    push it to g_latest immediately so the heatmap reflects every arrived
    row — partial snapshots still render, and a dropped row just leaves
    the previous value in place until the next pass overwrites it."""
    global g_rx_snapshot_count
    if len(payload) < 12:
        return
    matrix_id, rows, cols, row_idx, update_type, _pad = struct.unpack_from("<6B", payload, 0)
    seq, t_ms = struct.unpack_from("<HI", payload, 6)
    if matrix_id not in MATRIX_NAMES:
        return
    name = MATRIX_NAMES[matrix_id]
    expected_data_bytes = cols * 4
    if len(payload) < 12 + expected_data_bytes:
        return
    if rows == 0 or cols == 0 or rows > 15 or cols > 15 or row_idx >= rows:
        return

    row_vals = struct.unpack_from(f"<{cols}f", payload, 12)

    a = _assembly[name]
    # Re-allocate when the matrix shape changes (e.g. K toggling between
    # m=1, 2, 3 cols as the active update type changes).
    if a["data"] is None or a["rows"] != rows or a["cols"] != cols:
        a["rows"] = rows
        a["cols"] = cols
        a["data"] = np.zeros((rows, cols), dtype=np.float32)

    a["data"][row_idx, :] = row_vals
    a["seq"] = seq
    a["t_ms"] = t_ms
    a["update_type"] = update_type

    snap = {
        "rows": rows,
        "cols": cols,
        "update_type": update_type,
        "seq": seq,
        "t_ms": t_ms,
        "row_idx": row_idx,
        "data": a["data"].copy(),
    }
    with g_latest_lock:
        g_latest[name] = snap
    # Count a "snapshot" each time the FC starts a new pass (row 0).
    if row_idx == 0:
        g_rx_snapshot_count += 1


def serial_reader():
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
                elif msg_id == SEND_LOG_ID and length >= 12:
                    g_rx_frame_count += 1
                    _parse_eskf_row(payload)
    except Exception as e:
        print(f"Serial error: {e}")
    finally:
        if g_serial and g_serial.is_open:
            g_serial.close()


def _switch_matrix(name):
    """Stop the previous stream, start the new one."""
    global g_active_matrix, g_logging_active
    if not (g_serial and g_serial.is_open):
        g_active_matrix = name
        return
    if g_logging_active:
        send_log_class(g_serial, LOG_CLASS_NONE)
    send_log_class(g_serial, MATRIX_LOG_CLS[name])
    g_active_matrix = name
    g_logging_active = True


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

    fig = plt.figure(figsize=screen_fit_figsize(13, 9))
    fig.patch.set_facecolor(BG_COLOR)
    fig.canvas.manager.set_window_title("ESKF Matrix Viewer")
    fig.suptitle("ESKF Matrix Viewer — fusion6 internals (read-only)",
                 fontsize=14, color=TEXT_COLOR, fontweight="bold", y=0.99)

    # Layout: heatmap (left), info panel (right), Frobenius strip (bottom).
    gs = fig.add_gridspec(2, 2, left=0.07, right=0.97, top=0.93, bottom=0.18,
                          hspace=0.35, wspace=0.28,
                          height_ratios=[5, 1], width_ratios=[3, 1])

    ax_heat = fig.add_subplot(gs[0, 0])
    ax_info = fig.add_subplot(gs[0, 1])
    ax_norm = fig.add_subplot(gs[1, :])

    ax_info.axis("off")

    # Initial heatmap: 15×15 zeros so the colorbar/colormap is set up early.
    init = np.zeros((15, 15), dtype=np.float32)
    im = ax_heat.imshow(init, cmap="RdBu_r",
                        norm=Normalize(vmin=-1.0, vmax=1.0),
                        interpolation="nearest", aspect="auto")
    ax_heat.set_title("P (15×15)", color=TEXT_COLOR, fontsize=11, pad=8)
    cbar = fig.colorbar(im, ax=ax_heat, fraction=0.04, pad=0.02)
    cbar.ax.tick_params(colors=DIM_TEXT, labelsize=8)
    cbar.outline.set_edgecolor(GRID_COLOR)

    # Norm strip
    ax_norm.set_facecolor(BG_COLOR)
    ax_norm.grid(True, color=GRID_COLOR, alpha=0.4, linewidth=0.5)
    for sp in ax_norm.spines.values():
        sp.set_color(GRID_COLOR)
    ax_norm.set_ylabel("‖M‖_F", color=TEXT_COLOR, fontsize=9)
    ax_norm.set_xlabel("snapshot #", color=TEXT_COLOR, fontsize=9)
    norm_line, = ax_norm.plot([], [], color=ACCENT_BLUE, lw=1.4)

    # Side info panel text
    info_text = ax_info.text(
        0.0, 0.98, "", transform=ax_info.transAxes,
        ha="left", va="top", family="monospace", fontsize=10,
        color=TEXT_COLOR,
    )

    # Bottom info row
    chip_text = fig.text(0.02, 0.06, "", fontsize=8, ha="left",
                         color=DIM_TEXT)
    fps_text = fig.text(0.50, 0.06, "", fontsize=8, ha="center",
                        color=DIM_TEXT)
    status_text = fig.text(0.50, 0.10, "", fontsize=9, ha="center",
                           color=TEXT_COLOR, family="monospace")

    # Norm history per matrix label.
    norm_hist = {k: deque(maxlen=120) for k in MATRIX_NAMES.values()}
    last_seq = {k: None for k in MATRIX_NAMES.values()}
    last_rx_count = [0]
    last_snap_count = [0]
    last_fps_t = [time.time()]
    log_scale = [False]

    # --- Buttons --- (P, F, K, H, Log/Lin, Reset)
    def _make_btn(rect, label, color, hover):
        ax = fig.add_axes(rect)
        b = Button(ax, label, color=color, hovercolor=hover)
        b.label.set_color(TEXT_COLOR)
        b.label.set_fontsize(9)
        return ax, b

    ax_p, btn_p = _make_btn([0.07, 0.02, 0.07, 0.04], "P", BTN_BLUE, BTN_BLUE_HOV)
    ax_f, btn_f = _make_btn([0.15, 0.02, 0.07, 0.04], "F", BTN_GREEN, BTN_GREEN_HOV)
    ax_k, btn_k = _make_btn([0.23, 0.02, 0.07, 0.04], "K", BTN_GREEN, BTN_GREEN_HOV)
    ax_h, btn_h = _make_btn([0.31, 0.02, 0.07, 0.04], "H", BTN_GREEN, BTN_GREEN_HOV)

    ax_scale, btn_scale = _make_btn([0.55, 0.02, 0.10, 0.04], "Log scale",
                                     BTN_GREEN, BTN_GREEN_HOV)
    ax_stop, btn_stop  = _make_btn([0.66, 0.02, 0.10, 0.04], "Stop",
                                    BTN_RED, BTN_RED_HOV)
    ax_reset, btn_reset = _make_btn([0.87, 0.02, 0.10, 0.04], "Reset FC",
                                     BTN_RED, BTN_RED_HOV)

    pick_axes = {"P": ax_p, "F": ax_f, "K": ax_k, "H": ax_h}
    pick_btns = {"P": btn_p, "F": btn_f, "K": btn_k, "H": btn_h}

    def _refresh_picker_colors():
        for name, ax in pick_axes.items():
            sel = (name == g_active_matrix and g_logging_active)
            color = BTN_BLUE if sel else BTN_GREEN
            hover = BTN_BLUE_HOV if sel else BTN_GREEN_HOV
            pick_btns[name].color = color
            pick_btns[name].hovercolor = hover
            ax.set_facecolor(color)

    def _on_pick(name):
        def _f(_evt):
            _switch_matrix(name)
            _refresh_picker_colors()
        return _f

    btn_p.on_clicked(_on_pick("P"))
    btn_f.on_clicked(_on_pick("F"))
    btn_k.on_clicked(_on_pick("K"))
    btn_h.on_clicked(_on_pick("H"))

    def on_scale(_evt):
        log_scale[0] = not log_scale[0]
        btn_scale.label.set_text("Linear" if log_scale[0] else "Log scale")

    btn_scale.on_clicked(on_scale)

    def on_stop(_evt):
        global g_logging_active
        if g_serial and g_serial.is_open and g_logging_active:
            send_log_class(g_serial, LOG_CLASS_NONE)
            g_logging_active = False
            _refresh_picker_colors()

    btn_stop.on_clicked(on_stop)

    def on_reset(_evt):
        global g_logging_active
        if not (g_serial and g_serial.is_open):
            return
        send_reset(g_serial)
        g_logging_active = False
        for k in norm_hist:
            norm_hist[k].clear()
        for k in last_seq:
            last_seq[k] = None
        _refresh_picker_colors()

        def _after():
            time.sleep(2.0)
            if g_serial and g_serial.is_open:
                g_serial.reset_input_buffer()
                send_log_class(g_serial, LOG_CLASS_HEART_BEAT)
        threading.Thread(target=_after, daemon=True).start()

    btn_reset.on_clicked(on_reset)

    _refresh_picker_colors()

    def update(_):
        # FPS / chip-id text
        now = time.time()
        if g_chip_id and not chip_text.get_text():
            chip_text.set_text(f"Chip {g_chip_id}")
        if now - last_fps_t[0] >= 1.0:
            elapsed = now - last_fps_t[0]
            rx_hz = (g_rx_frame_count - last_rx_count[0]) / elapsed
            sn_hz = (g_rx_snapshot_count - last_snap_count[0]) / elapsed
            fps_text.set_text(f"RX {rx_hz:.0f} frame/s | {sn_hz:.1f} snap/s")
            last_rx_count[0] = g_rx_frame_count
            last_snap_count[0] = g_rx_snapshot_count
            last_fps_t[0] = now

        active = g_active_matrix
        with g_latest_lock:
            snap = g_latest.get(active)

        if snap is None:
            status_text.set_text(f"Selected: {active} — no snapshot yet "
                                 f"({'streaming' if g_logging_active else 'stopped'})")
            return ()

        # New row arrived? Update the heatmap. Key on (seq, row_idx) so
        # every paged row triggers a redraw, not just snapshot completion.
        snap_id = (snap["seq"], snap.get("row_idx", -1))
        if last_seq[active] != snap_id:
            last_seq[active] = snap_id
            data = snap["data"]
            rows, cols = data.shape

            # Color limits: symmetric around 0 using the largest |entry|.
            mx = float(np.max(np.abs(data)))
            if mx == 0.0 or not math.isfinite(mx):
                mx = 1.0

            if log_scale[0]:
                # Linear within ±lin_thresh, log outside.
                lin_thresh = max(mx * 1e-4, 1e-12)
                im.set_norm(SymLogNorm(linthresh=lin_thresh,
                                        vmin=-mx, vmax=mx, base=10))
            else:
                im.set_norm(Normalize(vmin=-mx, vmax=mx))

            im.set_data(data)
            # imshow's extent must reflect the actual matrix shape so the
            # axis ticks line up.
            im.set_extent((-0.5, cols - 0.5, rows - 0.5, -0.5))
            ax_heat.set_xlim(-0.5, cols - 0.5)
            ax_heat.set_ylim(rows - 0.5, -0.5)
            ax_heat.set_xticks(range(cols))
            ax_heat.set_yticks(range(rows))
            # Label axes with state names where applicable.
            if cols == 15:
                ax_heat.set_xticklabels(STATE_LABELS, rotation=60, fontsize=7)
            else:
                ax_heat.set_xticklabels([f"m{c}" for c in range(cols)], fontsize=8)
            if rows == 15:
                ax_heat.set_yticklabels(STATE_LABELS, fontsize=7)
            else:
                ax_heat.set_yticklabels([f"m{r}" for r in range(rows)], fontsize=8)

            ax_heat.set_title(f"{active} ({rows}×{cols})",
                              color=TEXT_COLOR, fontsize=11, pad=8)

            # Side info
            ut = UPDATE_TYPE_NAMES.get(snap["update_type"], f"?{snap['update_type']}")
            mn = float(np.min(data))
            mx_signed = float(np.max(data))
            fnorm = float(np.linalg.norm(data))
            info_lines = [
                f"matrix      : {active}",
                f"shape       : {rows} × {cols}",
                f"update_type : {ut}",
                f"seq         : {snap['seq']}",
                f"t_ms (FC)   : {snap['t_ms']}",
                f"min         : {mn:+.4g}",
                f"max         : {mx_signed:+.4g}",
                f"|M|_F       : {fnorm:.4g}",
                f"scale       : {'log' if log_scale[0] else 'linear'}",
            ]
            info_text.set_text("\n".join(info_lines))

            # Norm strip — append for the active matrix only.
            norm_hist[active].append(fnorm)

            ys = list(norm_hist[active])
            xs = list(range(len(ys)))
            norm_line.set_data(xs, ys)
            ax_norm.relim()
            ax_norm.autoscale_view(scaley=True)
            ax_norm.set_xlim(0, max(10, len(ys)))

            status_text.set_text(f"Selected: {active} — streaming "
                                 f"(seq {snap['seq']})")

        return ()

    ani = FuncAnimation(fig, update, interval=200, blit=False,
                        cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
