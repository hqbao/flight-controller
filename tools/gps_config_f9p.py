"""
gps_config_f9p.py - UI configurator & live monitor for u-blox ZED-F9P.

Connects directly to the F9P over USB CDC (or its UART at any baud) and provides:

  Live monitor panels
    - UBX-NAV-PVT     fix type, sats, lat/lon/alt, velocity, heading, accuracy
    - UBX-MON-HW      antenna status, jamming/noise, AGC
    - UBX-NAV-SAT     per-satellite C/N0 bar chart
    - UBX-MON-VER     firmware / hardware identification banner

  Configuration controls (CFG-VALSET, all three layers RAM+BBR+Flash)
    - Measurement rate (1 / 5 / 10 / 20 Hz)
    - UART1 + UART2 baud (9600 / 38400 / 115200 / 230400 / 460800)
    - Dynamic platform model (portable / stationary / airborne <1g/<2g/<4g, ...)
    - GNSS constellations (GPS / Galileo / GLONASS / BeiDou / SBAS / QZSS)
    - Output ports for UBX-NAV-PVT (UART1 / UART2 / USB)
    - "Drone-clean output" toggle: UBX-only on every port, all NMEA silenced

  Action buttons
    - Apply Config        write the form to the receiver (RAM only)
    - Save to Flash       UBX-CFG-CFG persist to BBR + Flash
    - Read Version        poll UBX-MON-VER and refresh the banner
    - Hot / Cold Reset    UBX-CFG-RST (keep / wipe ephemerides)
    - Factory Default     UBX-CFG-CFG clear + reset

UI palette and layout match the rest of the flight-controller toolkit.

Usage:
  python3 tools/gps_config_f9p.py              # auto-detect serial port
  python3 tools/gps_config_f9p.py /dev/cu.usbmodem14201 --baud 38400
"""

import argparse
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
# --- inlined view helpers (no external dependency) ---
FACE_VIEWS = {
    'Top':    (90, 180),
    'Bottom': (-90, 0),
    'Front':  (0, 0),
    'Back':   (0, 180),
    'Left':   (0, 90),
    'Right':  (0, -90),
}

def fit_to_screen_height(fig, margin_px=80):
    """Resize ``fig`` so its height matches the host screen height
    (preserving the original ``figsize`` aspect ratio).  No-op if the
    screen size cannot be determined."""
    import sys as _sys
    sw = sh = 0
    try:
        if _sys.platform == 'darwin':
            import ctypes as _c, ctypes.util as _cu
            _lib = _c.CDLL(_cu.find_library('ApplicationServices'))
            class _Sz(_c.Structure):
                _fields_ = [('w', _c.c_double), ('h', _c.c_double)]
            class _Pt(_c.Structure):
                _fields_ = [('x', _c.c_double), ('y', _c.c_double)]
            class _Rc(_c.Structure):
                _fields_ = [('o', _Pt), ('s', _Sz)]
            _lib.CGMainDisplayID.restype = _c.c_uint32
            _lib.CGDisplayBounds.restype = _Rc
            _lib.CGDisplayBounds.argtypes = [_c.c_uint32]
            r = _lib.CGDisplayBounds(_lib.CGMainDisplayID())
            sw, sh = int(r.s.w), int(r.s.h)
        else:
            import tkinter as _tk
            _r = _tk.Tk(); _r.withdraw()
            sw = _r.winfo_screenwidth(); sh = _r.winfo_screenheight()
            _r.destroy()
    except Exception:
        return
    if sh <= 0 or sw <= 0:
        return
    th = max(400, sh - margin_px)
    wi, hi = fig.get_size_inches()
    if hi <= 0:
        return
    asp = wi / hi
    tw = int(th * asp)
    if tw > sw - margin_px:
        tw = max(400, sw - margin_px)
        th = int(tw / asp)
    # Use rc logical DPI (not fig.get_dpi(), which doubles on HiDPI/retina
    # and would halve the resulting window).
    import matplotlib as _mpl
    _ldpi = _mpl.rcParams.get("figure.dpi", 100) or 100
    try:
        fig.set_size_inches(tw / _ldpi, th / _ldpi, forward=True)
    except Exception:
        pass

def add_3d_view_buttons(fig, ax, default='Back',
                        rect=(0.02, 0.05, 0.06, 0.035), gap=0.005,
                        color='#2a3140', hover='#3a4255',
                        text_color='#cfd6e4', fontsize=7):
    """Add a row of 6 face-view buttons (Top, Bottom, Front, Back, Left,
    Right) to ``fig`` controlling the 3D ``ax``.  Sets ``default`` view."""
    from matplotlib.widgets import Button as _Btn
    x0, y0, w, h = rect
    btns = []
    for i, (label, (elev, azim)) in enumerate(FACE_VIEWS.items()):
        bx = fig.add_axes([x0 + i * (w + gap), y0, w, h])
        b = _Btn(bx, label, color=color, hovercolor=hover)
        b.label.set_color(text_color)
        b.label.set_fontsize(fontsize)
        b.on_clicked(
            lambda event, e=elev, a=azim:
            (ax.view_init(elev=e, azim=a), fig.canvas.draw_idle()))
        btns.append(b)
    if default in FACE_VIEWS:
        e, a = FACE_VIEWS[default]
        ax.view_init(elev=e, azim=a)
    return btns
# --- end inlined view helpers ---
from matplotlib.animation import FuncAnimation  # noqa: E402
from matplotlib.widgets import Button, RadioButtons, CheckButtons, TextBox  # noqa: E402


# ============================================================================
# UI palette (matches every other tool)
# ============================================================================

BG_COLOR     = "#1e1e1e"
PANEL_COLOR  = "#252526"
TEXT_COLOR   = "#cccccc"
DIM_TEXT     = "#888888"
GRID_COLOR   = "#3c3c3c"
ACCENT_BLUE   = "#5599ff"
ACCENT_GREEN  = "#55cc55"
ACCENT_RED    = "#ff5555"
ACCENT_ORANGE = "#ff9955"
ACCENT_YELLOW = "#ffcc55"
ACCENT_CYAN   = "#55cccc"
ACCENT_MAG    = "#cc55cc"
BTN_GREEN     = "#2d5a2d"
BTN_GREEN_HOV = "#3d7a3d"
BTN_RED       = "#5a2d2d"
BTN_RED_HOV   = "#7a3d3d"
BTN_BLUE      = "#2d3a5a"
BTN_BLUE_HOV  = "#3d4a7a"
BTN_GREY      = "#3c3c3c"
BTN_GREY_HOV  = "#555555"


# ============================================================================
# UBX wire helpers
# ============================================================================

UBX_SYNC = b"\xb5\x62"

CLASS_NAV = 0x01
CLASS_CFG = 0x06
CLASS_MON = 0x0A
CLASS_ACK = 0x05

ID_NAV_PVT  = 0x07
ID_NAV_SAT  = 0x35
ID_MON_VER  = 0x04
ID_MON_HW   = 0x09
ID_CFG_VALSET = 0x8A
ID_CFG_VALGET = 0x8B
ID_CFG_CFG    = 0x09
ID_CFG_RST    = 0x04
ID_ACK_ACK = 0x01
ID_ACK_NAK = 0x00


def ubx_checksum(payload: bytes) -> bytes:
    ck_a = ck_b = 0
    for b in payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])


def ubx_frame(msg_class: int, msg_id: int, payload: bytes = b"") -> bytes:
    body = bytes([msg_class, msg_id]) + struct.pack("<H", len(payload)) + payload
    return UBX_SYNC + body + ubx_checksum(body)


# ============================================================================
# CFG-VALSET key encodings
# Storage type encoded in bits 28..30:
#   1=1-bit (1 byte 0/1), 2=U1/E1, 3=U2, 4=U4, 5=U8
# ============================================================================

SIZE_FOR_TYPE = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}


def key_size(key: int) -> int:
    return SIZE_FOR_TYPE[(key >> 28) & 0x07]


def cfg_valset(items, layers: int = 0x07) -> bytes:
    """layers bitmask: 0x01=RAM, 0x02=BBR, 0x04=Flash. 0x07 = all three."""
    payload = struct.pack("<BBH", 0x00, layers, 0x0000)
    for key, value in items:
        size = key_size(key)
        payload += struct.pack("<I", key)
        if   size == 1: payload += struct.pack("<B", int(value) & 0xFF)
        elif size == 2: payload += struct.pack("<H", int(value) & 0xFFFF)
        elif size == 4: payload += struct.pack("<I", int(value) & 0xFFFFFFFF)
        elif size == 8: payload += struct.pack("<Q", int(value) & 0xFFFFFFFFFFFFFFFF)
    return ubx_frame(CLASS_CFG, ID_CFG_VALSET, payload)


def cfg_save_all() -> bytes:
    return ubx_frame(CLASS_CFG, ID_CFG_CFG,
                     struct.pack("<IIIB", 0x00000000, 0xFFFFFFFF, 0x00000000, 0b00000011))


def cfg_factory_default() -> bytes:
    # Clear + Save resets RAM/BBR/Flash to factory defaults on next boot.
    return ubx_frame(CLASS_CFG, ID_CFG_CFG,
                     struct.pack("<IIIB", 0xFFFFFFFF, 0x00000000, 0xFFFFFFFF, 0b00010111))


def cfg_reset(navBbrMask: int, resetMode: int) -> bytes:
    return ubx_frame(CLASS_CFG, ID_CFG_RST,
                     struct.pack("<HBB", navBbrMask, resetMode, 0x00))


# Configuration keys we actually use ----------------------------------------

# Rate
K_RATE_MEAS    = 0x30210001  # U2  ms between measurements
K_RATE_NAV     = 0x30210002  # U2  measurements per nav solution
K_RATE_TIMEREF = 0x20210003  # E1

# UART baud
K_UART1_BAUD = 0x40520001
K_UART2_BAUD = 0x40530001

# Protocol enables (per direction, per port)
K_UART1_IN_UBX,  K_UART1_IN_NMEA,  K_UART1_IN_RTCM  = 0x10730001, 0x10730002, 0x10730004
K_UART1_OUT_UBX, K_UART1_OUT_NMEA, K_UART1_OUT_RTCM = 0x10740001, 0x10740002, 0x10740004
K_UART2_IN_UBX,  K_UART2_IN_NMEA,  K_UART2_IN_RTCM  = 0x10750001, 0x10750002, 0x10750004
K_UART2_OUT_UBX, K_UART2_OUT_NMEA, K_UART2_OUT_RTCM = 0x10760001, 0x10760002, 0x10760004
K_USB_OUT_UBX,   K_USB_OUT_NMEA                     = 0x10780001, 0x10780002

# UBX-NAV-PVT MSGOUT (per port)
K_MSGOUT_NAV_PVT_UART1 = 0x20910007
K_MSGOUT_NAV_PVT_UART2 = 0x20910008
K_MSGOUT_NAV_PVT_USB   = 0x20910009

# Other UBX-NAV msgs we silence
NAV_SILENCE_KEYS = {
    "POSLLH": (0x2091002A, 0x2091002B, 0x2091002C),
    "VELNED": (0x20910044, 0x20910045, 0x20910046),
    "STATUS": (0x2091001B, 0x2091001C, 0x2091001D),
    "DOP":    (0x20910039, 0x2091003A, 0x2091003B),
    "SAT":    (0x20910016, 0x20910017, 0x20910018),
}

# Standard NMEA messages per port
NMEA_KEYS = {
    "GGA": (0x209100BB, 0x209100BC, 0x209100BD),
    "GLL": (0x209100CA, 0x209100CB, 0x209100CC),
    "GSA": (0x209100C0, 0x209100C1, 0x209100C2),
    "GSV": (0x209100C5, 0x209100C6, 0x209100C7),
    "RMC": (0x209100AC, 0x209100AD, 0x209100AE),
    "VTG": (0x209100B1, 0x209100B2, 0x209100B3),
}

# Dynamic platform model
K_NAVSPG_DYNMODEL = 0x20110021  # E1
DYNMODEL_CHOICES = [
    ("Portable",       0),
    ("Stationary",     2),
    ("Pedestrian",     3),
    ("Automotive",     4),
    ("Sea",            5),
    ("Airborne <1g",   6),
    ("Airborne <2g",   7),
    ("Airborne <4g",   8),
]

# GNSS constellation enables (CFG-SIGNAL, 1-bit)
K_SIG_GPS  = 0x1031001F
K_SIG_SBAS = 0x10310020
K_SIG_GAL  = 0x10310021
K_SIG_BDS  = 0x10310022
K_SIG_QZSS = 0x10310024
K_SIG_GLO  = 0x10310025

GNSS_LIST = [
    ("GPS",     K_SIG_GPS,  True),
    ("Galileo", K_SIG_GAL,  True),
    ("GLONASS", K_SIG_GLO,  True),
    ("BeiDou",  K_SIG_BDS,  True),
    ("SBAS",    K_SIG_SBAS, False),
    ("QZSS",    K_SIG_QZSS, False),
]

# Selectable rate / baud / dyn-model labels for the radio buttons
RATE_CHOICES = [("1 Hz", 1000), ("5 Hz", 200), ("10 Hz", 100), ("20 Hz", 50)]
BAUD_CHOICES = [("9600", 9600), ("38400", 38400), ("115200", 115200),
                ("230400", 230400), ("460800", 460800)]

# Earth radius for the lat/lon → metres approximation around the home fix.
EARTH_R_M = 6_378_137.0

# History length for velocity / position trail (samples).
HISTORY_LEN = 600


def latlon_to_local_m(lat_deg: float, lon_deg: float,
                      ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """Equirectangular projection — accurate within a few cm out to several km."""
    lat_r = math.radians(lat_deg)
    ref_r = math.radians(ref_lat)
    dlat  = math.radians(lat_deg - ref_lat)
    dlon  = math.radians(lon_deg - ref_lon)
    north_m = dlat * EARTH_R_M
    east_m  = dlon * EARTH_R_M * math.cos(0.5 * (lat_r + ref_r))
    return north_m, east_m


# ============================================================================
# Live state
# ============================================================================

class LiveState:
    def __init__(self):
        self.lock = threading.Lock()
        # NAV-PVT
        self.have_pvt = False
        self.fix_type = 0
        self.flags    = 0
        self.num_sv   = 0
        self.lat      = 0.0
        self.lon      = 0.0
        self.alt_msl  = 0.0
        self.h_acc_m  = 0.0
        self.v_acc_m  = 0.0
        self.vN = self.vE = self.vD = 0.0
        self.gnd_speed = 0.0
        self.head_mot  = 0.0
        self.p_dop     = 0.0
        # MON-HW
        self.have_hw = False
        self.noise_per_ms = 0
        self.agc_cnt = 0
        self.ant_status = 0
        self.ant_power = 0
        self.jam_ind = 0
        self.jam_state = 0
        # NAV-SAT
        self.sats = []  # list of (gnssId, svId, cno, used)
        # MON-VER
        self.sw_version = ""
        self.hw_version = ""
        self.extensions = []
        # Bookkeeping
        self.last_pvt_t = 0.0
        self.last_hw_t  = 0.0
        self.frames_in  = 0


live = LiveState()
status_q: queue.Queue = queue.Queue()


def post_status(msg: str, level: str = "info") -> None:
    status_q.put((msg, level))


FIX_NAMES = {0: "NO FIX", 1: "DEAD RECK", 2: "2D FIX", 3: "3D FIX",
             4: "GNSS+DR", 5: "TIME ONLY"}
ANT_STATUS_NAMES = {0: "INIT", 1: "DON'T KNOW", 2: "OK", 3: "SHORT", 4: "OPEN"}
ANT_POWER_NAMES  = {0: "OFF", 1: "ON", 2: "DON'T KNOW"}
JAM_STATE_NAMES  = {0: "unknown", 1: "ok", 2: "warning", 3: "critical"}
GNSS_ID_NAMES    = {0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 4: "IMES",
                    5: "QZS", 6: "GLO", 7: "NavIC"}


# ============================================================================
# Serial transport
# ============================================================================

class UbxLink:
    def __init__(self):
        self.ser: serial.Serial | None = None
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        # FIFO of labels for the next ACK/NAK on CFG-VALSET (0x06/0x8A).
        self.valset_labels: queue.Queue = queue.Queue()

    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def open(self, port: str, baud: int) -> bool:
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(0.15)
            self.ser.reset_input_buffer()
        except Exception as exc:
            post_status(f"Connect failed: {exc}", "err")
            self.ser = None
            return False
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()
        post_status(f"Connected to {port} @ {baud}", "ok")
        # Kick off a few polls so the user sees data immediately.
        self.send(ubx_frame(CLASS_MON, ID_MON_VER))
        self.send(ubx_frame(CLASS_MON, ID_MON_HW))
        return True

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        if self.ser is not None:
            try: self.ser.close()
            except Exception: pass
        self.ser = None
        post_status("Disconnected", "warn")

    def send(self, frame: bytes) -> None:
        if not self.is_open():
            return
        with self.lock:
            try:
                self.ser.write(frame)
                self.ser.flush()
            except Exception as exc:
                post_status(f"TX error: {exc}", "err")

    def send_valset_labeled(self, items: list, label: str,
                            layers: int = 0x07) -> None:
        """Send one CFG-VALSET frame and tag it so the ACK/NAK is labeled."""
        if not items:
            return
        if not self.is_open():
            return
        self.valset_labels.put(label)
        self.send(cfg_valset(items, layers=layers))

    # --- background reader: parse UBX, update LiveState -----------------
    def _reader(self) -> None:
        ser = self.ser
        buf = bytearray()
        while not self._stop.is_set() and ser is not None and ser.is_open:
            try:
                chunk = ser.read(256)
            except Exception:
                break
            if not chunk:
                continue
            buf.extend(chunk)
            # Slide through buffer looking for sync
            while len(buf) >= 8:
                i = buf.find(UBX_SYNC)
                if i < 0:
                    del buf[:-1]
                    break
                if i > 0:
                    del buf[:i]
                if len(buf) < 8:
                    break
                length = buf[4] | (buf[5] << 8)
                total = 6 + length + 2
                if length > 4096:
                    del buf[:2]
                    continue
                if len(buf) < total:
                    break
                msg_class = buf[2]
                msg_id    = buf[3]
                payload   = bytes(buf[6:6+length])
                ck = ubx_checksum(bytes(buf[2:6+length]))
                if ck != bytes(buf[6+length:6+length+2]):
                    del buf[:2]
                    continue
                del buf[:total]
                self._dispatch(msg_class, msg_id, payload)

    def _dispatch(self, cls: int, mid: int, payload: bytes) -> None:
        live.frames_in += 1
        if cls == CLASS_NAV and mid == ID_NAV_PVT and len(payload) >= 92:
            self._parse_nav_pvt(payload)
        elif cls == CLASS_NAV and mid == ID_NAV_SAT and len(payload) >= 8:
            self._parse_nav_sat(payload)
        elif cls == CLASS_MON and mid == ID_MON_HW and len(payload) >= 60:
            self._parse_mon_hw(payload)
        elif cls == CLASS_MON and mid == ID_MON_VER and len(payload) >= 40:
            self._parse_mon_ver(payload)
        elif cls == CLASS_ACK:
            ack = "ACK" if mid == ID_ACK_ACK else "NAK"
            if len(payload) >= 2:
                cls_acked = payload[0]
                id_acked  = payload[1]
                # Attach a label if we tagged this VALSET.
                label = ""
                if cls_acked == CLASS_CFG and id_acked == ID_CFG_VALSET:
                    try:
                        label = self.valset_labels.get_nowait()
                    except queue.Empty:
                        label = ""
                tail = f" [{label}]" if label else ""
                post_status(
                    f"{ack} class=0x{cls_acked:02X} id=0x{id_acked:02X}{tail}",
                    "ok" if mid == ID_ACK_ACK else "err",
                )

    def _parse_nav_pvt(self, p: bytes) -> None:
        # See u-blox protocol spec (HPG 1.32) for NAV-PVT layout.
        fix_type = p[20]
        flags    = p[21]
        num_sv   = p[23]
        lon_e7, lat_e7, _, h_msl_mm = struct.unpack_from("<iiii", p, 24)
        h_acc_mm, v_acc_mm = struct.unpack_from("<II", p, 40)
        vN_mm, vE_mm, vD_mm, gs_mm = struct.unpack_from("<iiii", p, 48)
        head_mot_e5 = struct.unpack_from("<i", p, 64)[0]
        p_dop_raw = struct.unpack_from("<H", p, 76)[0]
        with live.lock:
            live.have_pvt   = True
            live.fix_type   = fix_type
            live.flags      = flags
            live.num_sv     = num_sv
            live.lat        = lat_e7 * 1e-7
            live.lon        = lon_e7 * 1e-7
            live.alt_msl    = h_msl_mm * 1e-3
            live.h_acc_m    = h_acc_mm * 1e-3
            live.v_acc_m    = v_acc_mm * 1e-3
            live.vN         = vN_mm * 1e-3
            live.vE         = vE_mm * 1e-3
            live.vD         = vD_mm * 1e-3
            live.gnd_speed  = gs_mm * 1e-3
            live.head_mot   = head_mot_e5 * 1e-5
            live.p_dop      = p_dop_raw * 0.01
            live.last_pvt_t = time.time()

    def _parse_nav_sat(self, p: bytes) -> None:
        num_svs = p[5]
        sats = []
        for i in range(num_svs):
            off = 8 + i * 12
            if off + 12 > len(p):
                break
            gnssId = p[off]
            svId   = p[off + 1]
            cno    = p[off + 2]
            flags  = struct.unpack_from("<I", p, off + 8)[0]
            used   = bool(flags & 0x08)
            sats.append((gnssId, svId, cno, used))
        sats.sort(key=lambda s: (-s[2], s[0], s[1]))
        with live.lock:
            live.sats = sats[:24]

    def _parse_mon_hw(self, p: bytes) -> None:
        noise_per_ms = struct.unpack_from("<H", p, 16)[0]
        agc_cnt      = struct.unpack_from("<H", p, 18)[0]
        ant_status   = p[20]
        ant_power    = p[21]
        flags        = p[22]
        jam_state    = (flags >> 2) & 0x03
        jam_ind      = p[45]
        with live.lock:
            live.have_hw      = True
            live.noise_per_ms = noise_per_ms
            live.agc_cnt      = agc_cnt
            live.ant_status   = ant_status
            live.ant_power    = ant_power
            live.jam_state    = jam_state
            live.jam_ind      = jam_ind
            live.last_hw_t    = time.time()

    def _parse_mon_ver(self, p: bytes) -> None:
        sw = p[0:30].split(b"\x00", 1)[0].decode("ascii", "replace")
        hw = p[30:40].split(b"\x00", 1)[0].decode("ascii", "replace")
        ext = []
        off = 40
        while off + 30 <= len(p):
            ext.append(p[off:off + 30].split(b"\x00", 1)[0].decode("ascii", "replace"))
            off += 30
        with live.lock:
            live.sw_version = sw
            live.hw_version = hw
            live.extensions = ext


link = UbxLink()


# ============================================================================
# Periodic poller (asks the receiver for HW + SAT status while connected)
# ============================================================================

def poller_thread() -> None:
    last_hw = 0.0
    last_sat = 0.0
    while True:
        time.sleep(0.5)
        if not link.is_open():
            continue
        now = time.time()
        if now - last_hw > 1.5:
            link.send(ubx_frame(CLASS_MON, ID_MON_HW))
            last_hw = now
        if now - last_sat > 1.5:
            link.send(ubx_frame(CLASS_NAV, ID_NAV_SAT))
            last_sat = now


# ============================================================================
# Configuration build helpers
# ============================================================================

def build_drone_clean_items() -> list:
    """Disable every NMEA + extra NAV msg on every port (input UBX-only)."""
    items = []
    items += [(K_UART1_IN_UBX,1), (K_UART1_IN_NMEA,0), (K_UART1_IN_RTCM,0)]
    items += [(K_UART2_IN_UBX,1), (K_UART2_IN_NMEA,0), (K_UART2_IN_RTCM,0)]
    items += [(K_UART1_OUT_UBX,1),(K_UART1_OUT_NMEA,0),(K_UART1_OUT_RTCM,0)]
    items += [(K_UART2_OUT_UBX,1),(K_UART2_OUT_NMEA,0),(K_UART2_OUT_RTCM,0)]
    items += [(K_USB_OUT_UBX,1),  (K_USB_OUT_NMEA,0)]
    for keys in NMEA_KEYS.values():
        for k in keys:
            items.append((k, 0))
    for keys in NAV_SILENCE_KEYS.values():
        for k in keys:
            items.append((k, 0))
    return items


# ============================================================================
# Auto-detect serial port
# ============================================================================

def auto_port() -> str | None:
    print("Scanning for serial ports...")
    for port, desc, _ in sorted(serial.tools.list_ports.comports()):
        good = any(t in port for t in ("usbmodem", "usbserial", "SLAB_USBtoUART",
                                       "ttyACM", "ttyUSB", "COM"))
        marker = "\u2713" if good else "\u00b7"
        print(f"  {marker} {port} ({desc})")
        if good:
            return port
    return None


# ============================================================================
# Main UI
# ============================================================================

def main() -> None:
    parser = argparse.ArgumentParser(description="ZED-F9P configurator + monitor")
    parser.add_argument("port", nargs="?", default=None,
                        help="Serial port (default: auto-detect)")
    parser.add_argument("--baud", type=int, default=38400,
                        help="Initial baud (default 38400; try 9600 on a fresh module)")
    args = parser.parse_args()

    initial_port = args.port or auto_port() or ""
    initial_baud = args.baud

    threading.Thread(target=poller_thread, daemon=True).start()

    # ---- Style -----------------------------------------------------------
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

    fig = plt.figure(figsize=(16, 10))
    fit_to_screen_height(fig)
    fig.patch.set_facecolor(BG_COLOR)
    fig.suptitle("ZED-F9P Configurator & Live Monitor",
                 fontsize=15, color=TEXT_COLOR, fontweight="bold", y=0.985)

    # ---- Layout (manual axes) ------------------------------------------
    # Left half: live monitor (x: 0.02 .. 0.62)
    ax_banner   = fig.add_axes([0.01, 0.925, 0.98, 0.045])  # MON-VER banner
    # Row A: Fix info | 2D map | Altitude bar
    ax_fix      = fig.add_axes([0.02, 0.560, 0.15, 0.330])
    ax_map      = fig.add_axes([0.21, 0.560, 0.32, 0.330])
    ax_alt      = fig.add_axes([0.56, 0.560, 0.05, 0.330])
    # Row B: Velocity line chart (wide, no xlabel needed)
    ax_vel      = fig.add_axes([0.02, 0.390, 0.59, 0.115])
    # Row C: HW status | Sat C/N0 bars
    ax_hw       = fig.add_axes([0.02, 0.110, 0.28, 0.220])
    ax_sats     = fig.add_axes([0.33, 0.110, 0.28, 0.220])

    # Right half: configuration form (x: 0.64 .. 0.99)
    ax_conn_lbl = fig.add_axes([0.64, 0.860, 0.35, 0.040])
    ax_port     = fig.add_axes([0.73, 0.812, 0.20, 0.035])
    ax_baud     = fig.add_axes([0.73, 0.762, 0.10, 0.035])
    ax_btn_conn = fig.add_axes([0.85, 0.762, 0.07, 0.085])
    ax_btn_disc = fig.add_axes([0.92, 0.762, 0.07, 0.085])

    ax_rate     = fig.add_axes([0.64, 0.580, 0.17, 0.150])
    ax_uartbaud = fig.add_axes([0.82, 0.580, 0.17, 0.150])

    ax_dyn      = fig.add_axes([0.64, 0.345, 0.17, 0.205])
    ax_gnss     = fig.add_axes([0.82, 0.345, 0.17, 0.205])

    ax_pvtports = fig.add_axes([0.64, 0.205, 0.17, 0.110])
    ax_clean    = fig.add_axes([0.82, 0.205, 0.17, 0.110])

    # Action buttons
    ax_btn_apply  = fig.add_axes([0.64, 0.143, 0.10, 0.045])
    ax_btn_save   = fig.add_axes([0.75, 0.143, 0.10, 0.045])
    ax_btn_ver    = fig.add_axes([0.86, 0.143, 0.13, 0.045])
    ax_btn_hot    = fig.add_axes([0.64, 0.090, 0.10, 0.045])
    ax_btn_cold   = fig.add_axes([0.75, 0.090, 0.10, 0.045])
    ax_btn_fact   = fig.add_axes([0.86, 0.090, 0.13, 0.045])

    # Status bar (bottom)
    ax_status   = fig.add_axes([0.02, 0.020, 0.97, 0.050])

    # ---- MON-VER banner -------------------------------------------------
    ax_banner.set_facecolor(PANEL_COLOR)
    ax_banner.set_xticks([]); ax_banner.set_yticks([])
    for sp in ax_banner.spines.values(): sp.set_edgecolor(GRID_COLOR)
    banner_text = ax_banner.text(0.01, 0.5, "Receiver: (not connected)",
                                 va="center", ha="left", fontsize=11,
                                 fontfamily="monospace", color=TEXT_COLOR,
                                 transform=ax_banner.transAxes)

    # ---- Fix panel ------------------------------------------------------
    ax_fix.set_facecolor(PANEL_COLOR)
    ax_fix.set_xticks([]); ax_fix.set_yticks([])
    for sp in ax_fix.spines.values(): sp.set_edgecolor(GRID_COLOR)
    ax_fix.set_title("Live Fix (UBX-NAV-PVT)", color=DIM_TEXT, fontsize=11)
    fix_big = ax_fix.text(0.5, 0.85, "NO FIX", ha="center", va="center",
                          fontsize=18, fontweight="bold", color=ACCENT_RED,
                          transform=ax_fix.transAxes)
    fix_text = ax_fix.text(0.05, 0.68, "Waiting...", va="top", ha="left",
                           fontsize=9, fontfamily="monospace",
                           color=TEXT_COLOR, transform=ax_fix.transAxes)

    # ---- 2D position map -----------------------------------------------
    ax_map.set_facecolor(PANEL_COLOR)
    ax_map.set_title("Position Map (local NED, m)",
                     color=DIM_TEXT, fontsize=11)
    ax_map.set_xlabel("East (m)", color=DIM_TEXT, fontsize=9)
    ax_map.set_ylabel("North (m)", color=DIM_TEXT, fontsize=9)
    ax_map.set_aspect("equal", adjustable="datalim")
    ax_map.grid(True, alpha=0.3)
    ax_map.axhline(y=0, color=GRID_COLOR, lw=1)
    ax_map.axvline(x=0, color=GRID_COLOR, lw=1)
    ax_map.set_xlim(-5, 5); ax_map.set_ylim(-5, 5)
    line_trail, = ax_map.plot([], [], color=ACCENT_BLUE, lw=1.0, alpha=0.7)
    pt_home = ax_map.scatter([0], [0], c=ACCENT_GREEN, s=80, marker="*",
                             zorder=5, label="Home")
    pt_now = ax_map.scatter([0], [0], c=ACCENT_RED, s=50, zorder=6,
                            label="Now")
    arrow_vel = ax_map.annotate("", xy=(0, 0), xytext=(0, 0),
                                arrowprops=dict(arrowstyle="->",
                                                color=ACCENT_YELLOW, lw=2))
    map_acc_circle = plt.Circle((0, 0), 0.0, fill=False,
                                edgecolor=ACCENT_CYAN, lw=1.0, alpha=0.6,
                                linestyle="--")
    ax_map.add_patch(map_acc_circle)
    ax_map.legend(loc="upper right", fontsize=7, facecolor=PANEL_COLOR,
                  edgecolor=GRID_COLOR)
    map_info = ax_map.text(0.02, 0.98, "", va="top", ha="left", fontsize=8,
                           fontfamily="monospace", color=DIM_TEXT,
                           transform=ax_map.transAxes)

    # ---- Altitude bar ---------------------------------------------------
    ax_alt.set_facecolor(PANEL_COLOR)
    ax_alt.set_title("Alt MSL (m)", color=DIM_TEXT, fontsize=10)
    ax_alt.set_xticks([])
    ax_alt.set_xlim(-0.5, 0.5)
    ax_alt.set_ylim(-1, 1)
    ax_alt.grid(True, axis="y", alpha=0.3)
    bar_alt = ax_alt.bar([0], [0], color=ACCENT_ORANGE, width=0.6)
    txt_alt = ax_alt.text(0.5, 0.05, "\u2014", ha="center", va="bottom",
                          color=TEXT_COLOR, fontsize=10,
                          fontfamily="monospace",
                          transform=ax_alt.transAxes)
    line_alt_min = ax_alt.axhline(y=0, color=ACCENT_BLUE, lw=0.8, ls="--",
                                  alpha=0.6)
    line_alt_max = ax_alt.axhline(y=0, color=ACCENT_RED,  lw=0.8, ls="--",
                                  alpha=0.6)

    # ---- Velocity line chart -------------------------------------------
    ax_vel.set_facecolor(PANEL_COLOR)
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
    ax_vel.tick_params(axis="x", labelsize=7)

    # ---- Hardware panel -------------------------------------------------
    ax_hw.set_facecolor(PANEL_COLOR)
    ax_hw.set_xticks([]); ax_hw.set_yticks([])
    for sp in ax_hw.spines.values(): sp.set_edgecolor(GRID_COLOR)
    ax_hw.set_title("Hardware (UBX-MON-HW)", color=DIM_TEXT, fontsize=11)
    hw_text = ax_hw.text(0.04, 0.92, "—", va="top", ha="left", fontsize=10,
                         fontfamily="monospace", color=TEXT_COLOR,
                         transform=ax_hw.transAxes)

    # ---- Satellite C/N0 chart ------------------------------------------
    ax_sats.set_title("Sat C/N0 (UBX-NAV-SAT, top 24)",
                      color=DIM_TEXT, fontsize=11)
    ax_sats.set_facecolor(PANEL_COLOR)
    ax_sats.set_xlim(0, 24); ax_sats.set_ylim(0, 55)
    ax_sats.set_ylabel("dB-Hz")
    ax_sats.grid(True, axis="y", alpha=0.3)
    sat_bars = ax_sats.bar(np.arange(24), np.zeros(24),
                           color=ACCENT_BLUE, width=0.85)
    sat_xticks = ax_sats.set_xticklabels([""] * 24, fontsize=7,
                                         color=DIM_TEXT, rotation=70)

    # ---- Connection controls -------------------------------------------
    ax_conn_lbl.set_facecolor(PANEL_COLOR)
    ax_conn_lbl.set_xticks([]); ax_conn_lbl.set_yticks([])
    for sp in ax_conn_lbl.spines.values(): sp.set_edgecolor(GRID_COLOR)
    ax_conn_lbl.text(0.5, 0.5, "Connection",
                     ha="center", va="center", color=TEXT_COLOR,
                     fontsize=12, fontweight="bold",
                     transform=ax_conn_lbl.transAxes)

    tb_port = TextBox(ax_port, "Port  ", initial=initial_port,
                      color=PANEL_COLOR, hovercolor="#303030",
                      label_pad=0.02)
    tb_baud = TextBox(ax_baud, "Baud  ", initial=str(initial_baud),
                      color=PANEL_COLOR, hovercolor="#303030",
                      label_pad=0.02)
    for tb in (tb_port, tb_baud):
        tb.label.set_color(DIM_TEXT)
        tb.text_disp.set_color(TEXT_COLOR)

    btn_connect = Button(ax_btn_conn, "Connect",
                         color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_disconnect = Button(ax_btn_disc, "Disconnect",
                            color=BTN_RED, hovercolor=BTN_RED_HOV)

    # ---- Rate radio -----------------------------------------------------
    ax_rate.set_facecolor(PANEL_COLOR)
    ax_rate.set_title("Measurement Rate", color=DIM_TEXT, fontsize=10)
    for sp in ax_rate.spines.values(): sp.set_edgecolor(GRID_COLOR)
    rate_radio = RadioButtons(ax_rate,
                              [lab for lab, _ in RATE_CHOICES],
                              active=2, activecolor=ACCENT_GREEN)
    for lbl in rate_radio.labels:
        lbl.set_color(TEXT_COLOR); lbl.set_fontsize(9)

    # ---- UART baud radio ------------------------------------------------
    ax_uartbaud.set_facecolor(PANEL_COLOR)
    ax_uartbaud.set_title("UART1+UART2 Baud", color=DIM_TEXT, fontsize=10)
    for sp in ax_uartbaud.spines.values(): sp.set_edgecolor(GRID_COLOR)
    baud_radio = RadioButtons(ax_uartbaud,
                              [lab for lab, _ in BAUD_CHOICES],
                              active=1, activecolor=ACCENT_GREEN)
    for lbl in baud_radio.labels:
        lbl.set_color(TEXT_COLOR); lbl.set_fontsize(9)

    # ---- Dynamic model radio -------------------------------------------
    ax_dyn.set_facecolor(PANEL_COLOR)
    ax_dyn.set_title("Dynamic Platform Model", color=DIM_TEXT, fontsize=10)
    for sp in ax_dyn.spines.values(): sp.set_edgecolor(GRID_COLOR)
    dyn_radio = RadioButtons(ax_dyn,
                             [lab for lab, _ in DYNMODEL_CHOICES],
                             active=6, activecolor=ACCENT_GREEN)
    for lbl in dyn_radio.labels:
        lbl.set_color(TEXT_COLOR); lbl.set_fontsize(8)

    # ---- GNSS check buttons --------------------------------------------
    ax_gnss.set_facecolor(PANEL_COLOR)
    ax_gnss.set_title("GNSS Constellations", color=DIM_TEXT, fontsize=10)
    for sp in ax_gnss.spines.values(): sp.set_edgecolor(GRID_COLOR)
    gnss_chk = CheckButtons(ax_gnss,
                            [lab for lab, _, _ in GNSS_LIST],
                            [enabled for _, _, enabled in GNSS_LIST])
    for lbl in gnss_chk.labels:
        lbl.set_color(TEXT_COLOR); lbl.set_fontsize(9)

    # ---- NAV-PVT output ports ------------------------------------------
    ax_pvtports.set_facecolor(PANEL_COLOR)
    ax_pvtports.set_title("Output NAV-PVT on", color=DIM_TEXT, fontsize=10)
    for sp in ax_pvtports.spines.values(): sp.set_edgecolor(GRID_COLOR)
    pvt_chk = CheckButtons(ax_pvtports,
                           ["UART1", "UART2", "USB"],
                           [True, True, True])
    for lbl in pvt_chk.labels:
        lbl.set_color(TEXT_COLOR); lbl.set_fontsize(9)

    # ---- Drone-clean toggle --------------------------------------------
    ax_clean.set_facecolor(PANEL_COLOR)
    ax_clean.set_title("Output protocol policy",
                       color=DIM_TEXT, fontsize=10)
    for sp in ax_clean.spines.values(): sp.set_edgecolor(GRID_COLOR)
    clean_chk = CheckButtons(ax_clean,
                             ["Drone-clean (UBX only,\nsilence all NMEA)"],
                             [True])
    for lbl in clean_chk.labels:
        lbl.set_color(TEXT_COLOR); lbl.set_fontsize(8)

    # ---- Action buttons -------------------------------------------------
    btn_apply = Button(ax_btn_apply, "Apply Config",
                       color=BTN_BLUE, hovercolor=BTN_BLUE_HOV)
    btn_save  = Button(ax_btn_save, "Save to Flash",
                       color=BTN_GREEN, hovercolor=BTN_GREEN_HOV)
    btn_ver   = Button(ax_btn_ver, "Read Version",
                       color=BTN_GREY, hovercolor=BTN_GREY_HOV)
    btn_hot   = Button(ax_btn_hot, "Hot Reset",
                       color=BTN_GREY, hovercolor=BTN_GREY_HOV)
    btn_cold  = Button(ax_btn_cold, "Cold Reset",
                       color=BTN_RED,  hovercolor=BTN_RED_HOV)
    btn_fact  = Button(ax_btn_fact, "Factory Default",
                       color=BTN_RED,  hovercolor=BTN_RED_HOV)

    # ---- Status bar -----------------------------------------------------
    ax_status.set_facecolor(PANEL_COLOR)
    ax_status.set_xticks([]); ax_status.set_yticks([])
    for sp in ax_status.spines.values(): sp.set_edgecolor(GRID_COLOR)
    status_text = ax_status.text(0.01, 0.5,
                                 "Idle. Set port + baud and press Connect.",
                                 va="center", ha="left", fontsize=10,
                                 fontfamily="monospace", color=DIM_TEXT,
                                 transform=ax_status.transAxes)
    status_recent: list[tuple[str, str, float]] = []

    # ====================================================================
    # Helpers used by callbacks
    # ====================================================================

    def selected_radio_value(radio: RadioButtons, choices: list) -> int:
        sel_label = radio.value_selected
        for lab, val in choices:
            if lab == sel_label:
                return val
        return choices[0][1]

    def selected_dynmodel() -> int:
        return selected_radio_value(dyn_radio, DYNMODEL_CHOICES)

    def gnss_states() -> list[tuple[int, bool]]:
        states = gnss_chk.get_status()
        return [(GNSS_LIST[i][1], states[i]) for i in range(len(GNSS_LIST))]

    def pvt_port_states() -> tuple[bool, bool, bool]:
        s = pvt_chk.get_status()
        return s[0], s[1], s[2]

    def clean_enabled() -> bool:
        return clean_chk.get_status()[0]

    # ====================================================================
    # Button callbacks
    # ====================================================================

    def on_connect(_evt) -> None:
        if link.is_open():
            post_status("Already connected", "warn"); return
        port = tb_port.text.strip()
        if not port:
            post_status("Enter a serial port", "err"); return
        try:
            baud = int(tb_baud.text.strip())
        except ValueError:
            post_status("Baud must be an integer", "err"); return
        link.open(port, baud)

    def on_disconnect(_evt) -> None:
        if link.is_open():
            link.close()
        else:
            post_status("Not connected", "warn")

    def on_apply(_evt) -> None:
        if not link.is_open():
            post_status("Connect first", "err"); return

        # Snapshot UI selections (touching widgets from worker thread is unsafe).
        rate_ms   = selected_radio_value(rate_radio, RATE_CHOICES)
        uart_baud = selected_radio_value(baud_radio, BAUD_CHOICES)
        dyn_val   = selected_dynmodel()
        gnss_sel  = gnss_states()
        u1, u2, ub = pvt_port_states()
        do_clean   = clean_enabled()
        try:
            current_baud = int(tb_baud.text.strip())
        except ValueError:
            current_baud = 0
        is_usb_port = "USB" in (tb_port.text or "").upper() or \
                      "usbmodem" in (tb_port.text or "").lower()

        # Build labeled sub-batches. Order matters:
        #   - rate / dyn / gnss / nav-pvt outputs first (cheap, common)
        #   - drone-clean (lots of msgs) next
        #   - UART baud last so we don't drop our own connection mid-stream
        batches: list[tuple[str, list]] = []

        batches.append(("rate", [
            (K_RATE_MEAS, rate_ms),
            (K_RATE_NAV,  1),
            (K_RATE_TIMEREF, 1),
        ]))
        batches.append(("dynmodel", [(K_NAVSPG_DYNMODEL, dyn_val)]))
        # One batch per constellation so a NAK pinpoints the bad key.
        for (name, _, _), (key, en) in zip(GNSS_LIST, gnss_sel):
            batches.append((f"gnss-{name.lower()}",
                            [(key, 1 if en else 0)]))
        batches.append(("nav-pvt-out", [
            (K_MSGOUT_NAV_PVT_UART1, 1 if u1 else 0),
            (K_MSGOUT_NAV_PVT_UART2, 1 if u2 else 0),
            (K_MSGOUT_NAV_PVT_USB,   1 if ub else 0),
        ]))
        if do_clean:
            # Split drone-clean into 3 smaller batches so a NAK is localized.
            proto_items = []
            proto_items += [(K_UART1_IN_UBX,1),(K_UART1_IN_NMEA,0),(K_UART1_IN_RTCM,0)]
            proto_items += [(K_UART2_IN_UBX,1),(K_UART2_IN_NMEA,0),(K_UART2_IN_RTCM,0)]
            proto_items += [(K_UART1_OUT_UBX,1),(K_UART1_OUT_NMEA,0),(K_UART1_OUT_RTCM,0)]
            proto_items += [(K_UART2_OUT_UBX,1),(K_UART2_OUT_NMEA,0),(K_UART2_OUT_RTCM,0)]
            proto_items += [(K_USB_OUT_UBX,1),  (K_USB_OUT_NMEA,0)]
            batches.append(("protocols", proto_items))

            nmea_items = []
            for keys in NMEA_KEYS.values():
                for k in keys:
                    nmea_items.append((k, 0))
            batches.append(("silence-nmea", nmea_items))

            nav_items = []
            for keys in NAV_SILENCE_KEYS.values():
                for k in keys:
                    nav_items.append((k, 0))
            batches.append(("silence-nav", nav_items))

        # UART baud last (changing it may drop our own link if we're on UART).
        batches.append(("uart-baud", [
            (K_UART1_BAUD, uart_baud),
            (K_UART2_BAUD, uart_baud),
        ]))

        total = sum(len(b) for _, b in batches)
        post_status(f"Apply: {len(batches)} batches, {total} items "
                    f"(rate={rate_ms}ms, baud={uart_baud})", "info")

        def worker():
            for label, items in batches:
                if not link.is_open():
                    return
                post_status(f"  -> {label}: {len(items)} items", "info")
                # RAM-only: avoids Flash-layer NAKs on some F9P FW revisions.
                # User can press "Save to Flash" to persist.
                link.send_valset_labeled(items, label, layers=0x01)
                # Give the receiver time to process and ACK before next batch.
                time.sleep(0.20)
            if uart_baud != current_baud and not is_usb_port:
                post_status(
                    f"UART baud changed to {uart_baud}; reconnect at new rate.",
                    "warn",
                )

        threading.Thread(target=worker, daemon=True).start()

    def on_save(_evt) -> None:
        if not link.is_open():
            post_status("Connect first", "err"); return
        post_status("Save to BBR + Flash", "info")
        link.send(cfg_save_all())

    def on_ver(_evt) -> None:
        if not link.is_open():
            post_status("Connect first", "err"); return
        link.send(ubx_frame(CLASS_MON, ID_MON_VER))
        link.send(ubx_frame(CLASS_MON, ID_MON_HW))

    def on_hot(_evt) -> None:
        if not link.is_open():
            post_status("Connect first", "err"); return
        post_status("Hot reset (keep ephemerides)", "warn")
        link.send(cfg_reset(0x0000, 0x01))

    def on_cold(_evt) -> None:
        if not link.is_open():
            post_status("Connect first", "err"); return
        post_status("Cold reset (wipe BBR)", "warn")
        link.send(cfg_reset(0xFFFF, 0x01))

    def on_factory(_evt) -> None:
        if not link.is_open():
            post_status("Connect first", "err"); return
        post_status("Factory default + reset", "warn")
        link.send(cfg_factory_default())
        time.sleep(0.1)
        link.send(cfg_reset(0xFFFF, 0x01))

    btn_connect.on_clicked(on_connect)
    btn_disconnect.on_clicked(on_disconnect)
    btn_apply.on_clicked(on_apply)
    btn_save.on_clicked(on_save)
    btn_ver.on_clicked(on_ver)
    btn_hot.on_clicked(on_hot)
    btn_cold.on_clicked(on_cold)
    btn_fact.on_clicked(on_factory)

    # ====================================================================
    # UI refresh loop
    # ====================================================================

    # History buffers + map state (closed over by refresh())
    hist_n  = np.zeros(HISTORY_LEN)
    hist_e  = np.zeros(HISTORY_LEN)
    hist_vN = np.zeros(HISTORY_LEN)
    hist_vE = np.zeros(HISTORY_LEN)
    hist_vD = np.zeros(HISTORY_LEN)
    hist_gs = np.zeros(HISTORY_LEN)
    x_axis  = np.arange(HISTORY_LEN)
    samples = [0]
    home    = [None]   # [lat, lon] of first reliable 3-D fix
    alt_min = [None]
    alt_max = [None]
    last_pvt_seen_t = [0.0]

    def refresh(_frame):
        with live.lock:
            have_pvt = live.have_pvt
            fix_type = live.fix_type
            num_sv   = live.num_sv
            lat, lon, alt = live.lat, live.lon, live.alt_msl
            h_acc, v_acc  = live.h_acc_m, live.v_acc_m
            vN, vE, vD    = live.vN, live.vE, live.vD
            gs, head, pdop = live.gnd_speed, live.head_mot, live.p_dop
            flags = live.flags
            have_hw = live.have_hw
            noise = live.noise_per_ms
            agc   = live.agc_cnt
            ant_st = live.ant_status
            ant_pw = live.ant_power
            jam_st = live.jam_state
            jam_in = live.jam_ind
            sw, hw, ext = live.sw_version, live.hw_version, list(live.extensions)
            sats = list(live.sats)
            last_pvt_t = live.last_pvt_t

        # MON-VER banner
        if sw or hw:
            ext_short = "  ".join(ext[:3]) if ext else ""
            banner_text.set_text(f"SW: {sw}    HW: {hw}    {ext_short}")
            banner_text.set_color(TEXT_COLOR)
        elif link.is_open():
            banner_text.set_text("Receiver connected — polling MON-VER...")
            banner_text.set_color(DIM_TEXT)
        else:
            banner_text.set_text("Receiver: (not connected)")
            banner_text.set_color(DIM_TEXT)

        # Fix panel
        fix_name = FIX_NAMES.get(fix_type, f"?{fix_type}")
        fix_color = ACCENT_GREEN if fix_type >= 3 else (
            ACCENT_YELLOW if fix_type in (2, 4, 5) else ACCENT_RED)
        fix_big.set_text(fix_name)
        fix_big.set_color(fix_color)
        if have_pvt:
            valid_fix = "yes" if (flags & 0x01) else "no"
            fix_text.set_text(
                f"Sats used : {num_sv:>3d}\n"
                f"Valid fix : {valid_fix}\n"
                f"pDOP      : {pdop:6.2f}\n"
                f"H acc     : {h_acc:6.2f} m\n"
                f"V acc     : {v_acc:6.2f} m\n"
                f"Heading   : {head:6.1f}\u00b0\n"
                f"Gnd speed : {gs:6.2f} m/s\n"
                f"Frames RX : {live.frames_in:d}\n"
                f"\n"
                f"Lat: {lat:11.6f}\u00b0\n"
                f"Lon: {lon:11.6f}\u00b0\n"
            )
        else:
            fix_text.set_text("Waiting for NAV-PVT...")

        # ---- Push new PVT samples into history buffers ------------------
        if have_pvt and last_pvt_t != last_pvt_seen_t[0]:
            last_pvt_seen_t[0] = last_pvt_t
            reliable_3d = (fix_type >= 3) and bool(flags & 0x01)
            if home[0] is None and reliable_3d:
                home[0] = (lat, lon)
            if home[0] is not None:
                north_m, east_m = latlon_to_local_m(lat, lon,
                                                    home[0][0], home[0][1])
            else:
                north_m, east_m = 0.0, 0.0
            hist_n[:-1]  = hist_n[1:];  hist_n[-1]  = north_m
            hist_e[:-1]  = hist_e[1:];  hist_e[-1]  = east_m
            hist_vN[:-1] = hist_vN[1:]; hist_vN[-1] = vN
            hist_vE[:-1] = hist_vE[1:]; hist_vE[-1] = vE
            hist_vD[:-1] = hist_vD[1:]; hist_vD[-1] = vD
            hist_gs[:-1] = hist_gs[1:]; hist_gs[-1] = gs
            samples[0] += 1
            alt_min[0] = alt if alt_min[0] is None else min(alt_min[0], alt)
            alt_max[0] = alt if alt_max[0] is None else max(alt_max[0], alt)

        # ---- 2D position map -------------------------------------------
        n_valid = min(samples[0], HISTORY_LEN)
        if n_valid > 0:
            line_trail.set_data(hist_e[-n_valid:], hist_n[-n_valid:])
            pt_now.set_offsets([[hist_e[-1], hist_n[-1]]])
            arrow_vel.xy = (hist_e[-1] + vE * 2.0,
                            hist_n[-1] + vN * 2.0)
            arrow_vel.set_position((hist_e[-1], hist_n[-1]))
            map_acc_circle.center = (hist_e[-1], hist_n[-1])
            map_acc_circle.set_radius(max(h_acc, 0.0))

            xs = hist_e[-n_valid:]
            ys = hist_n[-n_valid:]
            cx = 0.5 * (xs.min() + xs.max())
            cy = 0.5 * (ys.min() + ys.max())
            half = max(2.0,
                       0.6 * max(xs.max() - xs.min(), ys.max() - ys.min()))
            ax_map.set_xlim(cx - half, cx + half)
            ax_map.set_ylim(cy - half, cy + half)
            map_info.set_text(
                f"home  {home[0][0]:.6f}, {home[0][1]:.6f}\n"
                f"now   N={hist_n[-1]:+7.2f}m  E={hist_e[-1]:+7.2f}m\n"
                f"H acc {h_acc:5.2f}m   pDOP {pdop:4.2f}"
                if home[0] is not None else
                "Waiting for reliable 3-D fix..."
            )
        else:
            map_info.set_text("Waiting for reliable 3-D fix...")

        # ---- Altitude bar ----------------------------------------------
        bar_alt[0].set_height(alt if have_pvt else 0)
        if have_pvt:
            txt_alt.set_text(f"{alt:6.2f} m")
        else:
            txt_alt.set_text("\u2014")
        if alt_min[0] is not None and alt_max[0] is not None:
            lo = min(alt_min[0], alt) - 1.0
            hi = max(alt_max[0], alt) + 1.0
            if hi - lo < 4.0:
                mid = 0.5 * (lo + hi)
                lo, hi = mid - 2.0, mid + 2.0
            ax_alt.set_ylim(lo, hi)
            line_alt_min.set_ydata([alt_min[0]])
            line_alt_max.set_ydata([alt_max[0]])

        # ---- Velocity line chart ---------------------------------------
        if n_valid > 0:
            line_vN.set_data(x_axis, hist_vN)
            line_vE.set_data(x_axis, hist_vE)
            line_vD.set_data(x_axis, hist_vD)
            line_gs.set_data(x_axis, hist_gs)
            v_max = max(2.0,
                        float(np.max(np.abs(np.stack([hist_vN, hist_vE,
                                                      hist_vD, hist_gs])))))
            ax_vel.set_ylim(-v_max * 1.2, v_max * 1.2)

        # Hardware panel
        if have_hw:
            ant_st_name = ANT_STATUS_NAMES.get(ant_st, str(ant_st))
            ant_pw_name = ANT_POWER_NAMES.get(ant_pw, str(ant_pw))
            jam_st_name = JAM_STATE_NAMES.get(jam_st, str(jam_st))
            hw_text.set_text(
                f"Antenna status : {ant_st_name}\n"
                f"Antenna power  : {ant_pw_name}\n"
                f"Noise / ms     : {noise:5d}\n"
                f"AGC count      : {agc:5d}   ({100.0*agc/8191.0:4.1f}% of max)\n"
                f"Jamming state  : {jam_st_name}\n"
                f"Jamming level  : {jam_in:3d} / 255\n"
            )
        else:
            hw_text.set_text("Polling MON-HW..." if link.is_open()
                             else "—")

        # Sat C/N0 chart
        labels = []
        for i in range(24):
            if i < len(sats):
                gnssId, svId, cno, used = sats[i]
                sat_bars[i].set_height(cno)
                sat_bars[i].set_color(ACCENT_GREEN if used else ACCENT_BLUE)
                labels.append(f"{GNSS_ID_NAMES.get(gnssId, '?')[:3]}{svId}")
            else:
                sat_bars[i].set_height(0)
                labels.append("")
        ax_sats.set_xticks(np.arange(24))
        ax_sats.set_xticklabels(labels, fontsize=7, color=DIM_TEXT, rotation=70)

        # Drain status messages
        while True:
            try:
                msg, level = status_q.get_nowait()
            except queue.Empty:
                break
            print(f"[{level}] {msg}")
            status_recent.append((msg, level, time.time()))
            # Keep the latest, but don't let an ACK/info clobber a fresh NAK.
            now = time.time()
            keep_err = next((entry for entry in reversed(status_recent)
                             if entry[1] == "err" and now - entry[2] < 8.0),
                            None)
            status_recent[:] = ([keep_err, status_recent[-1]]
                                if keep_err and keep_err is not status_recent[-1]
                                else [status_recent[-1]])

        if status_recent:
            # Prefer showing a recent error over a later success message.
            entry = status_recent[0] if status_recent[0][1] == "err" \
                    else status_recent[-1]
            msg, level, t = entry
            color = {"ok": ACCENT_GREEN, "err": ACCENT_RED,
                     "warn": ACCENT_YELLOW, "info": TEXT_COLOR}.get(level, TEXT_COLOR)
            link_marker = "●" if link.is_open() else "○"
            age = time.time() - t
            stale = "" if age < 4.0 else f"  ({age:0.0f}s ago)"
            status_text.set_text(f"{link_marker}  {msg}{stale}")
            status_text.set_color(color)
        else:
            status_text.set_text(
                "● Connected" if link.is_open()
                else "○ Idle. Set port + baud and press Connect.")
            status_text.set_color(ACCENT_GREEN if link.is_open() else DIM_TEXT)

        # Stale-link indicator
        if link.is_open() and have_pvt and (time.time() - last_pvt_t > 3.0):
            fix_big.set_text("STALE")
            fix_big.set_color(ACCENT_ORANGE)

        return ()

    _ani = FuncAnimation(fig, refresh, interval=200, blit=False,
                         cache_frame_data=False)

    def on_close(_evt):
        if link.is_open():
            link.close()

    fig.canvas.mpl_connect("close_event", on_close)
    plt.show()


if __name__ == "__main__":
    main()
