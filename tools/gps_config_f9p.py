"""
gps_config_f9p.py — One-shot u-blox ZED-F9P configurator.

Connects directly to the F9P over USB CDC serial (or its UART at any baud)
and writes a clean, drone-ready configuration to RAM + BBR + Flash:

  • Disable every NMEA message on UART1, UART2, USB
  • Disable every UBX-NAV message except NAV-PVT
  • Enable UBX-NAV-PVT on UART1, UART2, USB at every nav epoch
  • Set measurement rate to 100 ms (10 Hz) — max safe rate with multi-GNSS
  • Set UART1 + UART2 baud to 38400 (matches STM32 USART2/3/4)
  • Restrict UART input to UBX (no NMEA / RTCM noise)
  • Save the whole layer set to BBR + Flash

After running once the receiver retains the configuration across power cycles.

Wire format reference: u-blox F9 HPG 1.32 interface description.

Usage:
  python3 tools/gps_config_f9p.py            # auto-detect serial port
  python3 tools/gps_config_f9p.py /dev/cu.usbmodem14201
  python3 tools/gps_config_f9p.py /dev/cu.usbmodem14201 --baud 38400
  python3 tools/gps_config_f9p.py --rate-ms 200       # 5 Hz
"""

import argparse
import struct
import sys
import time

import serial
import serial.tools.list_ports


# --- UBX wire helpers ----------------------------------------------------

UBX_SYNC = b"\xb5\x62"
CLASS_CFG = 0x06
ID_CFG_VALSET = 0x8A
ID_CFG_CFG    = 0x09
CLASS_ACK = 0x05
ID_ACK_ACK = 0x01
ID_ACK_NAK = 0x00


def ubx_checksum(payload: bytes) -> bytes:
    ck_a = ck_b = 0
    for b in payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])


def ubx_frame(msg_class: int, msg_id: int, payload: bytes) -> bytes:
    body = bytes([msg_class, msg_id]) + struct.pack("<H", len(payload)) + payload
    return UBX_SYNC + body + ubx_checksum(body)


# --- CFG-VALSET key encodings -------------------------------------------
# Storage type is encoded in bits 28..30 of the key:
#   1 = 1-bit (L)   → 1 byte  (0/1)
#   2 = 1-byte (U1/E1)
#   3 = 2-byte (U2)
#   4 = 4-byte (U4)
#   5 = 8-byte (U8)
SIZE_FOR_TYPE = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}


def key_size(key: int) -> int:
    type_id = (key >> 28) & 0x07
    if type_id not in SIZE_FOR_TYPE:
        raise ValueError(f"Unknown CFG key type for 0x{key:08X}")
    return SIZE_FOR_TYPE[type_id]


def cfg_valset(items: list, layers: int = 0x07) -> bytes:
    """Build one CFG-VALSET frame.
    layers bitmask: 0x01=RAM, 0x02=BBR, 0x04=Flash. 0x07 = all three.
    items is a list of (key, value) tuples. Value is encoded based on key size.
    """
    payload = struct.pack("<BBH", 0x00, layers, 0x0000)  # version, layers, reserved
    for key, value in items:
        size = key_size(key)
        payload += struct.pack("<I", key)
        if size == 1:
            payload += struct.pack("<B", int(value) & 0xFF)
        elif size == 2:
            payload += struct.pack("<H", int(value) & 0xFFFF)
        elif size == 4:
            payload += struct.pack("<I", int(value) & 0xFFFFFFFF)
        elif size == 8:
            payload += struct.pack("<Q", int(value) & 0xFFFFFFFFFFFFFFFF)
    return ubx_frame(CLASS_CFG, ID_CFG_VALSET, payload)


def cfg_save_all() -> bytes:
    """UBX-CFG-CFG: clear=0, save=all, load=0, deviceMask=BBR|Flash."""
    payload = struct.pack("<IIIB",
                          0x00000000,           # clearMask
                          0xFFFFFFFF,           # saveMask  (everything)
                          0x00000000,           # loadMask
                          0b00000011)           # devBBR | devFlash
    return ubx_frame(CLASS_CFG, ID_CFG_CFG, payload)


# --- Configuration key set ----------------------------------------------

# Measurement / nav rate
K_RATE_MEAS    = 0x30210001  # U2  ms between measurements
K_RATE_NAV     = 0x30210002  # U2  measurements per nav solution
K_RATE_TIMEREF = 0x20210003  # E1  0=UTC, 1=GPS, 2=GLONASS, 3=BeiDou, 4=Galileo

# UART baud
K_UART1_BAUD = 0x40520001    # U4
K_UART2_BAUD = 0x40530001    # U4

# Input / output protocol enables
K_UART1_IN_UBX   = 0x10730001
K_UART1_IN_NMEA  = 0x10730002
K_UART1_IN_RTCM  = 0x10730004
K_UART1_OUT_UBX  = 0x10740001
K_UART1_OUT_NMEA = 0x10740002
K_UART1_OUT_RTCM = 0x10740004
K_UART2_IN_UBX   = 0x10750001
K_UART2_IN_NMEA  = 0x10750002
K_UART2_IN_RTCM  = 0x10750004
K_UART2_OUT_UBX  = 0x10760001
K_UART2_OUT_NMEA = 0x10760002
K_UART2_OUT_RTCM = 0x10760004
K_USB_OUT_UBX    = 0x10780001
K_USB_OUT_NMEA   = 0x10780002

# UBX-NAV-PVT MSGOUT (one entry per port)
K_MSGOUT_NAV_PVT_UART1 = 0x20910007
K_MSGOUT_NAV_PVT_UART2 = 0x20910008
K_MSGOUT_NAV_PVT_USB   = 0x20910009

# NMEA standard messages on UART1 / UART2 / USB — disable every one
NMEA_PORTS = ("UART1", "UART2", "USB")
NMEA_MSGOUT_KEYS = {
    # standard NMEA: GGA, GLL, GSA, GSV, RMC, VTG
    "GGA": {"UART1": 0x209100BB, "UART2": 0x209100BC, "USB": 0x209100BD},
    "GLL": {"UART1": 0x209100CA, "UART2": 0x209100CB, "USB": 0x209100CC},
    "GSA": {"UART1": 0x209100C0, "UART2": 0x209100C1, "USB": 0x209100C2},
    "GSV": {"UART1": 0x209100C5, "UART2": 0x209100C6, "USB": 0x209100C7},
    "RMC": {"UART1": 0x209100AC, "UART2": 0x209100AD, "USB": 0x209100AE},
    "VTG": {"UART1": 0x209100B1, "UART2": 0x209100B2, "USB": 0x209100B3},
}

# Other UBX-NAV messages we explicitly silence (avoid bandwidth waste)
OTHER_NAV_MSGOUT_KEYS = {
    # NAV-POSLLH per port
    "POSLLH": {"UART1": 0x2091002A, "UART2": 0x2091002B, "USB": 0x2091002C},
    # NAV-VELNED per port
    "VELNED": {"UART1": 0x20910044, "UART2": 0x20910045, "USB": 0x20910046},
    # NAV-STATUS per port
    "STATUS": {"UART1": 0x2091001B, "UART2": 0x2091001C, "USB": 0x2091001D},
    # NAV-DOP per port
    "DOP":    {"UART1": 0x20910039, "UART2": 0x2091003A, "USB": 0x2091003B},
    # NAV-SAT per port
    "SAT":    {"UART1": 0x20910016, "UART2": 0x20910017, "USB": 0x20910018},
}


def build_config(rate_ms: int, uart_baud: int) -> list:
    items: list = []

    # Rate
    items.append((K_RATE_MEAS,    rate_ms))
    items.append((K_RATE_NAV,     1))
    items.append((K_RATE_TIMEREF, 1))   # GPS time reference

    # UART baud
    items.append((K_UART1_BAUD, uart_baud))
    items.append((K_UART2_BAUD, uart_baud))

    # Restrict input to UBX only (no NMEA, no RTCM by default)
    items.append((K_UART1_IN_UBX,   1))
    items.append((K_UART1_IN_NMEA,  0))
    items.append((K_UART1_IN_RTCM,  0))
    items.append((K_UART2_IN_UBX,   1))
    items.append((K_UART2_IN_NMEA,  0))
    items.append((K_UART2_IN_RTCM,  0))

    # Output: UBX only on every port
    items.append((K_UART1_OUT_UBX,  1))
    items.append((K_UART1_OUT_NMEA, 0))
    items.append((K_UART1_OUT_RTCM, 0))
    items.append((K_UART2_OUT_UBX,  1))
    items.append((K_UART2_OUT_NMEA, 0))
    items.append((K_UART2_OUT_RTCM, 0))
    items.append((K_USB_OUT_UBX,    1))
    items.append((K_USB_OUT_NMEA,   0))

    # Disable every NMEA standard message on every port
    for _, ports in NMEA_MSGOUT_KEYS.items():
        for p in NMEA_PORTS:
            items.append((ports[p], 0))

    # Disable other UBX-NAV messages on every port
    for _, ports in OTHER_NAV_MSGOUT_KEYS.items():
        for p in NMEA_PORTS:
            items.append((ports[p], 0))

    # Enable UBX-NAV-PVT on every port at every nav epoch (rate=1)
    items.append((K_MSGOUT_NAV_PVT_UART1, 1))
    items.append((K_MSGOUT_NAV_PVT_UART2, 1))
    items.append((K_MSGOUT_NAV_PVT_USB,   1))

    return items


# --- Serial transport ----------------------------------------------------

def auto_port() -> str:
    print("Scanning for serial ports...")
    for port, desc, _ in sorted(serial.tools.list_ports.comports()):
        looks_like_usb = any(t in port for t in ("usbmodem", "usbserial",
                                                  "SLAB_USBtoUART", "ttyACM",
                                                  "ttyUSB", "COM"))
        marker = "\u2713" if looks_like_usb else "\u00b7"
        print(f"  {marker} {port} ({desc})")
        if looks_like_usb:
            return port
    raise SystemExit("\u2717 No compatible serial port found.")


def send_and_wait_ack(ser: serial.Serial, frame: bytes, label: str,
                      timeout_s: float = 1.5) -> bool:
    """Send a UBX frame and wait for UBX-ACK-ACK / NAK matching the embedded class+id."""
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()

    msg_class = frame[2]
    msg_id    = frame[3]
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        b = ser.read(1)
        if not b or b[0] != 0xB5:
            continue
        b = ser.read(1)
        if not b or b[0] != 0x62:
            continue
        hdr = ser.read(4)
        if len(hdr) < 4:
            continue
        cls, mid = hdr[0], hdr[1]
        length = hdr[2] | (hdr[3] << 8)
        payload = ser.read(length)
        _ck = ser.read(2)
        if cls != CLASS_ACK or length < 2:
            continue
        if payload[0] != msg_class or payload[1] != msg_id:
            continue
        if mid == ID_ACK_ACK:
            print(f"  \u2713 {label} acknowledged")
            return True
        if mid == ID_ACK_NAK:
            print(f"  \u2717 {label} REJECTED (UBX-ACK-NAK) — check key IDs / firmware version")
            return False
    print(f"  ! {label} no ACK within {timeout_s:.1f}s (the receiver may be at a different baud)")
    return False


# --- Main ----------------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(description="Configure ZED-F9P for UBX-NAV-PVT only.")
    p.add_argument("port", nargs="?", default=None, help="Serial port (default: auto-detect)")
    p.add_argument("--baud", type=int, default=38400,
                   help="Serial baud rate to talk to the F9P (default: 38400). "
                        "Try 9600 if the receiver is fresh out of the box.")
    p.add_argument("--rate-ms", type=int, default=100,
                   help="Measurement rate in milliseconds (default: 100 = 10 Hz). "
                        "Use 200 (5 Hz) for slow links, or 50 (20 Hz) only with single-GNSS.")
    p.add_argument("--uart-baud", type=int, default=38400,
                   help="Final baud rate to set on the F9P UART1 + UART2 outputs "
                        "(default: 38400, matches STM32 USART2/3/4).")
    p.add_argument("--no-save", action="store_true",
                   help="Skip the final UBX-CFG-CFG save (configuration is RAM-only).")
    args = p.parse_args()

    port = args.port or auto_port()

    print(f"\nOpening {port} @ {args.baud} baud...")
    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except Exception as exc:
        print(f"\u2717 {exc}")
        return 1

    try:
        time.sleep(0.2)

        # Build the full configuration into one VALSET frame.
        items = build_config(args.rate_ms, args.uart_baud)
        print(f"\nWriting {len(items)} CFG-VALSET items "
              f"(rate={args.rate_ms}ms = {1000/args.rate_ms:.1f} Hz, "
              f"uart_baud={args.uart_baud})...")

        frame = cfg_valset(items, layers=0x07)
        ok = send_and_wait_ack(ser, frame, "CFG-VALSET")

        if ok and not args.no_save:
            print("\nSaving to BBR + Flash...")
            send_and_wait_ack(ser, cfg_save_all(), "CFG-CFG (save)")

        print("\nDone. Power-cycle the receiver to verify it boots into the new config.")
        return 0 if ok else 2
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
