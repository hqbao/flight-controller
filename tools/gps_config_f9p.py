#!/usr/bin/env python3
"""
Configure U-BLOX ZED-F9P to output UBX-only NAV messages on USB, UART1, UART2.
Sets UART1 and UART2 baud rate to 38400.
Uses UBX-CFG-VALSET (generation 9 configuration interface).
"""

import struct
import time
import serial
import json
import os
import sys

# UBX Protocol
UBX_SYNC1 = 0xB5
UBX_SYNC2 = 0x62

UBX_CLASS_CFG = 0x06
UBX_CLASS_ACK = 0x05
UBX_CLASS_NAV = 0x01

UBX_CFG_VALSET = 0x8A
UBX_ACK_ACK = 0x01
UBX_ACK_NAK = 0x00

UBX_NAV_PVT = 0x07
UBX_NAV_SAT = 0x35
UBX_NAV_DOP = 0x04

# UBX-CFG-VALSET layers: RAM + BBR + Flash
LAYER_RAM_BBR_FLASH = 0x07

TARGET_BAUDRATE = 38400
NAV_RATE_HZ = 10  # 10 Hz max with multi-constellation (GPS+GLO+GAL+BDS)

# ── Configuration Keys (from ZED-F9P Interface Description) ──────────────

# Navigation/measurement rate (U2, 2 byte value)
CFG_RATE_MEAS           = 0x30210001  # Measurement period in ms
CFG_RATE_NAV            = 0x30210002  # Navigation solutions per measurement

# UART baud rates (U4, 4 byte value)
CFG_UART1_BAUDRATE      = 0x40520001
CFG_UART2_BAUDRATE      = 0x40530001

# Output protocol enable/disable per port (L / bool, 1 byte value)
CFG_UART1OUTPROT_UBX   = 0x10740001
CFG_UART1OUTPROT_NMEA  = 0x10740002
CFG_UART1OUTPROT_RTCM3 = 0x10740004

CFG_UART2OUTPROT_UBX   = 0x10750001
CFG_UART2OUTPROT_NMEA  = 0x10750002
CFG_UART2OUTPROT_RTCM3 = 0x10750004

CFG_USBOUTPROT_UBX     = 0x10770001
CFG_USBOUTPROT_NMEA    = 0x10770002
CFG_USBOUTPROT_RTCM3   = 0x10770004

# NAV-PVT message rate per port (U1, 1 byte value)
CFG_MSGOUT_NAV_PVT_UART1 = 0x20910007
CFG_MSGOUT_NAV_PVT_UART2 = 0x20910008
CFG_MSGOUT_NAV_PVT_USB   = 0x20910009

# NAV-SAT message rate per port
CFG_MSGOUT_NAV_SAT_UART1 = 0x20910016
CFG_MSGOUT_NAV_SAT_UART2 = 0x20910017
CFG_MSGOUT_NAV_SAT_USB   = 0x20910018

# NAV-DOP message rate per port
CFG_MSGOUT_NAV_DOP_UART1 = 0x20910039
CFG_MSGOUT_NAV_DOP_UART2 = 0x2091003A
CFG_MSGOUT_NAV_DOP_USB   = 0x2091003B

CONFIG_FILE = os.path.expanduser("~/.ubx_reader_config.json")

# NAV message name lookup
NAV_MSG_NAMES = {
    UBX_NAV_PVT: "NAV-PVT",
    UBX_NAV_SAT: "NAV-SAT",
    UBX_NAV_DOP: "NAV-DOP",
}

# ── UBX helpers ──────────────────────────────────────────────────────────

def ubx_checksum(data):
    """Fletcher-8 checksum over class, id, length and payload bytes."""
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b

def send_ubx(ser, msg_class, msg_id, payload=b''):
    """Build and send a UBX frame."""
    length = len(payload)
    header = bytes([msg_class, msg_id, length & 0xFF, (length >> 8) & 0xFF])
    ck_a, ck_b = ubx_checksum(header + payload)
    frame = bytes([UBX_SYNC1, UBX_SYNC2]) + header + payload + bytes([ck_a, ck_b])
    ser.write(frame)
    ser.flush()

def wait_for_ack(ser, expected_class, expected_id, timeout=2.0):
    """Wait for UBX-ACK-ACK or UBX-ACK-NAK for the given class/id."""
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf.extend(chunk)
        # Scan for UBX-ACK frames
        while len(buf) >= 10:
            idx = buf.find(bytes([UBX_SYNC1, UBX_SYNC2, UBX_CLASS_ACK]))
            if idx < 0:
                buf = buf[-2:]  # keep tail in case of partial sync
                break
            if len(buf) < idx + 10:
                break  # need more data
            ack_id = buf[idx + 3]       # 0x01 = ACK, 0x00 = NAK
            cls_acked = buf[idx + 6]
            id_acked  = buf[idx + 7]
            buf = buf[idx + 10:]
            if cls_acked == expected_class and id_acked == expected_id:
                return ack_id == UBX_ACK_ACK
    return None  # timeout

def build_valset_payload(layer, key_values):
    """Build UBX-CFG-VALSET payload.
    key_values: list of (key_id, value) tuples.
    Key type is inferred from the key ID prefix byte:
        0x10 → L  (bool, 1 byte)
        0x20 → U1 (uint8, 1 byte)
        0x30 → U2 (uint16, 2 bytes)
        0x40 → U4 (uint32, 4 bytes)
    """
    # Header: version(1) + layers(1) + reserved(2)
    payload = struct.pack('<BBH', 0x00, layer, 0x0000)
    for key, val in key_values:
        payload += struct.pack('<I', key)
        size_id = (key >> 28) & 0x0F
        if size_id <= 2:      # L or U1
            payload += struct.pack('<B', val)
        elif size_id == 3:    # U2
            payload += struct.pack('<H', val)
        elif size_id == 4:    # U4
            payload += struct.pack('<I', val)
        else:
            payload += struct.pack('<B', val)  # fallback
    return payload

# ── Main ─────────────────────────────────────────────────────────────────

def main():
    # Load saved connection config
    port = None
    baudrate = None
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r') as f:
                config = json.load(f)
                port = config.get('port')
                baudrate = config.get('baudrate')
    except Exception:
        pass

    if not port:
        print("No saved GPS config found. Please run gps_read_upx.py first to detect GPS.")
        return 1

    print(f"Configuring ZED-F9P on {port} @ {baudrate} baud ...\n")

    try:
        ser = serial.Serial(port, baudrate, timeout=0.5)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return 1

    # Drain any stale data
    time.sleep(0.1)
    ser.reset_input_buffer()

    # ── Build configuration ──────────────────────────────────────────────
    meas_period_ms = 1000 // NAV_RATE_HZ  # 100 ms for 10 Hz
    # NAV-SAT is large (~300-500 bytes); output at 1 Hz even at 10 Hz nav rate
    nav_sat_divisor = NAV_RATE_HZ  # every Nth solution → 1 Hz

    key_values = [
        # Navigation rate: 10 Hz (100 ms measurement, 1 nav per measurement)
        (CFG_RATE_MEAS, meas_period_ms),
        (CFG_RATE_NAV,  1),

        # Set UART1 and UART2 baud rate to 38400
        (CFG_UART1_BAUDRATE, TARGET_BAUDRATE),
        (CFG_UART2_BAUDRATE, TARGET_BAUDRATE),

        # Enable UBX output on all three ports
        (CFG_UART1OUTPROT_UBX,   1),
        (CFG_UART2OUTPROT_UBX,   1),
        (CFG_USBOUTPROT_UBX,     1),

        # Disable NMEA output on all three ports
        (CFG_UART1OUTPROT_NMEA,  0),
        (CFG_UART2OUTPROT_NMEA,  0),
        (CFG_USBOUTPROT_NMEA,    0),

        # Disable RTCM3 output on all three ports
        (CFG_UART1OUTPROT_RTCM3, 0),
        (CFG_UART2OUTPROT_RTCM3, 0),
        (CFG_USBOUTPROT_RTCM3,   0),

        # NAV-PVT at 10 Hz on all ports (every nav solution)
        (CFG_MSGOUT_NAV_PVT_UART1, 1),
        (CFG_MSGOUT_NAV_PVT_UART2, 1),
        (CFG_MSGOUT_NAV_PVT_USB,   1),

        # NAV-SAT at 1 Hz on all ports (every 10th nav solution)
        (CFG_MSGOUT_NAV_SAT_UART1, nav_sat_divisor),
        (CFG_MSGOUT_NAV_SAT_UART2, nav_sat_divisor),
        (CFG_MSGOUT_NAV_SAT_USB,   nav_sat_divisor),

        # NAV-DOP at 10 Hz on all ports (every nav solution)
        (CFG_MSGOUT_NAV_DOP_UART1, 1),
        (CFG_MSGOUT_NAV_DOP_UART2, 1),
        (CFG_MSGOUT_NAV_DOP_USB,   1),
    ]

    payload = build_valset_payload(LAYER_RAM_BBR_FLASH, key_values)

    # ── Send and verify ──────────────────────────────────────────────────
    print("Sending UBX-CFG-VALSET (RAM + BBR + Flash) ...")
    print(f"  Navigation rate: {NAV_RATE_HZ} Hz ({meas_period_ms} ms)")
    print("  Baud rate:")
    print(f"    UART1 : {TARGET_BAUDRATE}")
    print(f"    UART2 : {TARGET_BAUDRATE}")
    print("  Output protocol:")
    print("    UART1 : UBX=ON  NMEA=OFF  RTCM3=OFF")
    print("    UART2 : UBX=ON  NMEA=OFF  RTCM3=OFF")
    print("    USB   : UBX=ON  NMEA=OFF  RTCM3=OFF")
    print("  Message rates:")
    print(f"    NAV-PVT  {NAV_RATE_HZ} Hz  on UART1, UART2, USB")
    print(f"    NAV-DOP  {NAV_RATE_HZ} Hz  on UART1, UART2, USB")
    print(f"    NAV-SAT  1 Hz  on UART1, UART2, USB (every {nav_sat_divisor}th solution)")

    send_ubx(ser, UBX_CLASS_CFG, UBX_CFG_VALSET, payload)

    result = wait_for_ack(ser, UBX_CLASS_CFG, UBX_CFG_VALSET)
    if result is True:
        print("\n✓ Configuration applied and saved to flash successfully!")
    elif result is False:
        print("\n✗ GPS responded with NAK — configuration rejected.")
        print("  Check that the ZED-F9P firmware supports these keys.")
        ser.close()
        return 1
    else:
        print("\n⚠ No ACK/NAK received (timeout). Configuration may still have been applied.")

    # If we're on a UART (not USB) the baud rate just changed — reconnect
    is_usb = 'usb' in port.lower() or 'cu.usbmodem' in port.lower()
    if not is_usb:
        print(f"\nBaud rate changed — reconnecting at {TARGET_BAUDRATE} ...")
        ser.close()
        time.sleep(0.5)
        try:
            ser = serial.Serial(port, TARGET_BAUDRATE, timeout=0.5)
        except Exception as e:
            print(f"  Failed to reconnect at {TARGET_BAUDRATE}: {e}")
            print("  Skipping message verification.")
            return 0

    # Update saved config with new baud rate if on UART
    if not is_usb:
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump({'port': port, 'baudrate': TARGET_BAUDRATE}, f)
            print(f"  Updated saved config to {TARGET_BAUDRATE} baud.")
        except Exception:
            pass

    # ── Verify: listen for NAV messages ──────────────────────────────────
    print("\n── Verifying UBX message reception (5 seconds) ──")
    ser.reset_input_buffer()

    received = {}  # msg_id -> count
    unexpected = []

    deadline = time.monotonic() + 5.0
    buf = bytearray()

    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf.extend(chunk)

        while True:
            # Find UBX sync
            idx = buf.find(bytes([UBX_SYNC1, UBX_SYNC2]))
            if idx < 0:
                buf = buf[-1:]  # keep last byte for partial sync
                break
            if idx > 0:
                # Bytes before sync — could be leftover NMEA
                garbage = buf[:idx]
                if any(ch >= 0x20 and ch < 0x7F for ch in garbage):
                    unexpected.append(f"non-UBX data ({len(garbage)} bytes): {garbage[:40]}")
                buf = buf[idx:]

            if len(buf) < 6:
                break  # need header
            msg_class = buf[2]
            msg_id = buf[3]
            plen = buf[4] | (buf[5] << 8)
            total = 6 + plen + 2  # header + payload + checksum
            if len(buf) < total:
                break  # incomplete

            # Verify checksum
            frame = buf[:total]
            data = frame[2:total - 2]
            ck_a, ck_b = ubx_checksum(data)
            if ck_a == frame[-2] and ck_b == frame[-1]:
                if msg_class == UBX_CLASS_NAV:
                    name = NAV_MSG_NAMES.get(msg_id, f"NAV-0x{msg_id:02X}")
                    received[name] = received.get(name, 0) + 1
                elif msg_class != UBX_CLASS_ACK:
                    unexpected.append(f"UBX class=0x{msg_class:02X} id=0x{msg_id:02X} len={plen}")
            else:
                unexpected.append(f"checksum fail class=0x{msg_class:02X} id=0x{msg_id:02X}")

            buf = buf[total:]

    ser.close()

    # ── Report ───────────────────────────────────────────────────────────
    if received:
        print("\n  ✓ Received UBX NAV messages:")
        for name, count in sorted(received.items()):
            print(f"    {name:12s}  ×{count}")
    else:
        print("\n  ✗ No UBX NAV messages received in 5 seconds!")
        print("    Check wiring and that the GPS has a fix or is powered on.")

    if unexpected:
        print(f"\n  ⚠ Unexpected data ({len(unexpected)} items):")
        for item in unexpected[:5]:
            print(f"    {item}")
    else:
        if received:
            print("  ✓ No NMEA or unexpected data — UBX-only output confirmed.")

    print(f"\nDone. You can now run: python3 gps_read_upx.py")
    return 0

if __name__ == "__main__":
    exit(main())
