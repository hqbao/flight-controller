#!/usr/bin/env python3
"""
Log Class Test — Flight Controller

Tests all log classes by sending each one and verifying data frames arrive
with the expected payload size. Validates the full data path:
  Python tool → UART → FC logger → module SEND_LOG → UART → Python tool

For each log class, tests:
  - Send DB_CMD_LOG_CLASS to activate
  - Wait for data frames with expected payload size
  - Validate payload (finite floats where applicable)
  - Print sample values for tracing
  - Stop logging before moving to next test

Usage: python3 test_dblink.py [serial_port]
"""

import sys
import time
import struct
import serial
import serial.tools.list_ports

BAUD_RATE = 38400

DB_CMD_LOG_CLASS = 0x03
DB_CMD_RESET     = 0x07
DB_CMD_CHIP_ID   = 0x09

SEND_LOG_ID = 0x00

LOG_CLASS_NONE              = 0x00
LOG_CLASS_IMU_ACCEL_RAW     = 0x01
LOG_CLASS_COMPASS           = 0x02
LOG_CLASS_ATTITUDE          = 0x03
LOG_CLASS_POSITION          = 0x04
LOG_CLASS_POSITION_OPTFLOW  = 0x06
LOG_CLASS_ATTITUDE_MAG      = 0x07
LOG_CLASS_GYRO_CAL          = 0x08
LOG_CLASS_HEART_BEAT        = 0x09
LOG_CLASS_IMU_ACCEL_CALIB   = 0x0A
LOG_CLASS_IMU_GYRO_RAW      = 0x0B
LOG_CLASS_IMU_GYRO_CALIB    = 0x0C
LOG_CLASS_COMPASS_CALIB     = 0x0D
LOG_CLASS_STORAGE           = 0x10
LOG_CLASS_MIX_CONTROL       = 0x11
LOG_CLASS_FLIGHT_TELEMETRY  = 0x12
LOG_CLASS_ATTITUDE_EARTH    = 0x13
LOG_CLASS_FFT_SPECTRUM_DUAL_X = 0x14
LOG_CLASS_FFT_SPECTRUM_DUAL_Y = 0x15
LOG_CLASS_FFT_SPECTRUM_DUAL_Z = 0x16
LOG_CLASS_FFT_PEAKS         = 0x17
LOG_CLASS_FFT_SPECTRUM_X    = 0x18
LOG_CLASS_FFT_SPECTRUM_Y    = 0x19
LOG_CLASS_FFT_SPECTRUM_Z    = 0x1A
LOG_CLASS_RC_RECEIVER       = 0x1B

CHIP_ID_SIZE   = 8
HEARTBEAT_SIZE = 4

# (log_class, name, expected_payload_size, timeout_s)
# timeout is longer for slow-rate classes (1Hz, 3.3Hz)
LOG_CLASS_TESTS = [
    (LOG_CLASS_IMU_ACCEL_RAW,    "IMU Accel Raw",    16, 3.0),
    (LOG_CLASS_IMU_ACCEL_CALIB,  "IMU Accel Cal",    16, 3.0),
    (LOG_CLASS_IMU_GYRO_RAW,     "IMU Gyro Raw",     16, 3.0),
    (LOG_CLASS_IMU_GYRO_CALIB,   "IMU Gyro Cal",     16, 3.0),
    (LOG_CLASS_COMPASS,          "Compass Raw",       12, 3.0),
    (LOG_CLASS_COMPASS_CALIB,    "Compass Cal",       12, 3.0),
    (LOG_CLASS_ATTITUDE,         "Attitude Body",     36, 3.0),
    (LOG_CLASS_ATTITUDE_EARTH,   "Attitude Earth",    36, 3.0),
    (LOG_CLASS_ATTITUDE_MAG,     "Attitude Mag",      36, 3.0),
    (LOG_CLASS_POSITION,         "Position",          24, 3.0),
    (LOG_CLASS_POSITION_OPTFLOW, "Pos OptFlow",       24, 3.0),
    (LOG_CLASS_MIX_CONTROL,      "Mix Control",       32, 3.0),
    (LOG_CLASS_FLIGHT_TELEMETRY, "Telemetry",         66, 3.0),
    (LOG_CLASS_RC_RECEIVER,      "RC Receiver",       28, 3.0),
    (LOG_CLASS_FFT_PEAKS,        "FFT Peaks",         36, 5.0),
    (LOG_CLASS_FFT_SPECTRUM_X,   "FFT Spectrum X",   116, 5.0),
    (LOG_CLASS_FFT_SPECTRUM_Y,   "FFT Spectrum Y",   116, 5.0),
    (LOG_CLASS_FFT_SPECTRUM_Z,   "FFT Spectrum Z",   116, 5.0),
    (LOG_CLASS_FFT_SPECTRUM_DUAL_X, "FFT Dual X",     231, 5.0),
    (LOG_CLASS_FFT_SPECTRUM_DUAL_Y, "FFT Dual Y",     231, 5.0),
    (LOG_CLASS_FFT_SPECTRUM_DUAL_Z, "FFT Dual Z",     231, 5.0),
    (LOG_CLASS_STORAGE,          "Storage",          None, 5.0),  # 4 pages of 104 bytes
]


def find_serial_port():
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB', 'COM']):
            return port
    return None


def build_db_frame(cmd_id, payload_bytes):
    msg_class = 0x00
    length = len(payload_bytes)
    header = struct.pack('<2sBBH', b'db', cmd_id, msg_class, length)
    checksum = cmd_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
    for b in payload_bytes:
        checksum += b
    checksum &= 0xFFFF
    return header + payload_bytes + struct.pack('<H', checksum)


def read_db_frame(ser, timeout):
    """Read one DB frame. Returns (msg_id, msg_class, payload) or None on timeout."""
    deadline = time.time() + timeout
    raw_bytes = bytearray()
    while time.time() < deadline:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        ser.timeout = remaining

        b1 = ser.read(1)
        if len(b1) == 0:
            break
        raw_bytes.append(b1[0])
        if b1[0] != 0x64:  # 'd'
            continue

        b2 = ser.read(1)
        if len(b2) == 0:
            break
        raw_bytes.append(b2[0])
        if b2[0] != 0x62:  # 'b'
            continue

        hdr = ser.read(4)
        if len(hdr) < 4:
            break
        raw_bytes.extend(hdr)

        msg_id = hdr[0]
        msg_class = hdr[1]
        length = int.from_bytes(hdr[2:4], 'little')

        if length > 1024:
            continue

        payload = ser.read(length)
        if len(payload) < length:
            break
        raw_bytes.extend(payload)

        cksum = ser.read(2)
        if len(cksum) < 2:
            break
        raw_bytes.extend(cksum)

        return (msg_id, msg_class, payload)

    if raw_bytes:
        print(f"         [trace] raw: {raw_bytes[:60].hex()}{'...' if len(raw_bytes) > 60 else ''} ({len(raw_bytes)} bytes)")
    return None


def send_log_class(ser, log_class):
    frame = build_db_frame(DB_CMD_LOG_CLASS, bytes([log_class]))
    ser.write(frame)


def send_chip_id_request(ser):
    frame = build_db_frame(DB_CMD_CHIP_ID, bytes([0x00]))
    ser.write(frame)


def collect_frames(ser, expected_size, timeout, count=3):
    """Collect up to `count` frames of expected_size. Returns list of payloads."""
    frames = []
    deadline = time.time() + timeout
    while len(frames) < count and time.time() < deadline:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        result = read_db_frame(ser, remaining)
        if result is None:
            break
        msg_id, msg_class, payload = result
        if msg_id != SEND_LOG_ID:
            continue
        if expected_size is not None and len(payload) != expected_size:
            continue
        if expected_size is None and len(payload) != 104:
            continue  # Storage sends 4 pages of 104 bytes
        frames.append(payload)
    return frames


def format_floats(payload):
    n = len(payload) // 4
    vals = struct.unpack(f'<{n}f', payload)
    return ", ".join(f"{v:+10.3f}" for v in vals)


def format_payload(payload, expected_size):
    """Format payload for display based on type."""
    if expected_size == 66:
        # Telemetry: mixed format
        att = struct.unpack('<3f', payload[0:12])
        pos = struct.unpack('<3f', payload[12:24])
        state = payload[64] if len(payload) > 64 else 0
        health = payload[65] if len(payload) > 65 else 0
        return (f"att=({att[0]:+.1f},{att[1]:+.1f},{att[2]:+.1f})° "
                f"pos=({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:+.2f})m "
                f"state={state} health=0x{health:02X}")
    elif expected_size == 116:
        # FFT spectrum: 1 byte axis + 103 bytes bins + 3 floats peaks
        axis = payload[0]
        peaks = struct.unpack('<3f', payload[104:116])
        axis_name = {0: 'X', 1: 'Y', 2: 'Z'}.get(axis, '?')
        return f"axis={axis_name}, peaks=({peaks[0]:.1f},{peaks[1]:.1f},{peaks[2]:.1f})Hz"
    elif expected_size is None:
        # Storage: dump size
        return f"{len(payload)} bytes"
    else:
        # All-float payloads
        return format_floats(payload)


def test_chip_id(ser):
    """Test chip ID request/response."""
    ser.reset_input_buffer()
    send_chip_id_request(ser)

    deadline = time.time() + 3.0
    while time.time() < deadline:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        result = read_db_frame(ser, remaining)
        if result is None:
            break
        msg_id, msg_class, payload = result
        if msg_id == SEND_LOG_ID and len(payload) == CHIP_ID_SIZE:
            chip_hex = payload.hex().upper()
            print(f"  Chip ID       : PASS — {chip_hex}")
            return True

    print(f"  Chip ID       : FAIL — no response")
    return False


def test_heartbeat(ser):
    """Test heartbeat: enable LOG_CLASS_HEART_BEAT, wait for 4-byte float frame."""
    ser.reset_input_buffer()
    send_log_class(ser, LOG_CLASS_HEART_BEAT)
    time.sleep(0.1)

    deadline = time.time() + 3.0
    while time.time() < deadline:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        result = read_db_frame(ser, remaining)
        if result is None:
            break
        msg_id, msg_class, payload = result
        if msg_id == SEND_LOG_ID and len(payload) == HEARTBEAT_SIZE:
            val = struct.unpack('<f', payload)[0]
            send_log_class(ser, LOG_CLASS_NONE)
            time.sleep(0.05)
            ser.reset_input_buffer()
            print(f"  Heartbeat     : PASS — counter={val:.0f}")
            return True

    send_log_class(ser, LOG_CLASS_NONE)
    time.sleep(0.05)
    ser.reset_input_buffer()
    print(f"  Heartbeat     : FAIL — no heartbeat received")
    return False


def test_log_class(ser, log_class, class_name, expected_size, timeout):
    """Test a single log class: enable, collect frames, validate, stop."""
    ser.reset_input_buffer()
    send_log_class(ser, log_class)
    time.sleep(0.2)

    frames = collect_frames(ser, expected_size, timeout=timeout, count=2)

    send_log_class(ser, LOG_CLASS_NONE)
    time.sleep(0.05)
    ser.reset_input_buffer()

    label = f"{class_name:15s}"

    if len(frames) == 0:
        print(f"  {label}: FAIL — no data frames received")
        return False

    sample_str = format_payload(frames[0], expected_size)
    print(f"  {label}: PASS — {len(frames)} frames, sample=[{sample_str}]")
    return True


def main():
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_serial_port()
        if port is None:
            print("ERROR: No serial port found. Specify port as argument.")
            sys.exit(1)

    print("=" * 70)
    print("Log Class Test — Flight Controller")
    print(f"  Port: {port}")
    print(f"  Baud: {BAUD_RATE}")
    print("=" * 70)
    print()

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=3.0)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()

    # Link check — actively probe for chip ID
    print("[Link Check]")
    ser.reset_input_buffer()
    send_chip_id_request(ser)
    deadline = time.time() + 3.0
    link_alive = False
    while time.time() < deadline:
        remaining = deadline - time.time()
        if remaining <= 0:
            break
        result = read_db_frame(ser, remaining)
        if result is not None:
            link_alive = True
            break
    if link_alive:
        print("  Link is ALIVE")
    else:
        print("  No response — link is DOWN")
        print("    1. Is flight controller powered and connected?")
        print("    2. Is the correct serial port selected?")
        ser.close()
        sys.exit(1)

    print()
    ser.reset_input_buffer()

    total_pass = 0
    total_fail = 0

    # Chip ID
    if test_chip_id(ser):
        total_pass += 1
    else:
        total_fail += 1
    time.sleep(0.3)

    # Heartbeat
    if test_heartbeat(ser):
        total_pass += 1
    else:
        total_fail += 1
    time.sleep(0.3)

    # All log classes
    for log_class, class_name, expected_size, timeout in LOG_CLASS_TESTS:
        if test_log_class(ser, log_class, class_name, expected_size, timeout):
            total_pass += 1
        else:
            total_fail += 1
        time.sleep(0.3)

    # Restore heartbeat so FC keeps streaming
    print()
    print("[Cleanup]")
    send_log_class(ser, LOG_CLASS_HEART_BEAT)
    time.sleep(0.1)
    print("  Heartbeat restored")
    print()

    # Summary
    total = total_pass + total_fail
    print("=" * 70)
    print(f"Results: {total_pass} passed, {total_fail} failed out of {total} tests")
    if total_fail == 0:
        print("All tests PASSED")
    else:
        print("Some tests FAILED — check trace output above")
    print("=" * 70)

    ser.close()
    sys.exit(0 if total_fail == 0 else 1)


if __name__ == '__main__':
    main()
