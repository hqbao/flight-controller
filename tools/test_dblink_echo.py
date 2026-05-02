#!/usr/bin/env python3
"""
DBLink Echo Benchmark — round-trip throughput / drop / latency measurement.

How it works:
  - Stops any active log class so the only frames flowing are echo replies.
  - Sends N DB_CMD_ECHO requests at a chosen rate.  Each payload starts with
    a 4-byte little-endian sequence number followed by zero padding to the
    requested size.
  - The FC's db_sender re-publishes the payload via SEND_LOG (id=0x00).
  - This tool listens for replies, matches them by sequence number, and
    reports throughput, drops, RTT statistics and out-of-order arrivals.

Use this to localise lag in the FC -> dblink -> Python pipeline:
  - Low send-rate that still drops -> link / framing issue
  - High RTT but no drops             -> queue depth / serialisation latency
  - Many drops only at high rate      -> baud-rate saturation
  - All sequence numbers received OOO -> reordering somewhere (shouldn't happen)

Usage:
  python3 test_dblink_echo.py [serial_port] [--rate HZ] [--size BYTES] [--count N]

Defaults: 25 Hz, 64-byte payload, 250 packets (10 s).
"""

import argparse
import struct
import sys
import time
import statistics

import serial
import serial.tools.list_ports


BAUD_RATE = 38400

DB_CMD_LOG_CLASS = 0x03
DB_CMD_CHIP_ID   = 0x09
DB_CMD_ECHO      = 0x0B
LOG_CLASS_NONE   = 0x00
SEND_LOG_ID      = 0x00


def find_serial_port():
    for port, _desc, _hwid in sorted(serial.tools.list_ports.comports()):
        if any(x in port for x in ('usbmodem', 'usbserial', 'SLAB_USBtoUART',
                                    'ttyACM', 'ttyUSB', 'COM')):
            return port
    return None


def build_db_frame(cmd_id, payload):
    """Build a 'db' wire frame.  Header: 'd''b' ID Class Length(LE)."""
    msg_class = 0x00
    length = len(payload)
    header = struct.pack('<2sBBH', b'db', cmd_id, msg_class, length)
    cksum = (cmd_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
             + sum(payload)) & 0xFFFF
    return header + payload + struct.pack('<H', cksum)


def send_log_class(ser, log_class):
    ser.write(build_db_frame(DB_CMD_LOG_CLASS, bytes([log_class])))
    ser.flush()


def send_echo(ser, seq, size):
    """Echo payload = uint32 seq (LE) + zero padding to `size`."""
    payload = struct.pack('<I', seq) + b'\x00' * (size - 4)
    ser.write(build_db_frame(DB_CMD_ECHO, payload))


def send_chipid(ser, seq, size):
    """Probe alternative: hammer DB_CMD_CHIP_ID instead of echo.  Reply is
    always 8 bytes (the chip ID) and carries no sequence number, so we can
    only count replies, not match them.  Used to A/B test whether the
    echo handler itself is the problem vs general system load."""
    ser.write(build_db_frame(DB_CMD_CHIP_ID, b'\x00'))


def parse_db_stream(buf):
    """Generator: yield (msg_id, payload) for every complete frame in buf,
    consuming bytes as it goes.  Returns the leftover (incomplete) bytes."""
    out = []
    i = 0
    while True:
        j = buf.find(b'db', i)
        if j < 0:
            # Drop everything except a possible trailing 'd'.
            tail = buf[-1:] if buf.endswith(b'd') else b''
            return out, tail
        if len(buf) - j < 6:
            return out, bytes(buf[j:])
        length = int.from_bytes(buf[j + 4:j + 6], 'little')
        if length > 1024:
            i = j + 2  # bogus length, skip past sync
            continue
        frame_total = 6 + length + 2
        if len(buf) - j < frame_total:
            return out, bytes(buf[j:])
        msg_id = buf[j + 2]
        payload = bytes(buf[j + 6:j + 6 + length])
        out.append((msg_id, payload))
        i = j + frame_total


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port', nargs='?', help='Serial port (auto-detect if omitted)')
    ap.add_argument('--rate', type=float, default=10.0, help='Send rate in Hz')
    ap.add_argument('--size', type=int, default=64,
                    help='Echo payload size in bytes (>=4, <=240)')
    ap.add_argument('--count', type=int, default=100, help='Number of echoes to send')
    ap.add_argument('--rx-timeout', type=float, default=2.0,
                    help='Seconds to wait for trailing replies after last send')
    ap.add_argument('--probe', choices=['echo', 'chipid'], default='echo',
                    help='echo: use DB_CMD_ECHO (default).  '
                         'chipid: hammer DB_CMD_CHIP_ID at the same rate '
                         '(A/B test to isolate handler vs system load).')
    args = ap.parse_args()
    if args.probe == 'chipid':
        args.size = 8  # chip-ID reply is always 8 bytes

    if args.size < 4 or args.size > 240:
        sys.exit('--size must be in [4, 240]')

    port = args.port or find_serial_port()
    if not port:
        sys.exit('No serial port found.  Specify one as argument.')

    print('=' * 70)
    print('DBLink Echo Benchmark')
    print(f'  Port  : {port}  @ {BAUD_RATE} baud')
    print(f'  Rate  : {args.rate:.1f} Hz')
    print(f'  Size  : {args.size} B payload  ({args.size + 8} B wire frame)')
    print(f'  Count : {args.count} echoes')
    wire_bps = (args.size + 8) * args.rate * 2  # request + reply
    print(f'  Wire  : ~{wire_bps:.0f} B/s round-trip '
          f'({100.0 * wire_bps / (BAUD_RATE / 10):.0f}% of {BAUD_RATE} baud)')
    print('=' * 70)
    print()

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f'Cannot open {port}: {e}')

    time.sleep(0.3)

    # Silence everything else so the only SEND_LOG frames are echo replies.
    print('-> stopping all log streams...')
    send_log_class(ser, LOG_CLASS_NONE)
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Sanity probe: ask for the chip ID to confirm the link is alive and the
    # FC is responsive *before* judging echo failures.
    print('-> probing link with DB_CMD_CHIP_ID...')
    ser.write(build_db_frame(DB_CMD_CHIP_ID, b'\x00'))
    ser.flush()
    probe_buf = bytearray()
    probe_deadline = time.perf_counter() + 1.5
    chip_ok = False
    while time.perf_counter() < probe_deadline:
        n = ser.in_waiting
        if n:
            probe_buf.extend(ser.read(n))
        frames, tail = parse_db_stream(probe_buf)
        probe_buf = bytearray(tail)
        for msg_id, payload in frames:
            if msg_id == SEND_LOG_ID and len(payload) == 8:
                print(f'   chip ID = {payload.hex().upper()}  (link OK)')
                chip_ok = True
                break
        if chip_ok:
            break
        time.sleep(0.02)
    if not chip_ok:
        print('   FAIL: no chip-ID reply.  Link or FC is dead — fix that first.')
        ser.close()
        sys.exit(2)
    ser.reset_input_buffer()
    print()

    period = 1.0 / args.rate
    send_times = {}        # seq -> tx timestamp
    rtts = []              # in ms
    rx_seqs = []
    rx_buf = bytearray()
    duplicates = 0
    bad_size = 0
    out_of_order = 0
    last_rx_seq = -1

    def drain():
        nonlocal duplicates, bad_size, out_of_order, last_rx_seq
        n = ser.in_waiting
        if n:
            rx_buf.extend(ser.read(n))
        frames, tail = parse_db_stream(rx_buf)
        rx_buf.clear()
        rx_buf.extend(tail)
        now = time.perf_counter()
        for msg_id, payload in frames:
            if msg_id != SEND_LOG_ID:
                continue
            if len(payload) != args.size:
                bad_size += 1
                continue
            seq = struct.unpack_from('<I', payload, 0)[0]
            tx = send_times.pop(seq, None)
            if tx is None:
                duplicates += 1
                continue
            rtts.append((now - tx) * 1000.0)
            rx_seqs.append(seq)
            if seq < last_rx_seq:
                out_of_order += 1
            last_rx_seq = max(last_rx_seq, seq)

    # In chipid mode, replies have no seq number — match them in arrival order.
    chipid_pending = []  # list of tx timestamps, FIFO

    def drain_chipid():
        nonlocal duplicates, bad_size
        n = ser.in_waiting
        if n:
            rx_buf.extend(ser.read(n))
        frames, tail = parse_db_stream(rx_buf)
        rx_buf.clear()
        rx_buf.extend(tail)
        now = time.perf_counter()
        for msg_id, payload in frames:
            if msg_id != SEND_LOG_ID:
                continue
            if len(payload) != 8:
                bad_size += 1
                continue
            if not chipid_pending:
                duplicates += 1
                continue
            tx = chipid_pending.pop(0)
            rtts.append((now - tx) * 1000.0)

    sender = send_echo if args.probe == 'echo' else send_chipid
    drain_fn = drain if args.probe == 'echo' else drain_chipid

    # --- Send phase ---
    t0 = time.perf_counter()
    next_tx = t0
    for seq in range(args.count):
        # Pace
        sleep = next_tx - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        if args.probe == 'echo':
            send_times[seq] = time.perf_counter()
        else:
            chipid_pending.append(time.perf_counter())
        sender(ser, seq, args.size)
        next_tx += period
        # Opportunistic drain so the OS RX buffer doesn't grow unbounded.
        drain_fn()

    send_elapsed = time.perf_counter() - t0
    print(f'-> sent {args.count} echoes in {send_elapsed:.2f}s '
          f'({args.count / send_elapsed:.1f} Hz actual)')

    # --- Drain trailing replies ---
    drain_deadline = time.perf_counter() + args.rx_timeout
    while time.perf_counter() < drain_deadline:
        time.sleep(0.02)
        drain_fn()
        pending = send_times if args.probe == 'echo' else chipid_pending
        if not pending:  # all replies received
            break

    # Restore default heartbeat so the FC keeps streaming for other tools.
    send_log_class(ser, 0x09)  # LOG_CLASS_HEART_BEAT
    ser.close()

    # --- Report ---
    received = len(rtts)
    sent     = args.count
    dropped  = sent - received
    drop_pct = 100.0 * dropped / sent if sent else 0.0

    print()
    print('=' * 70)
    print('Results')
    print('-' * 70)
    print(f'  Sent           : {sent}')
    print(f'  Received       : {received}')
    print(f'  Dropped        : {dropped}  ({drop_pct:.1f}%)')
    print(f'  Duplicates     : {duplicates}')
    print(f'  Wrong size     : {bad_size}')
    print(f'  Out-of-order   : {out_of_order}')
    if rtts:
        rtts_sorted = sorted(rtts)
        def pct(p):
            k = max(0, min(len(rtts_sorted) - 1, int(round(p / 100 * (len(rtts_sorted) - 1)))))
            return rtts_sorted[k]
        print(f'  RTT min/p50/p95/max : '
              f'{min(rtts):6.1f} / {pct(50):6.1f} / {pct(95):6.1f} / {max(rtts):6.1f}  ms')
        print(f'  RTT mean +/- stdev  : '
              f'{statistics.mean(rtts):6.1f} +/- '
              f'{statistics.pstdev(rtts):5.1f}  ms')
        # Effective throughput based on payload bytes only (one direction).
        bytes_one_way = received * args.size
        print(f'  Throughput     : {bytes_one_way / send_elapsed:.0f} B/s '
              f'one-way payload (request+reply doubles wire usage)')
    print('=' * 70)

    if dropped == 0:
        print('PASS — no drops.')
        sys.exit(0)
    else:
        print(f'FAIL — {dropped}/{sent} echoes lost.')
        sys.exit(1)


if __name__ == '__main__':
    main()
