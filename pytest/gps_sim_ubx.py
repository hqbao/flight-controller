#!/usr/bin/env python3
"""
GPS Simulator - UBX Protocol
Generates and sends UBX NAV-POSLLH and NAV-VELNED messages via UART
"""

import struct
import time
import serial.tools.list_ports
from serial import Serial
import math

# UBX Protocol Constants
UBX_SYNC_CHAR1 = 0xB5
UBX_SYNC_CHAR2 = 0x62

# UBX NAV Class
UBX_CLASS_NAV = 0x01
UBX_NAV_PVT = 0x07     # Position Velocity Time Solution (modern, combined message)
UBX_NAV_SAT = 0x35     # Satellite Information
UBX_NAV_DOP = 0x04     # Dilution of Precision

# Serial port configuration
BAUD_RATE = 9600
SERIAL_PORT = None


def calculate_ubx_checksum(msg_class, msg_id, payload):
    """Calculate UBX checksum (Fletcher-like algorithm)"""
    ck_a = 0
    ck_b = 0
    
    # Checksum over class, id, length, and payload
    ck_a = (ck_a + msg_class) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    ck_a = (ck_a + msg_id) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    length = len(payload)
    ck_a = (ck_a + (length & 0xFF)) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    ck_a = (ck_a + ((length >> 8) & 0xFF)) & 0xFF
    ck_b = (ck_b + ck_a) & 0xFF
    
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    
    return ck_a, ck_b


def create_ubx_message(msg_class, msg_id, payload):
    """Create a complete UBX message with header and checksum"""
    length = len(payload)
    ck_a, ck_b = calculate_ubx_checksum(msg_class, msg_id, payload)
    
    message = bytearray()
    message.append(UBX_SYNC_CHAR1)
    message.append(UBX_SYNC_CHAR2)
    message.append(msg_class)
    message.append(msg_id)
    message.append(length & 0xFF)
    message.append((length >> 8) & 0xFF)
    message.extend(payload)
    message.append(ck_a)
    message.append(ck_b)
    
    return bytes(message)


def create_nav_pvt(iTOW, year, month, day, hour, minute, second, 
                   lon, lat, height, hMSL, hAcc, vAcc,
                   velN, velE, velD, gSpeed, headMot, sAcc, headAcc, pDOP, numSV, fixType):
    """
    Create UBX NAV-PVT message (92 bytes payload minimum)
    
    Args:
        iTOW: GPS time of week (ms)
        year, month, day, hour, minute, second: UTC time
        lon: Longitude (degrees * 1e7)
        lat: Latitude (degrees * 1e7)
        height: Height above ellipsoid (mm)
        hMSL: Height above mean sea level (mm)
        hAcc: Horizontal accuracy estimate (mm)
        vAcc: Vertical accuracy estimate (mm)
        velN, velE, velD: NED velocity components (mm/s)
        gSpeed: Ground speed (2D) (mm/s)
        headMot: Heading of motion (degrees * 1e5)
        sAcc: Speed accuracy estimate (mm/s)
        headAcc: Heading accuracy estimate (degrees * 1e5)
        pDOP: Position DOP (0.01 scale)
        numSV: Number of satellites used
        fixType: GPS fix type (0=no fix, 2=2D, 3=3D, 4=GNSS+dead reckoning, 5=time only)
    """
    # Build 92-byte payload
    payload = bytearray(92)
    
    # bytes 0-3: iTOW
    struct.pack_into('<I', payload, 0, iTOW)
    
    # bytes 4-11: Date and time
    struct.pack_into('<H', payload, 4, year)      # year (u2)
    payload[6] = month                             # month (u1)
    payload[7] = day                               # day (u1)
    payload[8] = hour                              # hour (u1)
    payload[9] = minute                            # min (u1)
    payload[10] = second                           # sec (u1)
    payload[11] = 0x07                             # valid flags: validDate(bit0) | validTime(bit1) | fullyResolved(bit2)
    
    # bytes 12-15: Time accuracy
    struct.pack_into('<I', payload, 12, 1000)     # tAcc (ns)
    
    # bytes 16-19: Nanoseconds and leap seconds
    struct.pack_into('<i', payload, 16, 0)        # nano (ns)
    
    # bytes 20-23: Fix info
    payload[20] = fixType                          # fixType (0-5)
    payload[21] = 0x01                             # flags: gnssFixOK
    payload[22] = 0x00                             # flags2
    payload[23] = numSV                            # numSV
    
    # bytes 24-31: Position
    struct.pack_into('<ii', payload, 24, lon, lat)
    
    # bytes 32-39: Height
    struct.pack_into('<ii', payload, 32, height, hMSL)
    
    # bytes 40-47: Accuracy
    struct.pack_into('<II', payload, 40, hAcc, vAcc)
    
    # bytes 48-63: Velocity
    struct.pack_into('<iiii', payload, 48, velN, velE, velD, gSpeed)
    
    # bytes 64-67: Heading
    struct.pack_into('<i', payload, 64, headMot)
    
    # bytes 68-75: Accuracy
    struct.pack_into('<II', payload, 68, sAcc, headAcc)
    
    # bytes 76-77: pDOP
    struct.pack_into('<H', payload, 76, pDOP)
    
    # bytes 78-83: Reserved
    # (already zeros from bytearray initialization)
    
    # bytes 84-87: headVeh (degrees * 1e5) - use headMot
    struct.pack_into('<i', payload, 84, headMot)
    
    # bytes 88-89: magDec (degrees * 1e2)
    struct.pack_into('<h', payload, 88, 0)
    
    # bytes 90-91: magAcc (degrees * 1e2)
    struct.pack_into('<H', payload, 90, 18000)  # 180 degrees accuracy
    
    return create_ubx_message(UBX_CLASS_NAV, UBX_NAV_PVT, bytes(payload))


def create_nav_dop(iTOW, gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP):
    """
    Create UBX NAV-DOP message (18 bytes payload)
    
    Args:
        iTOW: GPS time of week (ms)
        gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP: DOP values (0.01 scale)
    """
    payload = struct.pack('<I7H',
                         iTOW,      # GPS time of week (ms)
                         gDOP,      # Geometric DOP
                         pDOP,      # Position DOP
                         tDOP,      # Time DOP
                         vDOP,      # Vertical DOP
                         hDOP,      # Horizontal DOP
                         nDOP,      # Northing DOP
                         eDOP)      # Easting DOP
    
    return create_ubx_message(UBX_CLASS_NAV, UBX_NAV_DOP, payload)


def create_nav_sat(iTOW, numSvs, satellites):
    """
    Create UBX NAV-SAT message (variable length)
    
    Args:
        iTOW: GPS time of week (ms)
        numSvs: Number of satellites
        satellites: List of dicts with keys: gnssId, svId, cno, elev, azim, prRes, flags
    """
    # Header: 8 bytes
    payload = bytearray()
    payload.extend(struct.pack('<IBBxx', iTOW, 1, numSvs))  # iTOW, version, numSvs, reserved
    
    # Each satellite: 12 bytes
    for sat in satellites[:numSvs]:
        payload.extend(struct.pack('<BBBbhhI',
                                  sat.get('gnssId', 0),     # GNSS identifier (u1)
                                  sat.get('svId', 1),       # Satellite identifier (u1)
                                  sat.get('cno', 35),       # Carrier-to-noise density (u1, dB-Hz)
                                  sat.get('elev', 45),      # Elevation (i1, degrees)
                                  sat.get('azim', 0),       # Azimuth (i2, degrees)
                                  sat.get('prRes', 0),      # Pseudorange residual (i2, 0.1 m)
                                  sat.get('flags', 0x0F)))  # Flags (u4, quality/health/etc)
    
    return create_ubx_message(UBX_CLASS_NAV, UBX_NAV_SAT, bytes(payload))


def find_serial_port():
    """Auto-detect USB serial port"""
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print(f"{port}: {desc} [{hwid}]")
        if port.startswith('/dev/cu.usbmodem') or \
           port.startswith('/dev/cu.usbserial') or \
           port.startswith('/dev/cu.SLAB_USBtoUART') or \
           port.startswith('/dev/ttyUSB') or \
           port.startswith('/dev/ttyACM'):
            return port
    return None


def simulate_circular_path(t, radius=100.0, period=60.0):
    """
    Simulate a circular flight path
    
    Args:
        t: Time in seconds
        radius: Circle radius in meters
        period: Period of one circle in seconds
    
    Returns:
        (lat, lon, alt, velN, velE, velD, heading)
    """
    # Center position (San Francisco)
    center_lat = 37.7749  # degrees
    center_lon = -122.4194  # degrees
    altitude = 100.0  # meters
    
    # Angular position
    angle = (2 * math.pi * t) / period
    
    # Convert radius to degrees (approximate)
    # 1 degree latitude ≈ 111 km
    lat_offset = (radius * math.cos(angle)) / 111000.0
    lon_offset = (radius * math.sin(angle)) / (111000.0 * math.cos(math.radians(center_lat)))
    
    lat = center_lat + lat_offset
    lon = center_lon + lon_offset
    alt = altitude
    
    # Calculate velocity (tangent to circle)
    v_magnitude = (2 * math.pi * radius) / period  # m/s
    velN = -v_magnitude * math.sin(angle) * 100  # cm/s
    velE = v_magnitude * math.cos(angle) * 100   # cm/s
    velD = 0  # cm/s
    
    # Heading (direction of motion)
    heading = math.degrees(angle + math.pi/2) % 360
    
    return lat, lon, alt, velN, velE, velD, heading


def main():
    """Main simulation loop"""
    global SERIAL_PORT
    
    # Find serial port
    SERIAL_PORT = find_serial_port()
    if SERIAL_PORT is None:
        print("No serial port found. Running in simulation mode (no output).")
        serial_conn = None
    else:
        print(f"Using serial port: {SERIAL_PORT} at {BAUD_RATE} baud")
        try:
            serial_conn = Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)  # Wait for connection to stabilize
        except Exception as e:
            print(f"Failed to open serial port: {e}")
            serial_conn = None
    
    print("\nGPS Simulator - UBX Protocol")
    print("Generating NAV-PVT, NAV-SAT, and NAV-DOP messages")
    print("Press Ctrl+C to stop\n")
    
    start_time = time.time()
    msg_count = 0
    
    try:
        while True:
            current_time = time.time() - start_time
            iTOW = int(current_time * 1000)  # milliseconds
            
            # Get current UTC time
            import datetime
            now = datetime.datetime.utcnow()
            
            # Simulate circular path
            lat, lon, alt, velN, velE, velD, heading = simulate_circular_path(current_time)
            
            # Convert to UBX format
            lat_e7 = int(lat * 1e7)      # degrees * 1e7
            lon_e7 = int(lon * 1e7)      # degrees * 1e7
            height_mm = int(alt * 1000)  # mm
            hMSL_mm = int(alt * 1000)    # mm
            hAcc_mm = 5000               # 5m horizontal accuracy
            vAcc_mm = 10000              # 10m vertical accuracy
            
            # Velocity in mm/s (was cm/s, now need mm/s)
            velN_mms = int(velN * 10)    # cm/s -> mm/s
            velE_mms = int(velE * 10)    # cm/s -> mm/s
            velD_mms = int(velD * 10)    # cm/s -> mm/s
            speed_mms = int(math.sqrt(velN_mms**2 + velE_mms**2 + velD_mms**2))
            gSpeed_mms = int(math.sqrt(velN_mms**2 + velE_mms**2))
            headMot_e5 = int(heading * 1e5)  # degrees * 1e5
            sAcc_mms = 500               # 0.5 m/s speed accuracy (mm/s)
            headAcc_e5 = int(5.0 * 1e5)  # 5 degree heading accuracy
            pDOP = 150                   # 1.5 DOP value (0.01 scale)
            numSV = 12                   # Number of satellites
            fixType = 3                  # 3D fix
            
            # Create NAV-PVT message (combines position and velocity)
            pvt_msg = create_nav_pvt(
                iTOW, now.year, now.month, now.day, now.hour, now.minute, now.second,
                lon_e7, lat_e7, height_mm, hMSL_mm, hAcc_mm, vAcc_mm,
                velN_mms, velE_mms, velD_mms, gSpeed_mms, headMot_e5, 
                sAcc_mms, headAcc_e5, pDOP, numSV, fixType
            )
            
            # Create NAV-DOP message
            gDOP = 180  # 1.8
            pDOP_val = 150  # 1.5
            tDOP = 90   # 0.9
            vDOP = 100  # 1.0
            hDOP = 120  # 1.2
            nDOP = 80   # 0.8
            eDOP = 85   # 0.85
            dop_msg = create_nav_dop(iTOW, gDOP, pDOP_val, tDOP, vDOP, hDOP, nDOP, eDOP)
            
            # Create NAV-SAT message with simulated satellites
            satellites = []
            for i in range(12):
                satellites.append({
                    'gnssId': 0,          # GPS
                    'svId': i + 1,        # PRN 1-12
                    'cno': 35 + (i % 10), # Signal strength 35-44 dB-Hz
                    'elev': 45 + (i * 5) % 45,  # Elevation 45-90 degrees
                    'azim': (i * 30) % 360,     # Azimuth evenly distributed
                    'prRes': 0,           # Pseudorange residual
                    'flags': 0x0F if i < 12 else 0x00  # Used in solution
                })
            sat_msg = create_nav_sat(iTOW, 12, satellites)
            
            # Send messages
            if serial_conn:
                serial_conn.write(pvt_msg)
                time.sleep(0.01)
                serial_conn.write(dop_msg)
                time.sleep(0.01)
                serial_conn.write(sat_msg)
            
            msg_count += 3
            
            # Print status
            if msg_count % 15 == 0:
                print(f"[{current_time:6.1f}s] Sent {msg_count} messages | "
                      f"Pos: {lat:.6f}°N, {lon:.6f}°E, {alt:.1f}m | "
                      f"Vel: N={velN/100:.2f} E={velE/100:.2f} m/s | "
                      f"Hdg: {heading:.1f}° | Sats: {numSV}")
            
            # Send at 10 Hz (every 100ms)
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print(f"\n\nSimulation stopped. Sent {msg_count} messages.")
    finally:
        if serial_conn:
            serial_conn.close()
            print("Serial port closed.")


if __name__ == "__main__":
    main()
