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
UBX_NAV_POSLLH = 0x02  # Geodetic Position Solution
UBX_NAV_VELNED = 0x12  # Velocity Solution in NED frame

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


def create_nav_posllh(iTOW, lon, lat, height, hMSL, hAcc, vAcc):
    """
    Create UBX NAV-POSLLH message (28 bytes payload)
    
    Args:
        iTOW: GPS time of week (ms)
        lon: Longitude (degrees * 1e7)
        lat: Latitude (degrees * 1e7)
        height: Height above ellipsoid (mm)
        hMSL: Height above mean sea level (mm)
        hAcc: Horizontal accuracy estimate (mm)
        vAcc: Vertical accuracy estimate (mm)
    """
    payload = struct.pack('<IiiiiII',
                         iTOW,      # GPS time of week (ms)
                         lon,       # Longitude (deg * 1e7)
                         lat,       # Latitude (deg * 1e7)
                         height,    # Height above ellipsoid (mm)
                         hMSL,      # Height above MSL (mm)
                         hAcc,      # Horizontal accuracy (mm)
                         vAcc)      # Vertical accuracy (mm)
    
    return create_ubx_message(UBX_CLASS_NAV, UBX_NAV_POSLLH, payload)


def create_nav_velned(iTOW, velN, velE, velD, speed, gSpeed, heading, sAcc, cAcc):
    """
    Create UBX NAV-VELNED message (36 bytes payload)
    
    Args:
        iTOW: GPS time of week (ms)
        velN: North velocity (cm/s)
        velE: East velocity (cm/s)
        velD: Down velocity (cm/s)
        speed: 3D speed (cm/s)
        gSpeed: Ground speed (cm/s)
        heading: Heading (degrees * 1e5)
        sAcc: Speed accuracy estimate (cm/s)
        cAcc: Course/heading accuracy (degrees * 1e5)
    """
    payload = struct.pack('<IiiiiiiII',
                         iTOW,      # GPS time of week (ms)
                         velN,      # North velocity (cm/s)
                         velE,      # East velocity (cm/s)
                         velD,      # Down velocity (cm/s)
                         speed,     # 3D speed (cm/s)
                         gSpeed,    # Ground speed (cm/s)
                         heading,   # Heading (deg * 1e5)
                         sAcc,      # Speed accuracy (cm/s)
                         cAcc)      # Course accuracy (deg * 1e5)
    
    return create_ubx_message(UBX_CLASS_NAV, UBX_NAV_VELNED, payload)


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
    print("Generating NAV-POSLLH and NAV-VELNED messages")
    print("Press Ctrl+C to stop\n")
    
    start_time = time.time()
    msg_count = 0
    
    try:
        while True:
            current_time = time.time() - start_time
            iTOW = int(current_time * 1000)  # milliseconds
            
            # Simulate circular path
            lat, lon, alt, velN, velE, velD, heading = simulate_circular_path(current_time)
            
            # Convert to UBX format
            lat_e7 = int(lat * 1e7)      # degrees * 1e7
            lon_e7 = int(lon * 1e7)      # degrees * 1e7
            height_mm = int(alt * 1000)  # mm
            hMSL_mm = int(alt * 1000)    # mm
            hAcc_mm = 5000               # 5m horizontal accuracy
            vAcc_mm = 10000              # 10m vertical accuracy
            
            velN_cms = int(velN)         # cm/s
            velE_cms = int(velE)         # cm/s
            velD_cms = int(velD)         # cm/s
            speed_cms = int(math.sqrt(velN**2 + velE**2 + velD**2))
            gSpeed_cms = int(math.sqrt(velN**2 + velE**2))
            heading_e5 = int(heading * 1e5)  # degrees * 1e5
            sAcc_cms = 50                # 0.5 m/s speed accuracy
            cAcc_e5 = int(5.0 * 1e5)     # 5 degree heading accuracy
            
            # Create UBX messages
            posllh_msg = create_nav_posllh(iTOW, lon_e7, lat_e7, height_mm, hMSL_mm, hAcc_mm, vAcc_mm)
            velned_msg = create_nav_velned(iTOW, velN_cms, velE_cms, velD_cms, 
                                          speed_cms, gSpeed_cms, heading_e5, 
                                          sAcc_cms, cAcc_e5)
            
            # Send messages
            if serial_conn:
                serial_conn.write(posllh_msg)
                time.sleep(0.01)  # Small delay between messages
                serial_conn.write(velned_msg)
            
            msg_count += 2
            
            # Print status
            if msg_count % 10 == 0:
                print(f"[{current_time:6.1f}s] Sent {msg_count} messages | "
                      f"Pos: {lat:.6f}°N, {lon:.6f}°E, {alt:.1f}m | "
                      f"Vel: N={velN/100:.2f} E={velE/100:.2f} m/s | "
                      f"Hdg: {heading:.1f}°")
            
            # Send at 5 Hz (every 200ms)
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        print(f"\n\nSimulation stopped. Sent {msg_count} messages.")
    finally:
        if serial_conn:
            serial_conn.close()
            print("Serial port closed.")


if __name__ == "__main__":
    main()
