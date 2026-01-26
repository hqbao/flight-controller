#!/usr/bin/env python3
"""
Configure U-BLOX ZED-F9P to output all necessary UBX messages
"""

import struct
import time
import serial
import json
import os

# UBX Constants
UBX_SYNC_CHAR1 = 0xB5
UBX_SYNC_CHAR2 = 0x62

UBX_CLASS_NAV = 0x01
UBX_CLASS_CFG = 0x06

UBX_NAV_PVT = 0x07
UBX_NAV_SAT = 0x35
UBX_NAV_DOP = 0x04

UBX_CFG_MSG = 0x01
UBX_CFG_CFG = 0x09

CONFIG_FILE = os.path.expanduser("~/.ubx_reader_config.json")

def send_ubx_command(ser, msg_class, msg_id, payload=b''):
    """Send UBX command to GPS"""
    # Calculate checksum
    ck_a = 0
    ck_b = 0
    
    for b in [msg_class, msg_id, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    
    # Build message
    msg = bytearray([UBX_SYNC_CHAR1, UBX_SYNC_CHAR2, msg_class, msg_id, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    msg.extend(payload)
    msg.extend([ck_a, ck_b])
    
    ser.write(msg)
    ser.flush()

def main():
    # Load saved config
    port = None
    baudrate = None
    
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r') as f:
                config = json.load(f)
                port = config.get('port')
                baudrate = config.get('baudrate')
    except:
        pass
    
    if not port:
        print("No saved GPS config found. Please run ubx_reader.py first to detect GPS.")
        return 1
    
    print(f"Configuring GPS on {port} @ {baudrate} baud...")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1.0)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return 1
    
    print("\nEnabling UBX messages...")
    
    # Enable NAV-PVT at 1 Hz
    print("  - NAV-PVT (Position/Velocity/Time)")
    payload = struct.pack('<BBB', UBX_CLASS_NAV, UBX_NAV_PVT, 1)
    send_ubx_command(ser, UBX_CLASS_CFG, UBX_CFG_MSG, payload)
    time.sleep(0.2)
    
    # Enable NAV-SAT at 1 Hz
    print("  - NAV-SAT (Satellite Information)")
    payload = struct.pack('<BBB', UBX_CLASS_NAV, UBX_NAV_SAT, 1)
    send_ubx_command(ser, UBX_CLASS_CFG, UBX_CFG_MSG, payload)
    time.sleep(0.2)
    
    # Enable NAV-DOP at 1 Hz
    print("  - NAV-DOP (Dilution of Precision)")
    payload = struct.pack('<BBB', UBX_CLASS_NAV, UBX_NAV_DOP, 1)
    send_ubx_command(ser, UBX_CLASS_CFG, UBX_CFG_MSG, payload)
    time.sleep(0.2)
    
    print("\nSaving configuration to GPS flash memory...")
    payload = struct.pack('<IIIB',
        0x00000000,  # clearMask
        0x0000061F,  # saveMask
        0x00000000,  # loadMask  
        0x17         # deviceMask
    )
    send_ubx_command(ser, UBX_CLASS_CFG, UBX_CFG_CFG, payload)
    time.sleep(1)
    
    ser.close()
    
    print("\n✓ GPS configured successfully!")
    print("All UBX messages are now enabled.")
    print("\nYou can now run: python3 ubx_reader.py")
    
    return 0

if __name__ == "__main__":
    exit(main())
