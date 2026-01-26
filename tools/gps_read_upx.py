#!/usr/bin/env python3
"""
UBX GPS Reader for u-blox ZED-F9P
Reads and displays GPS data with matplotlib plotting UI
Connection settings are automatically saved for future use
"""

import struct
import time
import serial
import serial.tools.list_ports
from collections import deque
from datetime import datetime
import threading
import sys
import json
import os

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    from matplotlib.gridspec import GridSpec
    from matplotlib.widgets import TextBox, Button
    import numpy as np
    PLOT_AVAILABLE = True
except ImportError:
    PLOT_AVAILABLE = False
    print("Warning: matplotlib not available, install with: pip3 install matplotlib numpy")

# UBX Protocol Constants
UBX_SYNC_CHAR1 = 0xB5
UBX_SYNC_CHAR2 = 0x62
UBX_CLASS_NAV = 0x01

# UBX NAV Messages
UBX_NAV_PVT = 0x07       # Navigation Position Velocity Time Solution
UBX_NAV_SAT = 0x35       # Satellite Information
UBX_NAV_DOP = 0x04       # Dilution of Precision

# Config file
CONFIG_FILE = os.path.expanduser("~/.ubx_reader_config.json")

class UBXParser:
    """Parse UBX protocol messages"""
    
    def __init__(self):
        self.state = 'SYNC1'
        self.msg_class = 0
        self.msg_id = 0
        self.length = 0
        self.payload = bytearray()
        self.ck_a = 0
        self.ck_b = 0
        
    def parse_byte(self, byte):
        """Parse incoming byte, returns (msg_class, msg_id, payload) when complete"""
        
        if self.state == 'SYNC1':
            if byte == UBX_SYNC_CHAR1:
                self.state = 'SYNC2'
        elif self.state == 'SYNC2':
            if byte == UBX_SYNC_CHAR2:
                self.state = 'CLASS'
            else:
                self.state = 'SYNC1'
        elif self.state == 'CLASS':
            self.msg_class = byte
            self.state = 'ID'
        elif self.state == 'ID':
            self.msg_id = byte
            self.state = 'LENGTH1'
        elif self.state == 'LENGTH1':
            self.length = byte
            self.state = 'LENGTH2'
        elif self.state == 'LENGTH2':
            self.length |= (byte << 8)
            self.payload = bytearray()
            self.state = 'PAYLOAD' if self.length > 0 else 'CK_A'
        elif self.state == 'PAYLOAD':
            self.payload.append(byte)
            if len(self.payload) >= self.length:
                self.state = 'CK_A'
        elif self.state == 'CK_A':
            self.ck_a = byte
            self.state = 'CK_B'
        elif self.state == 'CK_B':
            self.ck_b = byte
            self.state = 'SYNC1'
            
            # Verify checksum
            calc_ck_a, calc_ck_b = self.calculate_checksum(
                self.msg_class, self.msg_id, self.length, self.payload
            )
            
            if calc_ck_a == self.ck_a and calc_ck_b == self.ck_b:
                result = (self.msg_class, self.msg_id, bytes(self.payload))
                return result
                
        return None
    
    def calculate_checksum(self, msg_class, msg_id, length, payload):
        """Calculate UBX checksum"""
        ck_a = ck_b = 0
        for b in [msg_class, msg_id, length & 0xFF, (length >> 8) & 0xFF]:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        for byte in payload:
            ck_a = (ck_a + byte) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return ck_a, ck_b


class GPSData:
    """Store GPS data"""
    
    def __init__(self):
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude_msl = 0.0
        self.altitude_wgs84 = 0.0
        self.vel_north = 0.0
        self.vel_east = 0.0
        self.vel_down = 0.0
        self.ground_speed = 0.0
        self.heading = 0.0
        self.h_acc = 0.0
        self.v_acc = 0.0
        self.speed_acc = 0.0
        self.heading_acc = 0.0
        self.num_satellites = 0
        self.num_satellites_tracked = 0
        self.satellites = []
        self.pdop = 0.0
        self.hdop = 0.0
        self.vdop = 0.0
        self.fix_type = 0
        self.carrier_solution = 0
        self.time_utc = None
        self.time_valid = False
        self.last_update = time.time()
        self.update_rate = 0.0
        self.last_update_time = 0.0
        # Drift measurement (continuous)
        self.drift_origin_lat = 0.0
        self.drift_origin_lon = 0.0
        self.drift_origin_set = False
        self.drift_window = 5.0  # seconds
        self.drift_positions = deque(maxlen=1000)  # Store (timestamp, lat, lon)
        self.drift_current = 0.0
        self.drift_max = 0.0


class GPSReader:
    """Read GPS data from serial port"""
    
    def __init__(self):
        self.serial_port = None
        self.parser = UBXParser()
        self.gps_data = GPSData()
        self.running = False
        self.thread = None
        self.message_count = 0
        self.last_rate_time = time.time()
        self.messages_per_second = 0.0
    
    def load_config(self):
        """Load saved connection settings"""
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r') as f:
                    config = json.load(f)
                    return config.get('port'), config.get('baudrate')
        except:
            pass
        return None, None
    
    def save_config(self, port, baudrate):
        """Save connection settings"""
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump({'port': port, 'baudrate': baudrate}, f)
            print(f"Saved connection to {CONFIG_FILE}")
        except:
            pass
    
    def test_port(self, port, baudrate):
        """Test if port has valid UBX data"""
        try:
            ser = serial.Serial(port, baudrate, timeout=2.0)
            parser = UBXParser()
            start_time = time.time()
            valid_messages = 0
            
            while time.time() - start_time < 2.0:
                if ser.in_waiting:
                    byte = ser.read(1)[0]
                    if parser.parse_byte(byte):
                        valid_messages += 1
                        if valid_messages >= 2:
                            ser.close()
                            print(" ✓ GPS found!")
                            return True
            ser.close()
            print(" (no data)")
        except Exception as e:
            print(f" (error)")
        return False
    
    def auto_detect(self):
        """Auto-detect GPS port and baudrate"""
        print("Scanning for GPS...")
        
        # Try saved config first
        saved_port, saved_baud = self.load_config()
        if saved_port and saved_baud:
            print(f"Trying saved: {saved_port} @ {saved_baud} baud...", end='', flush=True)
            if self.test_port(saved_port, saved_baud):
                return saved_port, saved_baud
            print("  (saved config didn't work)")
        
        # Scan ports
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print("No serial ports found!")
            return None, None
        
        print(f"Found {len(ports)} port(s):")
        for p in ports:
            print(f"  {p.device}: {p.description}")
        
        for p in ports:
            for baud in [38400, 115200, 9600, 57600, 230400]:
                print(f"Trying {p.device} @ {baud}...", end='', flush=True)
                if self.test_port(p.device, baud):
                    return p.device, baud
        
        return None, None
    
    def connect(self):
        """Connect to GPS"""
        port, baudrate = self.auto_detect()
        if not port:
            return False
        
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1.0)
            print(f"Connected to {port} @ {baudrate} baud")
            self.save_config(port, baudrate)
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
    
    def start(self):
        """Start reading in background thread"""
        if not self.serial_port:
            return False
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        return True
    
    def stop(self):
        """Stop reading"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.serial_port:
            self.serial_port.close()
    
    def _read_loop(self):
        """Background thread to read serial data"""
        while self.running:
            try:
                if self.serial_port.in_waiting:
                    byte = self.serial_port.read(1)[0]
                    msg = self.parser.parse_byte(byte)
                    if msg:
                        self.message_count += 1
                        self._process_message(msg)
                        
                        now = time.time()
                        if now - self.last_rate_time >= 1.0:
                            self.messages_per_second = self.message_count / (now - self.last_rate_time)
                            self.message_count = 0
                            self.last_rate_time = now
                else:
                    time.sleep(0.001)
            except Exception as e:
                print(f"Read error: {e}")
                break
    
    def _process_message(self, msg):
        """Process UBX message"""
        msg_class, msg_id, payload = msg
        
        if msg_class == UBX_CLASS_NAV:
            if msg_id == UBX_NAV_PVT:
                self._parse_nav_pvt(payload)
            elif msg_id == UBX_NAV_DOP:
                self._parse_nav_dop(payload)
            elif msg_id == UBX_NAV_SAT:
                self._parse_nav_sat(payload)
    
    def _parse_nav_pvt(self, payload):
        """Parse NAV-PVT message"""
        # NAV-PVT is 92 bytes in protocol 15+
        if len(payload) < 84:
            return
        
        try:
            # Use simple unpacking for each field
            iTOW = struct.unpack('<I', payload[0:4])[0]
            year, month, day = struct.unpack('<HBB', payload[4:8])
            hour, minute, second, valid = struct.unpack('<BBBB', payload[8:12])
            
            fixType = payload[20]
            flags = payload[21]
            numSV = payload[23]
            
            lon, lat = struct.unpack('<ii', payload[24:32])
            height, hMSL = struct.unpack('<ii', payload[32:40])
            hAcc, vAcc = struct.unpack('<II', payload[40:48])
            
            velN, velE, velD, gSpeed = struct.unpack('<iiii', payload[48:64])
            headMot = struct.unpack('<i', payload[64:68])[0]
            sAcc = struct.unpack('<I', payload[68:72])[0]
            headAcc = struct.unpack('<I', payload[72:76])[0]
            pDOP = struct.unpack('<H', payload[76:78])[0]
            
            # Update GPS data
            self.gps_data.longitude = lon * 1e-7
            self.gps_data.latitude = lat * 1e-7
            self.gps_data.altitude_msl = hMSL * 1e-3
            self.gps_data.altitude_wgs84 = height * 1e-3
            self.gps_data.h_acc = hAcc * 1e-3
            self.gps_data.v_acc = vAcc * 1e-3
            self.gps_data.vel_north = velN * 1e-3
            self.gps_data.vel_east = velE * 1e-3
            self.gps_data.vel_down = velD * 1e-3
            self.gps_data.ground_speed = gSpeed * 1e-3
            self.gps_data.heading = headMot * 1e-5
            self.gps_data.speed_acc = sAcc * 1e-3
            self.gps_data.heading_acc = headAcc * 1e-5
            self.gps_data.fix_type = fixType
            self.gps_data.num_satellites = numSV
            self.gps_data.pdop = pDOP * 0.01
            self.gps_data.carrier_solution = (flags >> 6) & 0x03
            
            if valid & 0x03:
                try:
                    self.gps_data.time_utc = datetime(year, month, day, hour, minute, second)
                    self.gps_data.time_valid = True
                except:
                    pass
            
            # Store position for drift calculation
            now = time.time()
            self.gps_data.drift_positions.append((now, self.gps_data.latitude, self.gps_data.longitude))
            
            # Set initial drift origin if not set
            if not self.gps_data.drift_origin_set and self.gps_data.latitude != 0:
                self.gps_data.drift_origin_lat = self.gps_data.latitude
                self.gps_data.drift_origin_lon = self.gps_data.longitude
                self.gps_data.drift_origin_set = True
            
            self._update_timestamp(update_rate=True)  # NAV-PVT determines the update rate
        except Exception as e:
            pass  # Silently ignore parse errors now that GPS is configured
    
    def _parse_nav_dop(self, payload):
        """Parse NAV-DOP message"""
        if len(payload) < 18:
            return
        
        try:
            data = struct.unpack('<I7H', payload[:18])
            self.gps_data.pdop = data[2] * 0.01
            self.gps_data.hdop = data[5] * 0.01
            self.gps_data.vdop = data[4] * 0.01
            self._update_timestamp(update_rate=False)  # Don't count DOP messages in rate
        except:
            pass
    
    def _parse_nav_sat(self, payload):
        """Parse NAV-SAT message"""
        if len(payload) < 8:
            return
        
        try:
            # Header: iTOW (4), version (1), numSvs (1), reserved (2)
            header = struct.unpack('<IBBxx', payload[:8])
            numSvs = header[2]
            satellites = []
            offset = 8
            
            for i in range(numSvs):
                if offset + 12 > len(payload):
                    break
                
                gnssId, svId, cno, elev, azim, prRes, flags = struct.unpack('<BBBbhhI', payload[offset:offset+12])
                
                satellites.append({
                    'svid': svId,
                    'gnss': gnssId,
                    'cno': cno,
                    'elevation': elev,
                    'azimuth': azim,
                    'used': (flags & 0x08) != 0
                })
                
                offset += 12
            
            self.gps_data.satellites = satellites
            self.gps_data.num_satellites_tracked = len(satellites)
            self.gps_data.num_satellites = sum(1 for sat in satellites if sat['used'])
            self._update_timestamp(update_rate=False)  # Don't count SAT messages in rate
        except Exception as e:
            pass  # Silently ignore parse errors now that GPS is configured
    
    def _update_timestamp(self, update_rate=True):
        """Update timestamp and optionally the rate"""
        now = time.time()
        if update_rate:
            dt = now - self.gps_data.last_update_time
            if dt > 0:
                self.gps_data.update_rate = 1.0 / dt
        self.gps_data.last_update = now
        self.gps_data.last_update_time = now


def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance in meters between two GPS coordinates"""
    R = 6371000  # Earth radius in meters
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    dphi = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)
    
    a = np.sin(dphi/2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    
    return R * c


class PlotUI:
    """Matplotlib plotting UI"""
    
    def __init__(self, gps_reader, history_size=300):
        self.gps_reader = gps_reader
        self.history_size = history_size
        
        # History buffers
        self.time_history = deque(maxlen=history_size)
        self.lat_history = deque(maxlen=history_size)
        self.lon_history = deque(maxlen=history_size)
        self.alt_history = deque(maxlen=history_size)
        self.speed_history = deque(maxlen=history_size)
        self.heading_history = deque(maxlen=history_size)
        self.sat_history = deque(maxlen=history_size)
        self.hdop_history = deque(maxlen=history_size)
        self.vdop_history = deque(maxlen=history_size)
        
        # Create figure (sized for 14" MacBook Pro)
        self.fig = plt.figure(figsize=(13, 8))
        self.fig.canvas.manager.set_window_title('U-BLOX ZED-F9P GPS Monitor')
        
        gs = GridSpec(3, 3, figure=self.fig, hspace=0.3, wspace=0.3)
        
        # Position map
        self.ax_map = self.fig.add_subplot(gs[0:2, 0])
        self.ax_map.set_title('Position Track')
        self.ax_map.set_xlabel('Longitude (°)')
        self.ax_map.set_ylabel('Latitude (°)')
        self.ax_map.grid(True, alpha=0.3)
        
        # Altitude
        self.ax_alt = self.fig.add_subplot(gs[0, 1])
        self.ax_alt.set_title('Altitude')
        self.ax_alt.set_ylabel('Altitude (m)')
        self.ax_alt.grid(True, alpha=0.3)
        
        # Speed
        self.ax_speed = self.fig.add_subplot(gs[0, 2])
        self.ax_speed.set_title('Ground Speed')
        self.ax_speed.set_ylabel('Speed (m/s)')
        self.ax_speed.grid(True, alpha=0.3)
        
        # Heading
        self.ax_heading = self.fig.add_subplot(gs[1, 1])
        self.ax_heading.set_title('Heading')
        self.ax_heading.set_ylabel('Heading (°)')
        self.ax_heading.set_ylim(0, 360)
        self.ax_heading.grid(True, alpha=0.3)
        
        # Satellites
        self.ax_sats = self.fig.add_subplot(gs[1, 2])
        self.ax_sats.set_title('Satellites')
        self.ax_sats.set_ylabel('Count')
        self.ax_sats.grid(True, alpha=0.3)
        
        # DOP
        self.ax_dop = self.fig.add_subplot(gs[2, 0])
        self.ax_dop.set_title('DOP (Dilution of Precision)')
        self.ax_dop.set_xlabel('Time (s)')
        self.ax_dop.set_ylabel('DOP')
        self.ax_dop.grid(True, alpha=0.3)
        
        # Signal strength
        self.ax_signal = self.fig.add_subplot(gs[2, 1:])
        self.ax_signal.set_title('Satellite Signal Strength')
        self.ax_signal.set_xlabel('Satellite')
        self.ax_signal.set_ylabel('C/N0 (dBHz)')
        self.ax_signal.grid(True, alpha=0.3)
        
        # Status text
        self.status_text = self.fig.text(0.02, 0.98, '', 
                                         transform=self.fig.transFigure,
                                         verticalalignment='top',
                                         fontfamily='monospace',
                                         fontsize=9)
        
        # Drift measurement controls
        ax_duration = plt.axes([0.15, 0.02, 0.08, 0.04])
        self.duration_box = TextBox(ax_duration, 'Window (s):', initial='5.0')
        self.duration_box.on_submit(self._on_duration_change)
        
        ax_button = plt.axes([0.25, 0.02, 0.12, 0.04])
        self.measure_button = Button(ax_button, 'Reset Origin')
        self.measure_button.on_clicked(self._reset_drift_origin)
        
        ax_reset = plt.axes([0.05, 0.02, 0.08, 0.04])
        self.reset_button = Button(ax_reset, 'Reset Data')
        self.reset_button.on_clicked(self._reset_data)
        
        self.drift_text = self.fig.text(0.40, 0.035, '', 
                                       transform=self.fig.transFigure,
                                       verticalalignment='center',
                                       fontfamily='monospace',
                                       fontsize=9,
                                       color='red')
        
        self.start_time = time.time()
        
    def _on_duration_change(self, text):
        """Handle window duration input change"""
        try:
            duration = float(text)
            if duration > 0:
                self.gps_reader.gps_data.drift_window = duration
        except:
            pass
    
    def _reset_drift_origin(self, event):
        """Reset drift origin to current position"""
        gps = self.gps_reader.gps_data
        gps.drift_origin_lat = gps.latitude
        gps.drift_origin_lon = gps.longitude
        gps.drift_origin_set = True
        gps.drift_positions.clear()
        gps.drift_max = 0.0
        gps.drift_current = 0.0
    
    def _reset_data(self, event):
        """Reset all collected data and refresh charts"""
        # Clear all history buffers
        self.time_history.clear()
        self.lat_history.clear()
        self.lon_history.clear()
        self.alt_history.clear()
        self.speed_history.clear()
        self.heading_history.clear()
        self.sat_history.clear()
        self.hdop_history.clear()
        self.vdop_history.clear()
        
        # Reset start time for relative time tracking
        self.start_time = time.time()
        
        # Reset drift measurement
        gps = self.gps_reader.gps_data
        gps.drift_positions.clear()
        gps.drift_max = 0.0
        gps.drift_current = 0.0
    
    def update(self, frame):
        """Update plots"""
        gps = self.gps_reader.gps_data
        
        # Update real-time drift measurement
        if gps.drift_origin_set and len(gps.drift_positions) > 0:
            # Calculate current drift from origin
            gps.drift_current = haversine_distance(
                gps.drift_origin_lat, gps.drift_origin_lon,
                gps.latitude, gps.longitude
            )
            
            # Calculate max drift within the time window
            now = time.time()
            cutoff_time = now - gps.drift_window
            
            # Filter positions within window
            window_positions = [(t, lat, lon) for t, lat, lon in gps.drift_positions if t >= cutoff_time]
            
            if window_positions:
                # Calculate max drift in window
                drifts = [haversine_distance(gps.drift_origin_lat, gps.drift_origin_lon, lat, lon) 
                         for _, lat, lon in window_positions]
                gps.drift_max = max(drifts) if drifts else 0.0
                
                window_duration = now - window_positions[0][0]
                self.drift_text.set_text(
                    f'Drift (real-time) | Current: {gps.drift_current*100:.1f} cm | '
                    f'Max in {window_duration:.1f}s window: {gps.drift_max*100:.1f} cm'
                )
            else:
                self.drift_text.set_text(
                    f'Drift (real-time) | Current: {gps.drift_current*100:.1f} cm'
                )
        else:
            self.drift_text.set_text('Waiting for GPS fix...')
        
        # Add to history
        current_time = time.time() - self.start_time
        self.time_history.append(current_time)
        self.lat_history.append(gps.latitude)
        self.lon_history.append(gps.longitude)
        self.alt_history.append(gps.altitude_msl)
        self.speed_history.append(gps.ground_speed)
        self.heading_history.append(gps.heading)
        self.sat_history.append(gps.num_satellites)
        self.hdop_history.append(gps.hdop)
        self.vdop_history.append(gps.vdop)
        
        # Update position map
        self.ax_map.clear()
        self.ax_map.set_title('Position Track')
        self.ax_map.set_xlabel('Longitude (°)')
        self.ax_map.set_ylabel('Latitude (°)')
        self.ax_map.grid(True, alpha=0.3)
        if len(self.lat_history) > 1:
            self.ax_map.plot(self.lon_history, self.lat_history, 'b-', linewidth=1, alpha=0.6)
            self.ax_map.plot(self.lon_history[-1], self.lat_history[-1], 'ro', markersize=8)
            if gps.ground_speed > 0.5:
                dx = 0.00001 * np.sin(np.radians(gps.heading))
                dy = 0.00001 * np.cos(np.radians(gps.heading))
                self.ax_map.arrow(self.lon_history[-1], self.lat_history[-1], 
                                dx, dy, head_width=0.00001, head_length=0.00001, fc='r', ec='r')
        self.ax_map.set_aspect('equal', adjustable='box')
        
        # Update altitude
        self.ax_alt.clear()
        self.ax_alt.set_title('Altitude')
        self.ax_alt.set_ylabel('Altitude (m)')
        self.ax_alt.grid(True, alpha=0.3)
        if len(self.time_history) > 0:
            self.ax_alt.plot(self.time_history, self.alt_history, 'g-', linewidth=2)
        
        # Update speed
        self.ax_speed.clear()
        self.ax_speed.set_title('Ground Speed')
        self.ax_speed.set_ylabel('Speed (m/s)')
        self.ax_speed.grid(True, alpha=0.3)
        if len(self.time_history) > 0:
            self.ax_speed.plot(self.time_history, self.speed_history, 'r-', linewidth=2)
        
        # Update heading
        self.ax_heading.clear()
        self.ax_heading.set_title('Heading')
        self.ax_heading.set_ylabel('Heading (°)')
        self.ax_heading.set_ylim(0, 360)
        self.ax_heading.grid(True, alpha=0.3)
        if len(self.time_history) > 0:
            self.ax_heading.plot(self.time_history, self.heading_history, 'm-', linewidth=2)
        
        # Update satellites
        self.ax_sats.clear()
        self.ax_sats.set_title('Satellites')
        self.ax_sats.set_ylabel('Count')
        self.ax_sats.grid(True, alpha=0.3)
        if len(self.time_history) > 0:
            self.ax_sats.plot(self.time_history, self.sat_history, 'c-', linewidth=2)
            self.ax_sats.fill_between(self.time_history, 0, self.sat_history, alpha=0.3)
        
        # Update DOP
        self.ax_dop.clear()
        self.ax_dop.set_title('DOP (Dilution of Precision)')
        self.ax_dop.set_xlabel('Time (s)')
        self.ax_dop.set_ylabel('DOP')
        self.ax_dop.grid(True, alpha=0.3)
        if len(self.time_history) > 0:
            self.ax_dop.plot(self.time_history, self.hdop_history, 'b-', linewidth=2, label='HDOP')
            self.ax_dop.plot(self.time_history, self.vdop_history, 'r-', linewidth=2, label='VDOP')
            self.ax_dop.legend(loc='upper right')
        
        # Update signal strength
        self.ax_signal.clear()
        self.ax_signal.set_title('Satellite Signal Strength')
        self.ax_signal.set_xlabel('Satellite')
        self.ax_signal.set_ylabel('C/N0 (dBHz)')
        self.ax_signal.grid(True, alpha=0.3)
        
        if gps.satellites:
            gnss_names = {0: "GPS", 1: "SBAS", 2: "Galileo", 3: "BeiDou", 5: "QZSS", 6: "GLONASS"}
            gnss_colors = {0: "blue", 1: "orange", 2: "green", 3: "red", 5: "purple", 6: "cyan"}
            
            sorted_sats = sorted(gps.satellites, key=lambda s: (s['gnss'], -s['cno']))
            
            labels, values, colors = [], [], []
            for sat in sorted_sats[:20]:
                gnss_name = gnss_names.get(sat['gnss'], f"G{sat['gnss']}")
                labels.append(f"{gnss_name[:3]}\n{sat['svid']}")
                values.append(sat['cno'])
                color = gnss_colors.get(sat['gnss'], 'gray')
                colors.append(color if sat['used'] else 'lightgray')
            
            if labels:
                self.ax_signal.bar(range(len(labels)), values, color=colors, alpha=0.8)
                self.ax_signal.set_xticks(range(len(labels)))
                self.ax_signal.set_xticklabels(labels, fontsize=7)
                self.ax_signal.axhline(y=30, color='orange', linestyle='--', alpha=0.5, label='Good (30 dBHz)')
                self.ax_signal.axhline(y=40, color='green', linestyle='--', alpha=0.5, label='Excellent (40 dBHz)')
                self.ax_signal.legend(loc='upper right', fontsize=8)
        
        # Update status
        fix_types = ["No Fix", "Dead Reckoning", "2D Fix", "3D Fix", "GNSS+DR", "Time Only"]
        fix_type_str = fix_types[gps.fix_type] if gps.fix_type < len(fix_types) else "Unknown"
        
        rtk_status = ""
        if gps.carrier_solution == 1:
            rtk_status = " | RTK Float"
        elif gps.carrier_solution == 2:
            rtk_status = " | RTK Fixed"
        
        time_str = ""
        if gps.time_valid and gps.time_utc:
            time_str = f"UTC: {gps.time_utc.strftime('%Y-%m-%d %H:%M:%S')}"
        
        status = f"""Fix: {fix_type_str}{rtk_status} | Sats: {gps.num_satellites}/{gps.num_satellites_tracked} (used/tracked) | Rate: {gps.update_rate:.1f} Hz
Position: {gps.latitude:.8f}°, {gps.longitude:.8f}° | Alt: {gps.altitude_msl:.2f} m | Acc: ±{gps.h_acc:.2f} m
Speed: {gps.ground_speed:.2f} m/s ({gps.ground_speed*3.6:.2f} km/h) | Heading: {gps.heading:.1f}°
PDOP: {gps.pdop:.2f} | HDOP: {gps.hdop:.2f} | VDOP: {gps.vdop:.2f} | {time_str}"""
        
        self.status_text.set_text(status)
        
    def start(self):
        """Start animation"""
        ani = animation.FuncAnimation(
            self.fig, 
            self.update, 
            interval=500,
            blit=False,
            cache_frame_data=False
        )
        plt.show()


def main():
    """Main program"""
    print("U-BLOX ZED-F9P GPS Reader")
    print("=" * 80)
    print()
    
    reader = GPSReader()
    
    if not reader.connect():
        print("Failed to connect to GPS")
        print("\nTip: Make sure ZED-F9P is connected and run 'python3 gps_config.py' if needed")
        return 1
    
    if not reader.start():
        print("Failed to start GPS reader")
        return 1
    
    print("\nStarting plot display in 3 seconds...")
    print("(Waiting for GPS to acquire satellites...)")
    time.sleep(3)
    
    if PLOT_AVAILABLE:
        ui = PlotUI(reader)
        try:
            ui.start()
        finally:
            reader.stop()
    else:
        print("Matplotlib not available. Install with: pip3 install matplotlib numpy")
        reader.stop()
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
