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
            
            velN, velE, velD, _gSpeed = struct.unpack('<iiii', payload[48:64])
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
        self.heading_history = deque(maxlen=history_size)
        self.vel_n_history = deque(maxlen=history_size)
        self.vel_e_history = deque(maxlen=history_size)
        self.vel_d_history = deque(maxlen=history_size)
        self.sat_history = deque(maxlen=history_size)
        self.hdop_history = deque(maxlen=history_size)
        self.vdop_history = deque(maxlen=history_size)
        self._last_append_time = 0

        # ── Dark theme ────────────────────────────────────────────────────
        plt.style.use('dark_background')
        BG       = '#1a1a2e'
        PANEL_BG = '#16213e'
        GRID_CLR = '#334466'
        TXT_CLR  = '#e0e0e0'
        ACCENT   = '#0f3460'
        self._colors = {
            'north': '#ff6b6b', 'east': '#51cf66', 'down': '#339af0',
            'alt': '#51cf66', 'sat': '#22b8cf', 'hdop': '#339af0',
            'vdop': '#ff6b6b', 'track': '#74c0fc', 'dot': '#ff6b6b',
        }

        # ── Figure & grid ─────────────────────────────────────────────────
        #   Row 0: status bar  (small)
        #   Row 1: position | altitude | NED velocity
        #   Row 2: position | sats     | NED velocity
        #   Row 3: DOP      | signal   | signal
        #   Row 4: controls / drift bar (small)
        self.fig = plt.figure(figsize=(14, 9), facecolor=BG)
        self.fig.canvas.manager.set_window_title('ZED-F9P GPS Monitor')

        gs = GridSpec(5, 3, figure=self.fig,
                      height_ratios=[0.6, 1, 1, 1, 0.35],
                      hspace=0.35, wspace=0.30,
                      left=0.06, right=0.97, top=0.97, bottom=0.04)

        ax_defaults = dict(facecolor=PANEL_BG)

        # Row 0 — status text across full width
        self.ax_status = self.fig.add_subplot(gs[0, :], **ax_defaults)
        self.ax_status.set_axis_off()
        self.status_text = self.ax_status.text(
            0.01, 0.5, '', transform=self.ax_status.transAxes,
            verticalalignment='center', fontfamily='monospace',
            fontsize=8.5, color=TXT_CLR, linespacing=1.5)

        # Row 1-2 col 0 — Position Track (2 rows)
        self.ax_map = self.fig.add_subplot(gs[1:3, 0], **ax_defaults)
        self.ax_map.set_title('Position Track', fontsize=10, color=TXT_CLR, pad=6)
        self.ax_map.tick_params(colors=TXT_CLR, labelsize=8)
        self.ax_map.grid(True, alpha=0.15, color=GRID_CLR)

        # Row 1 col 1 — Altitude
        self.ax_alt = self.fig.add_subplot(gs[1, 1], **ax_defaults)
        self.ax_alt.set_title('Altitude (MSL)', fontsize=10, color=TXT_CLR, pad=6)
        self.ax_alt.set_ylabel('m', fontsize=8, color=TXT_CLR)
        self.ax_alt.tick_params(colors=TXT_CLR, labelsize=8)
        self.ax_alt.grid(True, alpha=0.15, color=GRID_CLR)

        # Row 2 col 1 — Satellites
        self.ax_sats = self.fig.add_subplot(gs[2, 1], **ax_defaults)
        self.ax_sats.set_title('Satellites Used', fontsize=10, color=TXT_CLR, pad=6)
        self.ax_sats.set_ylabel('Count', fontsize=8, color=TXT_CLR)
        self.ax_sats.tick_params(colors=TXT_CLR, labelsize=8)
        self.ax_sats.grid(True, alpha=0.15, color=GRID_CLR)

        # Row 1-2 col 2 — NED Velocity (2 rows)
        self.ax_vel = self.fig.add_subplot(gs[1:3, 2], **ax_defaults)
        self.ax_vel.set_title('NED Velocity', fontsize=10, color=TXT_CLR, pad=6)
        self.ax_vel.set_ylabel('m/s', fontsize=8, color=TXT_CLR)
        self.ax_vel.tick_params(colors=TXT_CLR, labelsize=8)
        self.ax_vel.grid(True, alpha=0.15, color=GRID_CLR)

        # Row 3 col 0 — DOP
        self.ax_dop = self.fig.add_subplot(gs[3, 0], **ax_defaults)
        self.ax_dop.set_title('DOP', fontsize=10, color=TXT_CLR, pad=6)
        self.ax_dop.set_ylabel('DOP', fontsize=8, color=TXT_CLR)
        self.ax_dop.tick_params(colors=TXT_CLR, labelsize=8)
        self.ax_dop.grid(True, alpha=0.15, color=GRID_CLR)

        # Row 3 col 1-2 — Signal Strength (spans 2 cols)
        self.ax_signal = self.fig.add_subplot(gs[3, 1:], **ax_defaults)
        self.ax_signal.set_title('Signal Strength', fontsize=10, color=TXT_CLR, pad=6)
        self.ax_signal.set_ylabel('C/N0 (dBHz)', fontsize=8, color=TXT_CLR)
        self.ax_signal.tick_params(colors=TXT_CLR, labelsize=8)
        self.ax_signal.grid(True, axis='y', alpha=0.15, color=GRID_CLR)

        # Row 4 — controls & drift
        self.ax_ctrl = self.fig.add_subplot(gs[4, :], **ax_defaults)
        self.ax_ctrl.set_axis_off()

        ax_reset = self.fig.add_axes([0.06, 0.015, 0.07, 0.03])
        self.reset_button = Button(ax_reset, 'Reset Data',
                                   color=ACCENT, hovercolor='#1a4a7a')
        self.reset_button.label.set_color(TXT_CLR)
        self.reset_button.label.set_fontsize(8)
        self.reset_button.on_clicked(self._reset_data)

        ax_duration = self.fig.add_axes([0.21, 0.015, 0.06, 0.03])
        self.duration_box = TextBox(ax_duration, 'Window (s): ',
                                    initial='5.0', color=PANEL_BG,
                                    hovercolor=ACCENT)
        self.duration_box.label.set_color(TXT_CLR)
        self.duration_box.label.set_fontsize(8)
        self.duration_box.text_disp.set_color(TXT_CLR)
        self.duration_box.on_submit(self._on_duration_change)

        ax_origin = self.fig.add_axes([0.30, 0.015, 0.08, 0.03])
        self.measure_button = Button(ax_origin, 'Reset Origin',
                                     color=ACCENT, hovercolor='#1a4a7a')
        self.measure_button.label.set_color(TXT_CLR)
        self.measure_button.label.set_fontsize(8)
        self.measure_button.on_clicked(self._reset_drift_origin)

        self.drift_text = self.fig.text(
            0.42, 0.028, '', transform=self.fig.transFigure,
            verticalalignment='center', fontfamily='monospace',
            fontsize=8.5, color='#ff922b')

        self.start_time = time.time()

        # ── Create persistent line artists (avoids ax.clear() each frame) ─
        C = self._colors

        # Position track — needs full redraw (variable aspect), but cached arrays
        self._map_line, = self.ax_map.plot([], [], color=C['track'], lw=1.2, alpha=0.7)
        self._map_dot, = self.ax_map.plot([], [], 'o', color=C['dot'], markersize=7, zorder=5)
        self._map_arrow = None
        self.ax_map.set_xlabel('East (m)', fontsize=8, color=TXT_CLR)
        self.ax_map.set_ylabel('North (m)', fontsize=8, color=TXT_CLR)

        # Altitude
        self._alt_line, = self.ax_alt.plot([], [], color=C['alt'], lw=1.5)
        self._alt_fill = None

        # Satellites
        self._sat_line, = self.ax_sats.plot([], [], color=C['sat'], lw=1.5)
        self._sat_fill = None
        self.ax_sats.set_ylim(0, 40)

        # NED Velocity
        self._vel_n_line, = self.ax_vel.plot([], [], color=C['north'], lw=1.2, label='North')
        self._vel_e_line, = self.ax_vel.plot([], [], color=C['east'],  lw=1.2, label='East')
        self._vel_d_line, = self.ax_vel.plot([], [], color=C['down'],  lw=1.2, label='Down')
        self._vel_zero = self.ax_vel.axhline(0, color=GRID_CLR, lw=0.5)
        self.ax_vel.legend(loc='upper right', fontsize=7, framealpha=0.4)

        # DOP
        self._hdop_line, = self.ax_dop.plot([], [], color=C['hdop'], lw=1.5, label='HDOP')
        self._vdop_line, = self.ax_dop.plot([], [], color=C['vdop'], lw=1.5, label='VDOP')
        self.ax_dop.set_xlabel('Time (s)', fontsize=8, color=TXT_CLR)
        self.ax_dop.legend(loc='upper right', fontsize=7, framealpha=0.4)
        self.ax_dop.set_ylim(0, 5)

        # Signal — must rebuild bars, but only every ~1 s
        self._signal_frame = 0
        
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
        self.heading_history.clear()
        self.vel_n_history.clear()
        self.vel_e_history.clear()
        self.vel_d_history.clear()
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

        # Reset persistent artists
        self._map_line.set_data([], [])
        self._map_dot.set_data([], [])
        if self._map_arrow is not None:
            self._map_arrow.remove()
            self._map_arrow = None
        self._alt_line.set_data([], [])
        if self._alt_fill is not None:
            self._alt_fill.remove()
            self._alt_fill = None
        self._sat_line.set_data([], [])
        if self._sat_fill is not None:
            self._sat_fill.remove()
            self._sat_fill = None
        self._vel_n_line.set_data([], [])
        self._vel_e_line.set_data([], [])
        self._vel_d_line.set_data([], [])
        self._hdop_line.set_data([], [])
        self._vdop_line.set_data([], [])
    
    def update(self, frame):
        """Update plots — uses set_xdata/set_ydata for speed (no ax.clear)"""
        gps = self.gps_reader.gps_data
        C = self._colors
        GRID_CLR = '#334466'

        # ── Drift measurement ────────────────────────────────────────────
        if gps.drift_origin_set and len(gps.drift_positions) > 0:
            gps.drift_current = haversine_distance(
                gps.drift_origin_lat, gps.drift_origin_lon,
                gps.latitude, gps.longitude)
            now = time.time()
            cutoff = now - gps.drift_window
            window = [(t, la, lo) for t, la, lo in gps.drift_positions if t >= cutoff]
            if window:
                drifts = [haversine_distance(gps.drift_origin_lat, gps.drift_origin_lon, la, lo)
                          for _, la, lo in window]
                gps.drift_max = max(drifts)
                dur = now - window[0][0]
                self.drift_text.set_text(
                    f'Drift | Now: {gps.drift_current*100:.1f} cm | '
                    f'Max ({dur:.0f}s): {gps.drift_max*100:.1f} cm')
            else:
                self.drift_text.set_text(
                    f'Drift | Now: {gps.drift_current*100:.1f} cm')
        else:
            self.drift_text.set_text('Waiting for fix…')

        # ── Append history ───────────────────────────────────────────────
        if gps.last_update_time > self._last_append_time:
            self._last_append_time = gps.last_update_time
            t = time.time() - self.start_time
            self.time_history.append(t)
            self.lat_history.append(gps.latitude)
            self.lon_history.append(gps.longitude)
            self.alt_history.append(gps.altitude_msl)
            self.heading_history.append(gps.heading)
            self.vel_n_history.append(gps.vel_north)
            self.vel_e_history.append(gps.vel_east)
            self.vel_d_history.append(gps.vel_down)
            self.sat_history.append(gps.num_satellites)
            self.hdop_history.append(gps.hdop)
            self.vdop_history.append(gps.vdop)

        th = list(self.time_history)

        # ── Position Track (must redraw for equal aspect) ────────────────
        if len(self.lat_history) > 1:
            lats = list(self.lat_history)
            lons = list(self.lon_history)
            lat0, lon0 = lats[0], lons[0]
            cos_lat = np.cos(np.radians(lat0))
            xs = [(lo - lon0) * 111320 * cos_lat for lo in lons]
            ys = [(la - lat0) * 110540 for la in lats]
            self._map_line.set_data(xs, ys)
            self._map_dot.set_data([xs[-1]], [ys[-1]])
            # Heading arrow
            if self._map_arrow is not None:
                self._map_arrow.remove()
                self._map_arrow = None
            h_speed = (gps.vel_north**2 + gps.vel_east**2)**0.5
            if h_speed > 0.5:
                rng = max(max(xs) - min(xs), max(ys) - min(ys), 1.0) * 0.08
                dx = rng * np.sin(np.radians(gps.heading))
                dy = rng * np.cos(np.radians(gps.heading))
                self._map_arrow = self.ax_map.annotate(
                    '', xy=(xs[-1]+dx, ys[-1]+dy), xytext=(xs[-1], ys[-1]),
                    arrowprops=dict(arrowstyle='->', color=C['dot'], lw=1.5))
            self.ax_map.relim()
            self.ax_map.autoscale_view()
            self.ax_map.set_aspect('equal', adjustable='datalim')

        # ── Altitude (in-place update) ───────────────────────────────────
        if th:
            alt_vals = list(self.alt_history)
            self._alt_line.set_data(th, alt_vals)
            # Rebuild fill sparingly
            if self._alt_fill is not None:
                self._alt_fill.remove()
            self._alt_fill = self.ax_alt.fill_between(th, alt_vals, alpha=0.08, color=C['alt'])
            self.ax_alt.relim()
            self.ax_alt.autoscale_view()

        # ── Satellites (in-place update) ─────────────────────────────────
        if th:
            sat_vals = list(self.sat_history)
            self._sat_line.set_data(th, sat_vals)
            if self._sat_fill is not None:
                self._sat_fill.remove()
            self._sat_fill = self.ax_sats.fill_between(th, 0, sat_vals, alpha=0.12, color=C['sat'])
            self.ax_sats.set_xlim(th[0], th[-1] if th[-1] > th[0] else th[0] + 1)

        # ── NED Velocity (in-place update) ───────────────────────────────
        if th:
            self._vel_n_line.set_data(th, list(self.vel_n_history))
            self._vel_e_line.set_data(th, list(self.vel_e_history))
            self._vel_d_line.set_data(th, list(self.vel_d_history))
            self.ax_vel.relim()
            self.ax_vel.autoscale_view()

        # ── DOP (in-place update) ────────────────────────────────────────
        if th:
            self._hdop_line.set_data(th, list(self.hdop_history))
            self._vdop_line.set_data(th, list(self.vdop_history))
            self.ax_dop.set_xlim(th[0], th[-1] if th[-1] > th[0] else th[0] + 1)
            hdop_max = max(list(self.hdop_history) + list(self.vdop_history))
            self.ax_dop.set_ylim(0, max(5, hdop_max * 1.2))

        # ── Signal Strength (rebuild bars every 10 frames ≈ 1 s) ────────
        self._signal_frame += 1
        if self._signal_frame >= 10:
            self._signal_frame = 0
            ax = self.ax_signal
            # Remove old bars but keep axis styling
            for container in ax.containers[:]:
                container.remove()
            for line in ax.get_lines():
                line.remove()
            if gps.satellites:
                gnss_clr = {0: '#339af0', 1: '#ffa94d', 2: '#51cf66',
                            3: '#ff6b6b', 5: '#cc5de8', 6: '#22b8cf'}
                gnss_nm  = {0: 'GPS', 1: 'SBA', 2: 'Gal', 3: 'BDS', 5: 'QZS', 6: 'GLO'}
                sorted_s = sorted(gps.satellites, key=lambda s: (s['gnss'], -s['cno']))
                labels, vals, clrs = [], [], []
                for s in sorted_s[:24]:
                    labels.append(f"{gnss_nm.get(s['gnss'], '?')}\n{s['svid']}")
                    vals.append(s['cno'])
                    c = gnss_clr.get(s['gnss'], '#888888')
                    clrs.append(c if s['used'] else '#333344')
                if labels:
                    ax.bar(range(len(labels)), vals, color=clrs, alpha=0.85, width=0.7)
                    ax.set_xticks(range(len(labels)))
                    ax.set_xticklabels(labels, fontsize=6, color='#e0e0e0')
                    ax.axhline(30, color='#ffa94d', ls='--', lw=0.7, alpha=0.5)
                    ax.axhline(40, color='#51cf66', ls='--', lw=0.7, alpha=0.5)
                    ax.set_ylim(0, max(50, max(vals) + 5))

        # ── Status text ──────────────────────────────────────────────────
        fix_names = ['No Fix', 'Dead Reckoning', '2D Fix', '3D Fix', 'GNSS+DR', 'Time Only']
        fix_str = fix_names[gps.fix_type] if gps.fix_type < len(fix_names) else '?'
        rtk = {1: ' RTK-Float', 2: ' RTK-Fixed'}.get(gps.carrier_solution, '')
        utc = ''
        if gps.time_valid and gps.time_utc:
            utc = gps.time_utc.strftime('%Y-%m-%d %H:%M:%S UTC')
        status = (
            f"Fix: {fix_str}{rtk}  |  Sats: {gps.num_satellites}/{gps.num_satellites_tracked}  "
            f"|  Rate: {gps.update_rate:.1f} Hz  |  {utc}\n"
            f"Pos: {gps.latitude:.8f}°, {gps.longitude:.8f}°  |  "
            f"Alt: {gps.altitude_msl:.2f} m  |  hAcc: ±{gps.h_acc:.2f} m  vAcc: ±{gps.v_acc:.2f} m\n"
            f"Vel NED: [{gps.vel_north:+.3f}, {gps.vel_east:+.3f}, {gps.vel_down:+.3f}] m/s  "
            f"|  sAcc: ±{gps.speed_acc:.2f} m/s  |  "
            f"Hdg: {gps.heading:.1f}° ±{gps.heading_acc:.1f}°  |  "
            f"PDOP: {gps.pdop:.2f}  HDOP: {gps.hdop:.2f}  VDOP: {gps.vdop:.2f}"
        )
        self.status_text.set_text(status)
        
    def start(self):
        """Start animation"""
        ani = animation.FuncAnimation(
            self.fig, 
            self.update, 
            interval=100,  # 10 Hz to match GPS update rate
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
        print("\nTip: Make sure ZED-F9P is connected and run 'python3 gps_config_f9p.py' if needed")
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
