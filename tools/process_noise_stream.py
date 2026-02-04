import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import matplotlib.animation as animation
import time
import collections
import sys
import argparse
import csv

"""
IMU Noise Analytics Tool (Unified)

Part 1: Real-Time Monitor
Visualizes the Noise Statistics (Mean/StdDev) enabled by noise_meas.c
Data is received via 'db' protocol over Serial.
Usage: python3 process_noise_stream.py (no args)

Part 2: Static Log Analysis
Analyzes a CSV log file for clipping and vibration issues.
Usage: python3 process_noise_stream.py <filename.csv>

Packet Format (56 bytes):
Header: 'd' 'b' 0x00 0x00 (4 bytes)
Length: 0x30 0x00 (48 bytes) -> 12 floats
Payload: 12 floats (48 bytes)
Checksum: 2 bytes
"""

# --- Configuration for Real-Time ---
SERIAL_BAUD = 9600 # Default for STM32 Virtual Com Port
PLOT_HISTORY_SEC = 10
UPDATE_RATE_HZ = 5 # Firmware updates at 5Hz
DATA_POINTS = PLOT_HISTORY_SEC * UPDATE_RATE_HZ 

SCALE_1G_RT = 16384.0 # For +/- 2G range

# --- Configuration for Static Analysis ---
SCALE_1G_STATIC = 16384.0 
SENSOR_MAX = 32767.0 # Max 16-bit signed integer (representing 2G)
CLIPPING_THRESHOLD = SENSOR_MAX * 0.90 
STD_EXCELLENT = 0.1 * SCALE_1G_STATIC
STD_ACCEPTABLE = 0.3 * SCALE_1G_STATIC
STD_DANGEROUS = 0.5 * SCALE_1G_STATIC


# --- Real-Time Logic ---
data_buffer = collections.deque(maxlen=DATA_POINTS)
latest_stats = {
    'gx_std': 0, 'gy_std': 0, 'gz_std': 0,
    'ax_std': 0, 'ay_std': 0, 'az_std': 0,
    'az_mean': 0
}
running = True
connection_status = "Disconnected"

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        # Look for typical STM32/ESP32 identifiers
        if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM']):
            return port
    return None

def packet_parser(serial_port):
    global running, connection_status, latest_stats
    
    # State machine for 'db' protocol
    HEADER = b'db\x00\x00'
    
    try:
        ser = serial.Serial(serial_port, SERIAL_BAUD, timeout=0.1)
        connection_status = f"Connected: {serial_port}"
        print(f"Opened {serial_port}")
    except Exception as e:
        connection_status = f"Error: {e}"
        return

    buffer = bytearray()
    
    while running:
        try:
            chunk = ser.read(64)
            if not chunk:
                continue
            
            buffer.extend(chunk)
            
            while len(buffer) >= 6: # Shortest possible header + len is 6
                header_idx = buffer.find(HEADER)
                if header_idx == -1:
                    buffer = buffer[-3:] 
                    break
                
                if header_idx > 0:
                    buffer = buffer[header_idx:]
                
                if len(buffer) < 6:
                    break
                    
                payload_len = buffer[4] | (buffer[5] << 8)
                packet_len = 6 + payload_len + 2 
                
                if len(buffer) < packet_len:
                    break 
                
                packet = buffer[:packet_len]
                buffer = buffer[packet_len:] 
                
                calc_sum = packet[2] + packet[3] + packet[4] + packet[5]
                payload = packet[6:6+payload_len]
                for b in payload:
                    calc_sum += b
                
                recv_sum = packet[-2] | (packet[-1] << 8)
                
                if (calc_sum & 0xFFFF) != recv_sum:
                    print(f"Checksum Fail: Calc {calc_sum & 0xFFFF:04x} != Recv {recv_sum:04x}")
                    continue
                
                if payload_len == 48:
                    values = struct.unpack('<12f', payload)
                    
                    # Extract Accel Std (Indices 9, 10, 11)
                    ax_std_g = values[9] / SCALE_1G_RT
                    ay_std_g = values[10] / SCALE_1G_RT
                    az_std_g = values[11] / SCALE_1G_RT
                    
                    # Extract Gyro Std (Indices 3, 4, 5)
                    gx_std = values[3]
                    gy_std = values[4]
                    gz_std = values[5]
                    
                    az_mean = values[8] / SCALE_1G_RT
                    
                    latest_stats = {
                        'ax_std': ax_std_g,
                        'ay_std': ay_std_g,
                        'az_std': az_std_g,
                        'gx_std': gx_std,
                        'gy_std': gy_std,
                        'gz_std': gz_std,
                        'az_mean': az_mean
                    }
                    
                    data_buffer.append(latest_stats.copy())
        except Exception as e:
            print(f"Serial Error: {e}")
            break
                    
    ser.close()

def run_realtime_monitor():
    global running
    
    port = find_serial_port()
    if not port:
        print("No serial port found. Connect flight controller via USB.")
        return

    # Start Serial Thread
    t = threading.Thread(target=packet_parser, args=(port,))
    t.start()
    
    fig = plt.figure(figsize=(12, 8))
    fig.canvas.manager.set_window_title('SkyDev Flight Stability Monitor (Real-Time)')
    
    gs = fig.add_gridspec(3, 2, height_ratios=[1, 2, 2])
    
    ax_status = fig.add_subplot(gs[0, :])
    ax_status.axis('off')
    
    ax_accel = fig.add_subplot(gs[1, :])
    ax_gyro = fig.add_subplot(gs[2, :])
    
    lines_accel = []
    labels_accel = ['Ax Std', 'Ay Std', 'Az Std']
    colors_accel = ['r', 'g', 'b']
    
    lines_gyro = []
    labels_gyro = ['Gx Std', 'Gy Std', 'Gz Std']
    colors_gyro = ['c', 'm', 'y']
    
    for i in range(3):
        line, = ax_accel.plot([], [], lw=2, label=labels_accel[i], color=colors_accel[i])
        lines_accel.append(line)
        
        line, = ax_gyro.plot([], [], lw=2, label=labels_gyro[i], color=colors_gyro[i])
        lines_gyro.append(line)
        
    ax_accel.set_ylabel('Vibration (G)')
    ax_accel.set_ylim(0, 0.6) 
    ax_accel.axhline(y=0.1, color='g', linestyle='--', alpha=0.5, label='Excellent')
    ax_accel.axhline(y=0.3, color='orange', linestyle='--', alpha=0.5, label='Warning')
    ax_accel.axhline(y=0.5, color='r', linestyle='--', alpha=0.5, label='Danger')
    ax_accel.legend(loc='upper right')
    ax_accel.grid(True)
    
    ax_gyro.set_ylabel('Noise (dps)')
    ax_gyro.set_ylim(0, 2.0)
    ax_gyro.legend(loc='upper right')
    ax_gyro.grid(True)
    
    def update(frame):
        # Update Text
        status_text = f"Connection: {connection_status}\n"
        max_vib = max(latest_stats['ax_std'], latest_stats['ay_std'], latest_stats['az_std'])
        
        state = "UNKNOWN"
        bg_color = "lightgray"
        
        if max_vib > 0.5:
            state = "UNSTABLE / DANGEROUS"
            bg_color = "#ffcccc" # Light Red
        elif max_vib > 0.3:
            state = "WARNING"
            bg_color = "#ffeeb0" # Light Orange
        else:
            state = "STABLE"
            bg_color = "#ccffcc" # Light Green
            
        status_text += f"Status: {state}\n"
        status_text += f"Peak Vibration: {max_vib:.3f} G\n"
        status_text += f"Az Mean: {latest_stats['az_mean']:.2f} G (Should be ~1.0)"
        
        ax_status.clear()
        ax_status.axis('off')
        ax_status.text(0.5, 0.5, status_text, ha='center', va='center', fontsize=14, 
                       bbox=dict(facecolor=bg_color, alpha=0.5, boxstyle='round,pad=1'))
        
        if len(data_buffer) > 0:
            x = np.arange(len(data_buffer))
            
            ax_data = [d['ax_std'] for d in data_buffer]
            ay_data = [d['ay_std'] for d in data_buffer]
            az_data = [d['az_std'] for d in data_buffer]
            
            gx_data = [d['gx_std'] for d in data_buffer]
            gy_data = [d['gy_std'] for d in data_buffer]
            gz_data = [d['gz_std'] for d in data_buffer]
            
            lines_accel[0].set_data(x, ax_data)
            lines_accel[1].set_data(x, ay_data)
            lines_accel[2].set_data(x, az_data)
            ax_accel.set_xlim(0, len(data_buffer))
            
            lines_gyro[0].set_data(x, gx_data)
            lines_gyro[1].set_data(x, gy_data)
            lines_gyro[2].set_data(x, gz_data)
            ax_gyro.set_xlim(0, len(data_buffer))
            
        return lines_accel + lines_gyro

    ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.tight_layout()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    running = False
    t.join()


# --- Static Analysis Logic ---
def analyze_static_log(filename):
    timestamps = []
    az_means = []
    az_stds = []
    ax_stds = []
    ay_stds = []
    
    print(f"Loading data from {filename}...")
    
    try:
        with open(filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                try:
                    timestamps.append(float(row['Timestamp']))
                    az_means.append(float(row['Az_Mean']))
                    az_stds.append(float(row['Az_Std']))
                    ax_stds.append(float(row['Ax_Std']))
                    ay_stds.append(float(row['Ay_Std']))
                except ValueError:
                    continue
    except FileNotFoundError:
        print(f"Error: File {filename} not found.")
        return

    if not timestamps:
        print("Error: No valid data found in file.")
        return

    # Metrics
    max_az_mean = max(az_means)
    min_az_mean = min(az_means)
    max_az_std = max(az_stds)
    
    # Reports
    print("-" * 40)
    print("      STATIC ANALYSIS REPORT      ")
    print("-" * 40)
    
    # Check 1: Saturation/Clipping
    if max_az_mean > CLIPPING_THRESHOLD or min_az_mean < -CLIPPING_THRESHOLD:
        print(f"CRITICAL: Accelerometer Clipping detected!")
    else:
        print(f"Saturation: OK (Headroom: {(SENSOR_MAX - max_az_mean)/SCALE_1G_STATIC:.2f}G)")

    # Check 2: Vibration Levels
    max_vib_g = max_az_std/SCALE_1G_STATIC
    if max_az_std > STD_DANGEROUS:
        print(f"Vibration:  DANGEROUS ({max_vib_g:.2f} G)")
    elif max_az_std > STD_ACCEPTABLE:
        print(f"Vibration:  WARNING ({max_vib_g:.2f} G)")
    else:
        print(f"Vibration:  EXCELLENT ({max_vib_g:.2f} G)")
    
    print("-" * 40)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        # If argument provided, assume static file analysis
        analyze_static_log(sys.argv[1])
    else:
        # If no argument, run real-time monitor
        run_realtime_monitor()
