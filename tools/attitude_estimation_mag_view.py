import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d import Axes3D
import time

"""
Attitude Estimation - Magnetometer Debug Tool

Visualizes tilt-compensated magnetometer data in real-time.
Uses LOG_CLASS_ATTITUDE_MAG (0x07) — no recompilation needed.

Data (9 floats, 36 bytes):
- Red:   Raw Magnetometer Vector (body frame)
- Blue:  Earth-Frame Mag Vector (tilt-compensated)
- Green: Predicted Gravity Vector (attitude reference)

USAGE:
1. Build and flash the flight controller
2. Run: python3 attitude_estimation_mag_view.py
3. Click "Start Log" to activate mag debug streaming

Useful for:
- Verifying compass calibration (hard/soft iron)
- Checking tilt compensation quality
- Debugging heading estimation
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
SEND_LOG_ID = 0x00  # Log data message ID

# Log class constants (match messages.h)
LOG_CLASS_NONE      = 0x00
LOG_CLASS_IMU_ACCEL = 0x01
LOG_CLASS_COMPASS   = 0x02
LOG_CLASS_ATTITUDE  = 0x03
LOG_CLASS_POSITION  = 0x04
LOG_CLASS_IMU_GYRO  = 0x05
LOG_CLASS_POSITION_OPTFLOW = 0x06
LOG_CLASS_ATTITUDE_MAG = 0x07
DB_CMD_LOG_CLASS    = 0x03  # Command ID for setting log class

# Auto-detect serial port
ports = serial.tools.list_ports.comports()
found_port = False
print("Scanning for ports...")
for port, desc, hwid in sorted(ports):
    if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB']):
        SERIAL_PORT = port
        found_port = True
        print(f"Auto-selected Port: {port} ({desc})")
        break
    else:
        print(f"Skipped: {port} ({desc})")

if not found_port:
    print('----------------------------------------------------')
    print('ERROR: No compatible serial port found.')
    print('Please connect the Flight Controller and try again.')
    print('----------------------------------------------------')


# --- Global State ---
data_queue = queue.Queue()
is_collecting = True
g_serial = None
v_raw_mag = np.array([0.0, 0.0, 0.0])
v_earth_mag = np.array([0.0, 0.0, 0.0])
v_attitude = np.array([0.0, 0.0, 0.0])

# --- Log Class Command ---
def send_log_class_command(ser, log_class):
    """Send DB frame to set active log class on the flight controller."""
    msg_id = DB_CMD_LOG_CLASS
    msg_class = 0x00
    length = 1
    payload = bytes([log_class])
    header = struct.pack('<2sBBH', b'db', msg_id, msg_class, length)
    checksum = (msg_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF) + log_class) & 0xFFFF
    frame = header + payload + struct.pack('<H', checksum)
    ser.write(frame)
    ser.flush()
    print(f"  → Log class set to 0x{log_class:02X}")

# --- Serial Reader ---
def serial_reader():
    global SERIAL_PORT
    if not SERIAL_PORT:
        return

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            global g_serial
            g_serial = ser
            print(f"Connected to {SERIAL_PORT}")
            while True:
                b1 = ser.read(1)
                if not b1: continue
                if b1[0] != 0x62 and b1[0] != 0x64: continue
                
                b2 = ser.read(1)
                if not b2: continue
                
                if not ((b1[0] == 0x64 and b2[0] == 0x62) or (b1[0] == 0x62 and b2[0] == 0x64)):
                    continue
                
                id_byte = ser.read(1)
                if not id_byte: continue
                msg_id = id_byte[0]
                
                class_byte = ser.read(1)
                if not class_byte: continue
                
                len_bytes = ser.read(2)
                if len(len_bytes) < 2: continue
                length = int.from_bytes(len_bytes, 'little')
                
                if length > 1024: continue
                
                payload = ser.read(length)
                if len(payload) != length: continue
                
                if msg_id == SEND_LOG_ID:
                    if length == 36:  # 9 floats
                        vals = struct.unpack('fffffffff', payload)
                        data_queue.put(vals)
                    
    except Exception as e:
        print(f"Serial error: {e}")

# --- GUI ---
def main():
    global is_collecting, v_raw_mag, v_earth_mag, v_attitude
    
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect((1, 1, 1))
    plt.subplots_adjust(bottom=0.2, right=0.75)
    
    # Red: Raw Mag, Blue: Earth Mag, Green: Attitude (gravity)
    line_raw, = ax.plot([0, 0], [0, 0], [0, 0], color='red', linewidth=3, label='Raw Mag (body)')
    head_raw, = ax.plot([0], [0], [0], color='red', marker='o', markersize=8)
    
    line_earth, = ax.plot([0, 0], [0, 0], [0, 0], color='blue', linewidth=3, label='Earth Mag (tilt-comp)')
    head_earth, = ax.plot([0], [0], [0], color='blue', marker='o', markersize=8)
    
    line_att, = ax.plot([0, 0], [0, 0], [0, 0], color='green', linewidth=2, label='Gravity (v_pred)')
    head_att, = ax.plot([0], [0], [0], color='green', marker='o', markersize=6)
    
    # Text panel
    text_ax = plt.axes([0.76, 0.2, 0.23, 0.6])
    text_ax.axis('off')
    info_text = text_ax.text(0, 1, "Waiting for data...", fontsize=10, va='top', fontfamily='monospace')
    
    # View Buttons
    views = {
        'Top': (90, -90), 'Bottom': (-90, -90),
        'Front': (0, -90), 'Back': (0, 90),
        'Left': (0, 180), 'Right': (0, 0)
    }
    
    start_x = 0.05
    width = 0.06
    gap = 0.01
    row1_y = 0.1
    
    view_btns = []
    for i, (label, (elev, azim)) in enumerate(views.items()):
        b_ax = plt.axes([start_x + i*(width+gap), row1_y, width, 0.05])
        b = Button(b_ax, label)
        b.on_clicked(lambda event, e=elev, a=azim: (ax.view_init(elev=e, azim=a), plt.draw()))
        view_btns.append(b)

    # Stream Toggle
    btn_stream_ax = plt.axes([start_x, 0.04, 0.15, 0.05])
    btn_stream = Button(btn_stream_ax, 'Stop Stream')
    
    def toggle_stream(event):
        global is_collecting
        is_collecting = not is_collecting
        btn_stream.label.set_text('Start Stream' if not is_collecting else 'Stop Stream')
        
    btn_stream.on_clicked(toggle_stream)

    # Start Log — sends LOG_CLASS_ATTITUDE_MAG
    btn_log_ax = plt.axes([start_x + 0.16, 0.04, 0.15, 0.05])
    btn_log = Button(btn_log_ax, 'Start Log', color='#335533', hovercolor='#557755')
    
    def start_log(event):
        if g_serial and g_serial.is_open:
            send_log_class_command(g_serial, LOG_CLASS_ATTITUDE_MAG)
        else:
            print('Serial not connected')
    
    btn_log.on_clicked(start_log)

    # Stop Log
    btn_stop_ax = plt.axes([start_x + 0.32, 0.04, 0.15, 0.05])
    btn_stop = Button(btn_stop_ax, 'Stop Log', color='#553333', hovercolor='#775555')

    def stop_log(event):
        if g_serial and g_serial.is_open:
            send_log_class_command(g_serial, LOG_CLASS_NONE)
        else:
            print('Serial not connected')

    btn_stop.on_clicked(stop_log)
    
    # Setup Axes
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.set_title("Magnetometer Debug — Tilt Compensation")
    
    while True:
        latest_vals = None
        while not data_queue.empty():
            latest_vals = data_queue.get()
        
        if is_collecting and latest_vals:
            v_raw_mag = np.array(latest_vals[0:3])
            v_earth_mag = np.array(latest_vals[3:6])
            v_attitude = np.array(latest_vals[6:9])
            
            # Compute heading from earth-frame mag
            heading = np.degrees(np.arctan2(-v_earth_mag[1], v_earth_mag[0]))
            
            # Update plots
            line_raw.set_data([0, v_raw_mag[0]], [0, v_raw_mag[1]])
            line_raw.set_3d_properties([0, v_raw_mag[2]])
            head_raw.set_data([v_raw_mag[0]], [v_raw_mag[1]])
            head_raw.set_3d_properties([v_raw_mag[2]])
            
            line_earth.set_data([0, v_earth_mag[0]], [0, v_earth_mag[1]])
            line_earth.set_3d_properties([0, v_earth_mag[2]])
            head_earth.set_data([v_earth_mag[0]], [v_earth_mag[1]])
            head_earth.set_3d_properties([v_earth_mag[2]])
            
            line_att.set_data([0, v_attitude[0]], [0, v_attitude[1]])
            line_att.set_3d_properties([0, v_attitude[2]])
            head_att.set_data([v_attitude[0]], [v_attitude[1]])
            head_att.set_3d_properties([v_attitude[2]])
            
            # Update text
            info_text.set_text(
                f"Raw Mag (Red):\n"
                f"X: {v_raw_mag[0]:.3f}\n"
                f"Y: {v_raw_mag[1]:.3f}\n"
                f"Z: {v_raw_mag[2]:.3f}\n"
                f"Mag: {np.linalg.norm(v_raw_mag):.3f}\n\n"
                f"Earth Mag (Blue):\n"
                f"X: {v_earth_mag[0]:.3f}\n"
                f"Y: {v_earth_mag[1]:.3f}\n"
                f"Z: {v_earth_mag[2]:.3f}\n"
                f"Mag: {np.linalg.norm(v_earth_mag):.3f}\n\n"
                f"Gravity (Green):\n"
                f"X: {v_attitude[0]:.3f}\n"
                f"Y: {v_attitude[1]:.3f}\n"
                f"Z: {v_attitude[2]:.3f}\n\n"
                f"Heading: {heading:.1f}°"
            )

        plt.pause(0.05)
        if not plt.fignum_exists(fig.number):
            break

if __name__ == "__main__":
    main()
