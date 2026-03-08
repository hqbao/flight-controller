import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import time

"""
Position Estimation Visualization Tool

Visualizes position and velocity estimation in real-time.

MODE 1 DATA (6 floats, 24 bytes):
- Position (X, Y, Z) in meters
- Velocity (Vx, Vy, Vz) in m/s

USAGE:
1. Set logging mode in position_estimation.c:
   #define POSITION_MONITOR_MODE 1
   
2. Build and flash the flight controller (SITL or STM32)

3. Run this script:
   python3 position_estimation_view.py
   (Log class is set automatically via UART command on connect)

The script auto-detects the serial port and displays 2 real-time plots:
- Position (X, Y, Z) in meters
- Velocity (Vx, Vy, Vz) in m/s

Refreshes at ~25Hz. Receives 'SEND_LOG' data from the flight controller
containing 6 floats: pos_x, pos_y, pos_z, vel_x, vel_y, vel_z.
The log class is set automatically via UART command on connect.
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
    # Filter for typical STM32/ESP32 USB IDs
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
running = True
g_serial = None

# Data buffers for plotting (keep last 200 samples = ~8 seconds at 25Hz)
MAX_SAMPLES = 200
timestamps = np.zeros(MAX_SAMPLES)
pos_x = np.zeros(MAX_SAMPLES)
pos_y = np.zeros(MAX_SAMPLES)
pos_z = np.zeros(MAX_SAMPLES)
vel_x = np.zeros(MAX_SAMPLES)
vel_y = np.zeros(MAX_SAMPLES)
vel_z = np.zeros(MAX_SAMPLES)

current_idx = 0
start_time = time.time()


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
    print(f"  \u2192 Log class set to 0x{log_class:02X}")

# --- Serial Reader Thread ---
def read_serial():
    global running
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        global g_serial
        g_serial = ser
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        time.sleep(1)  # Wait for serial to stabilize

        buffer = bytearray()

        while running:
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))

                # Look for message header: 'db' + ID + Class
                while len(buffer) >= 6:
                    # Search for header
                    if buffer[0] != ord('d') or buffer[1] != ord('b'):
                        buffer.pop(0)
                        continue

                    msg_id = buffer[2]
                    msg_class = buffer[3]
                    payload_size = struct.unpack('<H', buffer[4:6])[0]

                    # Check if full message is in buffer
                    if len(buffer) < 6 + payload_size + 2:
                        break  # Need more data

                    # Extract payload and checksum
                    payload = buffer[6:6 + payload_size]
                    checksum_recv = struct.unpack('<H', buffer[6 + payload_size:6 + payload_size + 2])[0]

                    # Verify checksum
                    checksum_calc = msg_id + msg_class + buffer[4] + buffer[5] + sum(payload)
                    checksum_calc = checksum_calc & 0xFFFF

                    if checksum_calc == checksum_recv and msg_id == SEND_LOG_ID:
                        # Parse 6 floats (24 bytes)
                        if len(payload) >= 24:
                            data = struct.unpack('<6f', payload[:24])
                            data_queue.put(data)

                    # Remove processed message from buffer
                    buffer = buffer[6 + payload_size + 2:]

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
        print("Serial connection closed")


# --- Data Update Function ---
def update_data():
    global current_idx, timestamps
    global pos_x, pos_y, pos_z
    global vel_x, vel_y, vel_z

    try:
        while not data_queue.empty():
            data = data_queue.get_nowait()

            # Shift data left
            timestamps[:-1] = timestamps[1:]
            pos_x[:-1] = pos_x[1:]
            pos_y[:-1] = pos_y[1:]
            pos_z[:-1] = pos_z[1:]
            vel_x[:-1] = vel_x[1:]
            vel_y[:-1] = vel_y[1:]
            vel_z[:-1] = vel_z[1:]

            # Add new data
            timestamps[-1] = time.time() - start_time
            pos_x[-1] = data[0]
            pos_y[-1] = data[1]
            pos_z[-1] = data[2]
            vel_x[-1] = data[3]
            vel_y[-1] = data[4]
            vel_z[-1] = data[5]

    except queue.Empty:
        pass


# --- Plot Update Function ---
def update_plot(frame):
    update_data()

    # Clear all axes
    for ax in [ax1, ax2]:
        ax.clear()

    # Get valid time range
    valid_mask = timestamps > 0
    t = timestamps[valid_mask]

    # Plot Position
    ax1.plot(t, pos_x[valid_mask], 'r-', label='X', linewidth=1.5)
    ax1.plot(t, pos_y[valid_mask], 'g-', label='Y', linewidth=1.5)
    ax1.plot(t, pos_z[valid_mask], 'b-', label='Z', linewidth=1.5)
    ax1.set_ylabel('Position (m)')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Position Estimation (X, Y, Z)')

    # Plot Velocity
    ax2.plot(t, vel_x[valid_mask], 'r-', label='Vx', linewidth=1.5)
    ax2.plot(t, vel_y[valid_mask], 'g-', label='Vy', linewidth=1.5)
    ax2.plot(t, vel_z[valid_mask], 'b-', label='Vz', linewidth=1.5)
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_xlabel('Time (s)')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Velocity (Vx, Vy, Vz)')


# --- Exit Handler ---
def on_close(event):
    global running
    running = False
    print("Closing application...")


def on_exit_button(event):
    global running
    running = False
    plt.close('all')


# --- Main ---
if __name__ == '__main__':
    if not found_port:
        exit(1)

    # Start serial reader thread
    reader_thread = threading.Thread(target=read_serial, daemon=True)
    reader_thread.start()

    # Create figure with 2 subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Position Estimation Viewer (Mode 1)', fontsize=16)
    fig.canvas.mpl_connect('close_event', on_close)

    # Add exit button
    ax_button = plt.axes([0.45, 0.02, 0.1, 0.04])
    btn_exit = Button(ax_button, 'Exit')
    btn_exit.on_clicked(on_exit_button)

    # Start Log button
    ax_log = plt.axes([0.57, 0.02, 0.12, 0.04])
    btn_log = Button(ax_log, 'Start Log', color='#335533', hovercolor='#557755')
    def start_log(event):
        if g_serial and g_serial.is_open:
            send_log_class_command(g_serial, LOG_CLASS_POSITION)
        else:
            print('Serial not connected')
    btn_log.on_clicked(start_log)

    # Adjust layout
    plt.subplots_adjust(left=0.08, right=0.95, top=0.94, bottom=0.08, hspace=0.25)

    # Start animation
    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, update_plot, interval=40, cache_frame_data=False)  # ~25Hz

    plt.show()

    # Cleanup
    running = False
    reader_thread.join(timeout=2)
    print("Application closed")
