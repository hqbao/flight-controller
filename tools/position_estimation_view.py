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
   #define ENABLE_POSITION_ESTIMATION_MONITOR_LOG 1
   
2. Build and flash the flight controller (SITL or STM32)

3. Run this script:
   python3 position_estimation_view.py

The script auto-detects the serial port and displays 2 real-time plots:
- Position (X, Y, Z) in meters
- Velocity (Vx, Vy, Vz) in m/s

Refreshes at ~25Hz. Requires 'MONITOR_DATA' from the flight controller 
containing 6 floats: pos_x, pos_y, pos_z, vel_x, vel_y, vel_z.
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
MONITOR_DATA_ID = 0x00  # From logger.c

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


# --- Serial Reader Thread ---
def read_serial():
    global running
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
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

                    if checksum_calc == checksum_recv and msg_id == MONITOR_DATA_ID:
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
