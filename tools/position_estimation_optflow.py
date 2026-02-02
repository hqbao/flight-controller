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
Position Estimation Optical Flow Visualization Tool

Visualizes optical flow data from upward and downward facing cameras in real-time.

MODE 2 DATA (6 floats, 24 bytes):
- Optical Flow Downward (dx, dy) - Blue
- Optical Flow Upward (dx, dy) - Red  
- Range Finder Altitude (raw sensor)
- Estimated Altitude (fused)

NOTE: Values are raw radians (small values).

USAGE:
1. Set logging mode in position_estimation.c:
   #define ENABLE_POSITION_ESTIMATION_MONITOR_LOG 2
   
2. Build and flash the flight controller (SITL or STM32)

3. Run this script:
   python3 position_estimation_optflow.py

The script auto-detects the serial port and displays 3 real-time plots:
- Optical Flow Downward (dx, dy) in radians
- Optical Flow Upward (dx, dy) in radians
- Altitude (Range vs Estimated) with Flow Magnitude overlay

Refreshes at ~25Hz. Requires 'MONITOR_DATA' from the flight controller
containing 6 floats: optflow_down_dx, optflow_down_dy, optflow_up_dx,
optflow_up_dy, range_alt, est_alt.
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
optflow_down_dx = np.zeros(MAX_SAMPLES)
optflow_down_dy = np.zeros(MAX_SAMPLES)
optflow_up_dx = np.zeros(MAX_SAMPLES)
optflow_up_dy = np.zeros(MAX_SAMPLES)
range_alt = np.zeros(MAX_SAMPLES)
est_alt = np.zeros(MAX_SAMPLES)

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
    global optflow_down_dx, optflow_down_dy
    global optflow_up_dx, optflow_up_dy
    global range_alt, est_alt

    try:
        while not data_queue.empty():
            data = data_queue.get_nowait()

            # Shift data left
            timestamps[:-1] = timestamps[1:]
            optflow_down_dx[:-1] = optflow_down_dx[1:]
            optflow_down_dy[:-1] = optflow_down_dy[1:]
            optflow_up_dx[:-1] = optflow_up_dx[1:]
            optflow_up_dy[:-1] = optflow_up_dy[1:]
            range_alt[:-1] = range_alt[1:]
            est_alt[:-1] = est_alt[1:]

            # Add new data
            timestamps[-1] = time.time() - start_time
            optflow_down_dx[-1] = data[0]
            optflow_down_dy[-1] = data[1]
            optflow_up_dx[-1] = data[2]
            optflow_up_dy[-1] = data[3]
            range_alt[-1] = data[4]
            est_alt[-1] = data[5]

    except queue.Empty:
        pass


# --- Plot Update Function ---
def update_plot(frame):
    update_data()

    # Clear all axes
    for ax in [ax1, ax2, ax3, ax3_twin]:
        ax.clear()

    # Get valid time range
    valid_mask = timestamps > 0
    t = timestamps[valid_mask]

    # Plot Optical Flow Downward
    ax1.plot(t, optflow_down_dx[valid_mask], 'b-', label='dx', linewidth=2)
    ax1.plot(t, optflow_down_dy[valid_mask], 'c-', label='dy', linewidth=2)
    ax1.axhline(y=0, color='gray', linestyle='--', linewidth=0.5, alpha=0.5)
    ax1.set_ylabel('Flow (rad)', fontsize=11)
    ax1.legend(loc='upper right', fontsize=10)
    ax1.grid(True, alpha=0.3, linestyle=':')  
    ax1.set_title('Optical Flow Downward (Blue)', fontsize=12, fontweight='bold')
    # Plot Optical Flow Upward
    ax2.plot(t, optflow_up_dx[valid_mask], 'r-', label='dx', linewidth=2)
    ax2.plot(t, optflow_up_dy[valid_mask], 'm-', label='dy', linewidth=2)
    ax2.axhline(y=0, color='gray', linestyle='--', linewidth=0.5, alpha=0.5)
    ax2.set_ylabel('Flow (rad)', fontsize=11)
    ax2.legend(loc='upper right', fontsize=10)
    ax2.grid(True, alpha=0.3, linestyle=':')
    ax2.set_title('Optical Flow Upward (Red)', fontsize=12, fontweight='bold')

    # Plot Altitude and Flow Magnitude
    ax3.plot(t, range_alt[valid_mask], 'orange', label='Range', linewidth=2, alpha=0.8)
    ax3.plot(t, est_alt[valid_mask], 'b-', label='Estimated', linewidth=2)
    
    # Use existing twin axis
    down_mag = np.sqrt(optflow_down_dx[valid_mask]**2 + optflow_down_dy[valid_mask]**2)
    up_mag = np.sqrt(optflow_up_dx[valid_mask]**2 + optflow_up_dy[valid_mask]**2)
    ax3_twin.plot(t, down_mag, 'b:', label='Down Mag', linewidth=1.5, alpha=0.6)
    ax3_twin.plot(t, up_mag, 'r:', label='Up Mag', linewidth=1.5, alpha=0.6)
    
    ax3.set_ylabel('Altitude (m)', color='b', fontsize=11)
    ax3_twin.set_ylabel('Flow Mag (rad)', color='gray', fontsize=10)
    ax3.set_xlabel('Time (s)', fontsize=11)

    ax3.legend(loc='upper left', fontsize=10)
    ax3_twin.legend(loc='upper right', fontsize=9)
    ax3.grid(True, alpha=0.3, linestyle=':')
    ax3.set_title('Altitude & Flow Magnitude', fontsize=12, fontweight='bold')
    
    # Set reasonable Y limits if data exists
    if len(t) > 0:
        alt_max = max(np.max(range_alt[valid_mask]), np.max(est_alt[valid_mask]))
        if alt_max > 0.01:
            ax3.set_ylim([0, min(alt_max * 1.2, 1.0)])  # Cap at 1 meter


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

    # Create figure with 3 subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 9))
    ax3_twin = ax3.twinx()  # Create twin axis ONCE
    
    fig.suptitle('Position Estimation - Optical Flow Viewer (Mode 2)', fontsize=16)
    fig.canvas.mpl_connect('close_event', on_close)

    # Add exit button
    ax_button = plt.axes([0.45, 0.02, 0.1, 0.04])
    btn_exit = Button(ax_button, 'Exit')
    btn_exit.on_clicked(on_exit_button)

    # Adjust layout
    plt.subplots_adjust(left=0.08, right=0.92, top=0.94, bottom=0.08, hspace=0.35)

    # Start animation
    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, update_plot, interval=40, cache_frame_data=False)  # ~25Hz

    plt.show()

    # Cleanup
    running = False
    reader_thread.join(timeout=2)
    print("Application closed")
