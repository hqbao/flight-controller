import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import queue
from threading import Thread
import serial.tools.list_ports
import math
import numpy as np

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600

# Auto-detect serial port
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
    # print(f"{port}: {desc} [{hwid}]")
    if port.startswith('/dev/cu.usbmodem') or port.startswith('/dev/cu.usbserial') or port.startswith('/dev/cu.SLAB_USBtoUART'):
        SERIAL_PORT = port
        break

if SERIAL_PORT is None:
    print('No serial port found. Please update SERIAL_PORT in the script.')
    exit()
# --- End Configuration ---

# Global variables
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# 3D quiver will be created on first update
q = None

# Data queue for inter-thread communication
queue1 = queue.Queue()

# Global variables to store parsed compass data
g_mag_x = 0.0
g_mag_y = 0.0
g_mag_z = 0.0
g_mag_x_s = 0.0
g_mag_y_s = 0.0
g_mag_z_s = 0.0
g_heading_deg = 0.0

# Visualization settings
SMOOTHING_ALPHA = 0.3  # exponential smoothing factor (0..1)
NORMALIZE_ARROW = True

def run_db_reader(in_queue):
    """
    Reads binary data from the serial port and puts it into a queue.
    Expects messages framed by the logger module: db ID Class Length Payload Checksum
    """
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=3) as stream:
        while True:
            try:
                    # Scan for the 'db' header bytes one byte at a time to avoid blocking
                    found = False
                    while True:
                        b1 = stream.read(1)
                        if not b1:
                            break
                        if b1 == b'd':
                            b2 = stream.read(1)
                            if not b2:
                                break
                            if b2 == b'b':
                                found = True
                                break
                            # if b2 is 'd' there may be overlapping 'd's; continue scanning
                            if b2 == b'd':
                                # treat this read as the next candidate 'd' by continuing
                                continue
                            # otherwise keep scanning
                            continue
                    if not found:
                        continue

                    # Read message fields after header
                    msg_id_b = stream.read(1)
                    if not msg_id_b:
                        continue
                    msg_id = msg_id_b[0]

                    msg_class_b = stream.read(1)
                    if not msg_class_b:
                        continue
                    msg_class = msg_class_b[0]

                    length_bytes = stream.read(2)
                    if len(length_bytes) < 2:
                        continue
                    payload_length = int.from_bytes(length_bytes, byteorder='little')

                    payload = stream.read(payload_length)
                    if len(payload) < payload_length:
                        # incomplete payload, skip
                        continue
                    checksum = stream.read(2) # Read and ignore checksum for now

                    in_queue.put((msg_id, msg_class, payload))
            except serial.SerialException as e:
                print(f"Serial reader error: {e}")
                # break
            except Exception as e:
                print(f"General reader error: {e}")
                # break

def run_parser():
    """
    Parses messages from the queue and updates global compass data.
    """
    global g_mag_x, g_mag_y
    while True:
        try:
            msg_id, msg_class, payload = queue1.get(timeout=0.01) # Small timeout to allow main thread to run

            # Accept MONITOR_DATA formatted payloads: 3x int32 (x,y,z) scaled by 1000
            # Logger sends messages with ID=0x00, Class=0x00 and a 12-byte payload for monitor data.
            # Accept either ordering (ID==0x00 or Class==0x00) to be tolerant.
            if msg_class == 0x00 or msg_id == 0x00:
                if len(payload) == 12:
                    x_scaled, y_scaled, z_scaled = struct.unpack('<iii', payload)
                    new_x = x_scaled / 1000.0
                    new_y = y_scaled / 1000.0
                    new_z = z_scaled / 1000.0

                    # Exponential smoothing to reduce jitter
                    global g_mag_x_s, g_mag_y_s, g_mag_z_s, g_mag_x, g_mag_y, g_mag_z, g_heading_deg
                    g_mag_x_s = SMOOTHING_ALPHA * new_x + (1.0 - SMOOTHING_ALPHA) * g_mag_x_s
                    g_mag_y_s = SMOOTHING_ALPHA * new_y + (1.0 - SMOOTHING_ALPHA) * g_mag_y_s
                    g_mag_z_s = SMOOTHING_ALPHA * new_z + (1.0 - SMOOTHING_ALPHA) * g_mag_z_s

                    g_mag_x = g_mag_x_s
                    g_mag_y = g_mag_y_s
                    g_mag_z = g_mag_z_s

                    # Heading in degrees (0..360), atan2(y,x)
                    g_heading_deg = (math.degrees(math.atan2(g_mag_y, g_mag_x)) + 360.0) % 360.0
                    # print(f"Parsed MAG: x={g_mag_x:.2f}, y={g_mag_y:.2f}, z={g_mag_z:.2f}, heading={g_heading_deg:.1f}°")
                else:
                    print(f"Parser: Incomplete compass payload (expected 12 bytes, got {len(payload)})")
            # Add other message ID handlers here if needed
            
        except queue.Empty:
            pass
        except struct.error as e:
            print(f"Struct unpacking error in parser: {e}")
        except Exception as e:
            print(f"General parser error: {e}")

def update(frame):
    """
    This function is called by the animation to update the plot (3D).
    """
    global q, g_mag_x, g_mag_y, g_mag_z

    ux = g_mag_x
    uy = g_mag_y
    uz = g_mag_z

    # Normalize the vector if requested
    if NORMALIZE_ARROW:
        norm = math.hypot(ux, uy, uz)
        if norm < 1e-6:
            u = v = w = 0.0
        else:
            u = ux / norm
            v = uy / norm
            w = uz / norm
    else:
        u = ux
        v = uy
        w = uz

    # Remove previous quiver and draw a new one
    try:
        if q is not None:
            q.remove()
    except Exception:
        pass

    # origin at (0,0,0), direction (u,v,w)
    q = ax.quiver([0], [0], [0], [u], [v], [w], length=0.8, normalize=False)

    # Update axes and labels
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Update title with heading and magnitude
    mag = math.hypot(g_mag_x, g_mag_y, g_mag_z)
    ax.set_title(f'Heading: {g_heading_deg:.1f}° — |mag|={mag:.3f}')
    return q,

def main():
    """
    Main function to set up the plot and animation.
    """
    # Start the serial reader and parser threads
    reader_thread = Thread(target=run_db_reader, args=(queue1,))
    reader_thread.daemon = True # Allow main program to exit even if thread is still running
    reader_thread.start()

    parser_thread = Thread(target=run_parser, args=())
    parser_thread.daemon = True
    parser_thread.start()


    # Setup 3D axes limits and labels
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    try:
        ax.set_zlim(-1, 1)
    except Exception:
        pass
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.set_title('Compass Heading (3D)')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Create the animation
    ani = animation.FuncAnimation(fig, update, blit=False, interval=50, cache_frame_data=False) # interval in ms

    # Show the plot
    plt.show()

if __name__ == '__main__':
    main()