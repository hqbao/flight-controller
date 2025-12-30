import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import queue
from threading import Thread
import serial.tools.list_ports
from serial import Serial
import math
import numpy as np

# --- Configuration ---
g_serial_port = None
g_baud_rate = 9600

# Auto-detect serial port
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
    # print(f"{port}: {desc} [{hwid}]")
    if port.startswith('/dev/cu.usbmodem') or port.startswith('/dev/cu.usbserial') or port.startswith('/dev/cu.SLAB_USBtoUART'):
        g_serial_port = port
        break

if g_serial_port is None:
    print('No serial port found. Please update g_serial_port in the script.')
    exit()
# --- End Configuration ---

# Global variables
ser = serial.Serial(g_serial_port, g_baud_rate)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# 3D quivers will be created on first update
q1 = None
q2 = None

# Data queue for inter-thread communication
queue1 = queue.Queue()

# Global variables to store parsed 3D vector data (vector 1 and vector 2)
g_vec1_x = 0.0
g_vec1_y = 0.0
g_vec1_z = 0.0
g_vec2_x = 0.0
g_vec2_y = 0.0
g_vec2_z = 0.0

def run_db_reader(in_queue):
  with Serial(g_serial_port, g_baud_rate, timeout=3) as stream:
    while True:
      try:
        byte = stream.read(1)
        if len(byte) != 1:
          continue
          
        if byte[0] == 0x62 or byte[0] == 0x64: # 'b' or 'd'
          byte2 = stream.read(1)
          if len(byte2) == 0:
            continue
          
          # Verify we have the full header 'db' or 'bd'
          if not ((byte[0] == 0x64 and byte2[0] == 0x62) or (byte[0] == 0x62 and byte2[0] == 0x64)):
            continue
          
          if byte2[0] == 0x64 or byte2[0] == 0x62: # 'd' or 'b'
            # Firmware logger.c sends: 'd', 'b', ID (byte 2), CLASS (byte 3), length_lo, length_hi, payload, checksum
            ID_bytes = stream.read(1)
            if len(ID_bytes) == 0:
              continue
            ID = ID_bytes[0]
            
            clazz_bytes = stream.read(1)
            if len(clazz_bytes) == 0:
              continue
            clazz = clazz_bytes[0]
            
            length = stream.read(2)
            if len(length) < 2:
              continue
            size = int.from_bytes(length, 'little')
            
            # Validate size is reasonable (prevent reading too much)
            if size > 1024 or size == 0:
              continue
              
            payload = stream.read(size)
            if len(payload) < size:
              continue
              
            checksum = stream.read(2)
            if len(checksum) < 2:
              continue
            
            # Calculate checksum: sum of ID + CLASS + length bytes + payload bytes
            calc_sum = ID + clazz + length[0] + length[1]
            for b in payload:
              calc_sum += b
            calc_sum = calc_sum & 0xFFFF  # 16-bit
            
            recv_sum = int.from_bytes(checksum, 'little')
            
            if recv_sum != 0:  # Only validate if firmware sends non-zero checksum
              if recv_sum != calc_sum:
                continue
            
            in_queue.put((clazz, ID, payload))
      except Exception as e:
        print(f"Reader error: {e}")
        continue

def run_parser():
    """
    Parses messages from the queue and updates global vector data.
    Supports:
    - 12-byte payloads: 3x float32 (single vector)
    - 24-byte payloads: 6x float32 (two vectors)
    """
    global g_vec1_x, g_vec1_y, g_vec1_z, g_vec2_x, g_vec2_y, g_vec2_z
    while True:
        try:
            clazz, ID, payload = queue1.get(timeout=0.01)

            # Accept MONITOR_DATA formatted payloads with Class=0x00
            if clazz == 0x00:
                if len(payload) == 12:
                    # Single vector: 3x float32
                    x_f, y_f, z_f = struct.unpack('<fff', payload)
                    new_x = float(x_f)
                    new_y = float(y_f)
                    new_z = float(z_f)

                    # Validate that values are within expected range [-1, 1]
                    if abs(new_x) > 1.0 or abs(new_y) > 1.0 or abs(new_z) > 1.0:
                        continue

                    # Assign to vector 1
                    g_vec1_x = new_x
                    g_vec1_y = new_y
                    g_vec1_z = new_z
                    
                elif len(payload) == 24:
                    # Two vectors: 6x float32 (v1.x, v1.y, v1.z, v2.x, v2.y, v2.z)
                    x1_f, y1_f, z1_f, x2_f, y2_f, z2_f = struct.unpack('<ffffff', payload)
                    new_x1 = float(x1_f)
                    new_y1 = float(y1_f)
                    new_z1 = float(z1_f)
                    new_x2 = float(x2_f)
                    new_y2 = float(y2_f)
                    new_z2 = float(z2_f)
                    print(f"Received vectors: V1=({new_x1:.3f}, {new_y1:.3f}, {new_z1:.3f}), V2=({new_x2:.3f}, {new_y2:.3f}, {new_z2:.3f})")

                    # Validate that values are within expected range [-1, 1]
                    if (abs(new_x1) > 1.0 or abs(new_y1) > 1.0 or abs(new_z1) > 1.0 or
                        abs(new_x2) > 1.0 or abs(new_y2) > 1.0 or abs(new_z2) > 1.0):
                        continue

                    # Assign to both vectors
                    g_vec1_x = new_x1
                    g_vec1_y = new_y1
                    g_vec1_z = new_z1
                    g_vec2_x = new_x2
                    g_vec2_y = new_y2
                    g_vec2_z = new_z2
            
        except queue.Empty:
            pass
        except struct.error as e:
            print(f"Struct unpacking error in parser: {e}")
        except Exception as e:
            print(f"General parser error: {e}")

def update(frame):
    """
    This function is called by the animation to update the 3D vector plot.
    Displays two vectors in different colors.
    """
    global q1, q2, g_vec1_x, g_vec1_y, g_vec1_z, g_vec2_x, g_vec2_y, g_vec2_z

    # Remove previous quivers
    try:
        if q1 is not None:
            q1.remove()
        if q2 is not None:
            q2.remove()
    except Exception:
        pass

    # Draw vector 1 in red
    q1 = ax.quiver([0], [0], [0], [g_vec1_x], [g_vec1_y], [g_vec1_z], 
                    length=0.8, normalize=False, color='red', alpha=0.8, arrow_length_ratio=0.2)
    
    # Draw vector 2 in blue
    q2 = ax.quiver([0], [0], [0], [g_vec2_x], [g_vec2_y], [g_vec2_z], 
                    length=0.8, normalize=False, color='blue', alpha=0.8, arrow_length_ratio=0.2)

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

    # Update title with magnitudes
    mag1 = math.hypot(g_vec1_x, g_vec1_y, g_vec1_z)
    mag2 = math.hypot(g_vec2_x, g_vec2_y, g_vec2_z)
    ax.set_title(f'Vector 1 (red): |vec|={mag1:.3f} | Vector 2 (blue): |vec|={mag2:.3f}')
    return q1, q2,

def main():
    """
    Main function to set up the 3D vector plot and animation.
    """
    # Start the serial reader and parser threads
    reader_thread = Thread(target=run_db_reader, args=(queue1,))
    reader_thread.daemon = True
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
    ax.set_title('3D Vectors Viewer')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Create the animation
    ani = animation.FuncAnimation(fig, update, blit=False, interval=50, cache_frame_data=False)

    # Show the plot
    plt.show()

if __name__ == '__main__':
    main()
