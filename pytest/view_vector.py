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
# 3D quiver will be created on first update
q = None

# Data queue for inter-thread communication
queue1 = queue.Queue()

# Global variables to store parsed 3D vector data
g_vec_x = 0.0
g_vec_y = 0.0
g_vec_z = 0.0
g_angle_deg = 0.0

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
    Expects 3x float32 little-endian payloads (x, y, z components).
    """
    global g_vec_x, g_vec_y
    while True:
        try:
            clazz, ID, payload = queue1.get(timeout=0.01) # Small timeout to allow main thread to run

            # Accept MONITOR_DATA formatted payloads: 3x float32 (x,y,z)
            # Logger sends messages with Class=0x00 and a 12-byte payload for vector data.
            if clazz == 0x00:
                if len(payload) == 12:
                    # payload is 3x float32 little-endian (raw values)
                    x_f, y_f, z_f = struct.unpack('<fff', payload)
                    new_x = float(x_f)
                    new_y = float(y_f)
                    new_z = float(z_f)

                    # Validate that values are within expected range [-1000, 1000]
                    if abs(new_x) > 1000.0 or abs(new_y) > 1000.0 or abs(new_z) > 1000.0:
                        continue

                    # Directly assign the new values
                    global g_vec_x, g_vec_y, g_vec_z, g_angle_deg
                    g_vec_x = new_x
                    g_vec_y = new_y
                    g_vec_z = new_z

                    # Angle in degrees (0..360), atan2(y,x)
                    g_angle_deg = (math.degrees(math.atan2(g_vec_y, g_vec_x)) + 360.0) % 360.0
            # Add other message ID handlers here if needed
            
        except queue.Empty:
            pass
        except struct.error as e:
            print(f"Struct unpacking error in parser: {e}")
        except Exception as e:
            print(f"General parser error: {e}")

def update(frame):
    """
    This function is called by the animation to update the 3D vector plot.
    """
    global q, g_vec_x, g_vec_y, g_vec_z

    u = g_vec_x
    v = g_vec_y
    w = g_vec_z

    # Remove previous quiver and draw a new one
    try:
        if q is not None:
            q.remove()
    except Exception:
        pass

    # origin at (0,0,0), direction (u,v,w)
    q = ax.quiver([0], [0], [0], [u], [v], [w], length=0.8, normalize=False)

    # Update axes and labels
    ax.set_xlim(-1000, 1000)
    ax.set_ylim(-1000, 1000)
    ax.set_zlim(-1000, 1000)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Update title with angle and magnitude
    magnitude = math.hypot(g_vec_x, g_vec_y, g_vec_z)
    ax.set_title(f'Angle: {g_angle_deg:.1f}° — |vec|={magnitude:.3f}')
    return q,

def main():
    """
    Main function to set up the 3D vector plot and animation.
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
    ax.set_title('3D Vector Viewer')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Create the animation
    ani = animation.FuncAnimation(fig, update, blit=False, interval=50, cache_frame_data=False) # interval in ms

    # Show the plot
    plt.show()

if __name__ == '__main__':
    main()
