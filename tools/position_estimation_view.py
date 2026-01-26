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

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
MONITOR_DATA_ID = 0x00  # From logger.c

# Auto-detect serial port
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
    if port.startswith('/dev/cu.usbmodem') or port.startswith('/dev/cu.usbserial') or port.startswith('/dev/cu.SLAB_USBtoUART'):
        SERIAL_PORT = port
        break

if SERIAL_PORT is None:
    print('No serial port found. Please configure manually.')

# --- Global State ---
data_queue = queue.Queue()
is_collecting = True
# Two vectors/points to visualize
vec1 = np.array([0.0, 0.0, 0.0]) # EST0 or VELOC0 (Red)
vec2 = np.array([0.0, 0.0, 0.0]) # EST1 or VELOC1 (Blue)
current_limit = 1.0

# Visualization Mode
# 0: Vector (Lines from Origin) - Good for Velocity
# 1: Point (Dots only) - Good for Position
vis_mode = 0 

view_btns = [] # Keep references to buttons

# --- Serial Reader ---
def serial_reader():
    global SERIAL_PORT
    if not SERIAL_PORT:
        return

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Connected to {SERIAL_PORT}")
            while True:
                # Header: 'db' or 'bd' (0x64 0x62)
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

                checksum = ser.read(2) # Read checksum to advance stream
                
                if msg_id == MONITOR_DATA_ID:
                    if length == 24: # 6 floats (v1_x,y,z, v2_x,y,z)
                        vals = struct.unpack('<ffffff', payload)
                        data_queue.put(vals)
                    elif length == 12: # Fallback for old mode 3
                        vals = struct.unpack('<fff', payload)
                        # Padding with zeros for vec2
                        data_queue.put(vals + (0.0, 0.0, 0.0))
                    
    except Exception as e:
        print(f"Serial error: {e}")

# --- GUI ---
def main():
    global is_collecting, vec1, vec2, view_btns, vis_mode
    
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect((1, 1, 1))
    plt.subplots_adjust(bottom=0.2, right=0.75)
    
    # Initialize visuals
    # Red: EST0 / VELOC0
    line1, = ax.plot([], [], [], color='red', linewidth=2, label='EST0 / VELOC0')
    head1, = ax.plot([], [], [], color='red', marker='o', markersize=6)
    
    # Blue: EST1 / VELOC1
    line2, = ax.plot([], [], [], color='blue', linewidth=2, label='EST1 / VELOC1')
    head2, = ax.plot([], [], [], color='blue', marker='o', markersize=6) # Slightly larger head for primary?
    
    # Text for results
    text_info = fig.text(0.05, 0.9, "", fontsize=12)
    
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend(loc='upper right')
    ax.set_title('Dual Vector Monitor')

    # --- Buttons ---
    # View Buttons
    ax_xy = plt.axes([0.8, 0.7, 0.15, 0.05])
    btn_xy = Button(ax_xy, 'Top View (XY)')
    view_btns.append(btn_xy)

    ax_xz = plt.axes([0.8, 0.63, 0.15, 0.05])
    btn_xz = Button(ax_xz, 'Side View (XZ)')
    view_btns.append(btn_xz)

    ax_yz = plt.axes([0.8, 0.56, 0.15, 0.05])
    btn_yz = Button(ax_yz, 'Front View (YZ)')
    view_btns.append(btn_yz)
    
    ax_iso = plt.axes([0.8, 0.49, 0.15, 0.05])
    btn_iso = Button(ax_iso, 'Reset View')
    view_btns.append(btn_iso)

    def set_view_xy(event):
        ax.view_init(elev=90, azim=-90)
        plt.draw()

    def set_view_xz(event):
        ax.view_init(elev=0, azim=-90)
        plt.draw()

    def set_view_yz(event):
        ax.view_init(elev=0, azim=0)
        plt.draw()
        
    def set_view_iso(event):
        ax.view_init(elev=30, azim=-60)
        plt.draw()

    btn_xy.on_clicked(set_view_xy)
    btn_xz.on_clicked(set_view_xz)
    btn_yz.on_clicked(set_view_yz)
    btn_iso.on_clicked(set_view_iso)

    # Mode Toggle
    ax_mode = plt.axes([0.8, 0.35, 0.15, 0.05])
    btn_mode = Button(ax_mode, 'Mode: Vector')
    view_btns.append(btn_mode)

    def toggle_mode(event):
        global vis_mode
        vis_mode = 1 - vis_mode # Toggle 0 <-> 1
        btn_mode.label.set_text('Mode: Vector' if vis_mode == 0 else 'Mode: Point')

    btn_mode.on_clicked(toggle_mode)

    # Stream Control
    ax_pause = plt.axes([0.8, 0.28, 0.15, 0.05])
    btn_pause = Button(ax_pause, 'Pause/Resume')
    view_btns.append(btn_pause)

    def toggle_stream(event):
        global is_collecting
        is_collecting = not is_collecting
        btn_pause.label.set_text('Resume' if not is_collecting else 'Pause')

    btn_pause.on_clicked(toggle_stream)

    def update():
        global vec1, vec2, current_limit
        
        try:
            while not data_queue.empty():
                vals = data_queue.get_nowait()
                if is_collecting:
                    vec1 = np.array(vals[0:3])
                    vec2 = np.array(vals[3:6])
        except queue.Empty:
            pass
            
        # Draw Logic
        if vis_mode == 0: # Vector Mode (Line from Origin)
            line1.set_data([0, vec1[0]], [0, vec1[1]])
            line1.set_3d_properties([0, vec1[2]])
            
            line2.set_data([0, vec2[0]], [0, vec2[1]])
            line2.set_3d_properties([0, vec2[2]])
        else: # Point Mode (Just the point)
            # To hide lines, we can set them to be the same point or empty. 
            # Empty is better but difficult to manage with set_data.
            # Setting start=end works to hide it (zero length line).
            line1.set_data([vec1[0], vec1[0]], [vec1[1], vec1[1]]) 
            line1.set_3d_properties([vec1[2], vec1[2]])
            
            line2.set_data([vec2[0], vec2[0]], [vec2[1], vec2[1]])
            line2.set_3d_properties([vec2[2], vec2[2]])

        # Heads (Markers) always at tip
        head1.set_data([vec1[0]], [vec1[1]])
        head1.set_3d_properties([vec1[2]])
        
        head2.set_data([vec2[0]], [vec2[1]])
        head2.set_3d_properties([vec2[2]])
        
        # Dynamic Scaling
        max_val = max(np.max(np.abs(vec1)), np.max(np.abs(vec2)))
        target_limit = max(1.0, max_val * 1.2) # Min 1.0m, 20% padding
        
        # Update limit only if we need to expand (scale up)
        if target_limit > current_limit:
            current_limit = target_limit
            ax.set_xlim(-current_limit, current_limit)
            ax.set_ylim(-current_limit, current_limit)
            ax.set_zlim(-current_limit, current_limit)

        text_info.set_text(
            f"RED (0): {vec1[0]:.2f}, {vec1[1]:.2f}, {vec1[2]:.2f}\n"
            f"BLUE (1): {vec2[0]:.2f}, {vec2[1]:.2f}, {vec2[2]:.2f}"
        )

    # Animation loop
    plt.show(block=False)
    while plt.fignum_exists(fig.number):
        update()
        plt.pause(0.05)

if __name__ == "__main__":
    main()
