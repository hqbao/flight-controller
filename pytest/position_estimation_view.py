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
# Vector: Position (Green)
position = np.array([0.0, 0.0, 0.0])
position_offset = np.array([0.0, 0.0, 0.0])
current_limit = 1.0
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
                
                if msg_id == MONITOR_DATA_ID:
                    if length == 12: # 3 floats (vx, vy, vz)
                        vals = struct.unpack('fff', payload)
                        data_queue.put(vals)
                    
    except Exception as e:
        print(f"Serial error: {e}")

# --- GUI ---
def main():
    global is_collecting, position, view_btns
    
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect((1, 1, 1))
    plt.subplots_adjust(bottom=0.2, right=0.75)
    
    # Initialize lines
    # Green: Position
    line_pos, = ax.plot([0, 0], [0, 0], [0, 0], color='green', linewidth=3, label='Position')
    head_pos, = ax.plot([0], [0], [0], color='green', marker='o', markersize=8)
    
    # Text for results
    text_pos = fig.text(0.05, 0.9, "", fontsize=12, color='green')
    
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    ax.set_title('Position Vector Monitor')

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

    # Zero/Tare Button
    ax_zero = plt.axes([0.8, 0.42, 0.15, 0.05])
    btn_zero = Button(ax_zero, 'Zero / Tare')
    view_btns.append(btn_zero)

    def zero_offset(event):
        global position_offset, current_limit
        position_offset = np.copy(position)
        current_limit = 1.0
        ax.set_xlim(-current_limit, current_limit)
        ax.set_ylim(-current_limit, current_limit)
        ax.set_zlim(-current_limit, current_limit)

    btn_zero.on_clicked(zero_offset)

    # Stream Control
    ax_pause = plt.axes([0.8, 0.3, 0.15, 0.05])
    btn_pause = Button(ax_pause, 'Pause/Resume')
    view_btns.append(btn_pause)

    def toggle_stream(event):
        global is_collecting
        is_collecting = not is_collecting
        btn_pause.label.set_text('Resume' if not is_collecting else 'Pause')

    btn_pause.on_clicked(toggle_stream)

    def update():
        global position, position_offset, current_limit
        
        try:
            while not data_queue.empty():
                vals = data_queue.get_nowait()
                if is_collecting:
                    position = np.array(vals)
        except queue.Empty:
            pass
            
        # Calculate displayed position (Raw - Offset)
        display_position = position - position_offset
        
        # Dynamic Scaling
        max_val = np.max(np.abs(display_position))
        target_limit = max(1.0, max_val * 1.2) # Min 1.0m, 20% padding
        
        # Update limit only if we need to expand (scale up)
        if target_limit > current_limit:
            current_limit = target_limit
            ax.set_xlim(-current_limit, current_limit)
            ax.set_ylim(-current_limit, current_limit)
            ax.set_zlim(-current_limit, current_limit)

        # Update Position Vector
        line_pos.set_data([0, display_position[0]], [0, display_position[1]])
        line_pos.set_3d_properties([0, display_position[2]])
        head_pos.set_data([display_position[0]], [display_position[1]])
        head_pos.set_3d_properties([display_position[2]])
        
        text_pos.set_text(f"Position: X={display_position[0]:.2f}, Y={display_position[1]:.2f}, Z={display_position[2]:.2f} m")

    # Animation loop
    plt.show(block=False)
    while plt.fignum_exists(fig.number):
        update()
        plt.pause(0.05)

if __name__ == "__main__":
    main()
