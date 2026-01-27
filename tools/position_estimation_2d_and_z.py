import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib import gridspec
import time

# --- Configuration ---
plt.style.use('dark_background')

SERIAL_PORT = None
BAUD_RATE = 9600
MONITOR_DATA_ID = 0x00  # From logger.c

# Auto-detect serial port
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
    print(f"Found: {port} - {desc}")
    if port.startswith('/dev/cu.usbmodem') or port.startswith('/dev/cu.usbserial') or port.startswith('/dev/cu.SLAB_USBtoUART'):
        SERIAL_PORT = port
        break

if SERIAL_PORT is None:
    print('No serial port found. Please configure manually.')

# --- Global State ---
data_queue = queue.Queue()
is_collecting = True

# Data Points
vec_pos_xy = np.array([0.0, 0.0]) # Pos X, Y
vec_vel_xy = np.array([0.0, 0.0]) # Vel X, Y
val_pos_z = 0.0
val_vel_z = 0.0

xy_limit = 1.0
z_pos_limit = 2.0 # 0 to 2m default
z_vel_limit = 1.0 # -1 to 1m/s default

# Visualization Mode
# 0: Vector (Lines from Origin) - Good for direction
# 1: Point (Dots only) - Good for position
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
                    if length == 24: # 6 floats (Pos XYZ, Vel XYZ)
                        vals = struct.unpack('<ffffff', payload)
                        # Extract components:
                        # Pos: X=0, Y=1, Z=2
                        # Vel: X=3, Y=4, Z=5
                        pos_xy_val = (vals[0], vals[1])
                        pos_z_val = vals[2]
                        vel_xy_val = (vals[3], vals[4])
                        vel_z_val = vals[5]
                        data_queue.put((pos_xy_val, pos_z_val, vel_xy_val, vel_z_val))
                    
    except Exception as e:
        print(f"Serial error: {e}")

# --- GUI ---
def main():
    global is_collecting, vec_pos_xy, vec_vel_xy, val_pos_z, val_vel_z, view_btns, vis_mode
    
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    
    # Setup Figure with GridSpec
    fig = plt.figure(figsize=(14, 8))
    gs = gridspec.GridSpec(2, 3, width_ratios=[1, 1, 0.5]) 
    # Columns 0,1 are for XY Plot (Square aspect)
    # Column 2 is split into Top (Pos Z) and Bottom (Vel Z)

    # --- XY PLOT (Left) ---
    ax_xy = fig.add_subplot(gs[:, 0:2])
    plt.subplots_adjust(bottom=0.15)
    
    line_pos, = ax_xy.plot([], [], color='#FF5555', linewidth=3, label='Pos XY')
    head_pos, = ax_xy.plot([], [], color='#FF5555', marker='o', markersize=8)
    
    line_vel, = ax_xy.plot([], [], color='#5555FF', linewidth=3, label='Vel XY')
    head_vel, = ax_xy.plot([], [], color='#5555FF', marker='o', markersize=8)
    
    text_info = ax_xy.text(0.02, 0.98, "", transform=ax_xy.transAxes, fontsize=12, color='white', verticalalignment='top')
    
    ax_xy.set_aspect('equal')
    ax_xy.grid(True, color='#333333', linestyle='--')
    ax_xy.set_xlabel('X (m or m/s)')
    ax_xy.set_ylabel('Y (m or m/s)')
    ax_xy.legend(loc='upper right')
    ax_xy.set_title('2D Position & Velocity')
    ax_xy.set_xlim(-1, 1)
    ax_xy.set_ylim(-1, 1)

    # --- Z POS PLOT (Top Right) ---
    ax_z_pos = fig.add_subplot(gs[0, 2])
    bar_z_pos = ax_z_pos.bar([0], [0], color='#55FF55', width=0.5)
    ax_z_pos.set_xlim(-0.5, 0.5)
    ax_z_pos.set_ylim(0, 2) # 0 to 2m
    ax_z_pos.set_title('Altitude (Z)')
    ax_z_pos.set_xticks([])
    ax_z_pos.grid(True, axis='y', color='#333333', linestyle='--')
    text_z_pos = ax_z_pos.text(0, 0.1, "0.00m", ha='center', color='white', fontweight='bold')

    # --- Z VEL PLOT (Bottom Right) ---
    ax_z_vel = fig.add_subplot(gs[1, 2])
    bar_z_vel = ax_z_vel.bar([0], [0], color='#FFFF00', width=0.5)
    ax_z_vel.axhline(0, color='white', linewidth=1)
    ax_z_vel.set_xlim(-0.5, 0.5)
    ax_z_vel.set_ylim(-1, 1) # -1 to 1 m/s
    ax_z_vel.set_title('Vertical Velocity')
    ax_z_vel.set_xticks([])
    ax_z_vel.grid(True, axis='y', color='#333333', linestyle='--')
    text_z_vel = ax_z_vel.text(0, 0.1, "0.00m/s", ha='center', color='white', fontweight='bold')


    # --- Buttons ---
    # Mode Toggle
    ax_mode = plt.axes([0.15, 0.02, 0.15, 0.05])
    btn_mode = Button(ax_mode, 'Mode: Vector', color='#444444', hovercolor='#666666')
    view_btns.append(btn_mode)

    def toggle_mode(event):
        global vis_mode
        vis_mode = 1 - vis_mode 
        btn_mode.label.set_text('Mode: Vector' if vis_mode == 0 else 'Mode: Point')

    btn_mode.on_clicked(toggle_mode)

    # Stream Control
    ax_pause = plt.axes([0.35, 0.02, 0.15, 0.05])
    btn_pause = Button(ax_pause, 'Pause/Resume', color='#444444', hovercolor='#666666')
    view_btns.append(btn_pause)

    def toggle_stream(event):
        global is_collecting
        is_collecting = not is_collecting
        btn_pause.label.set_text('Resume' if not is_collecting else 'Pause')

    btn_pause.on_clicked(toggle_stream)
    
    # Reset Limits
    ax_reset = plt.axes([0.55, 0.02, 0.15, 0.05])
    btn_reset = Button(ax_reset, 'Reset Scale', color='#444444', hovercolor='#666666')
    view_btns.append(btn_reset)
    
    def reset_scale(event):
        global xy_limit, z_pos_limit, z_vel_limit
        xy_limit = 1.0
        z_pos_limit = 2.0
        z_vel_limit = 1.0
        ax_xy.set_xlim(-xy_limit, xy_limit)
        ax_xy.set_ylim(-xy_limit, xy_limit)
        ax_z_pos.set_ylim(0, z_pos_limit)
        ax_z_vel.set_ylim(-z_vel_limit, z_vel_limit)
        
    btn_reset.on_clicked(reset_scale)

    def update_plot():
        global vec_pos_xy, vec_vel_xy, val_pos_z, val_vel_z
        global xy_limit, z_pos_limit, z_vel_limit
        
        # Consuming queue
        try:
            while not data_queue.empty():
                vals = data_queue.get_nowait()
                if is_collecting:
                    vec_pos_xy = np.array(vals[0])
                    val_pos_z = vals[1]
                    vec_vel_xy = np.array(vals[2])
                    val_vel_z = vals[3]
        except queue.Empty:
            pass
        
        # --- XY Auto-scaling ---
        max_xy = max(np.max(np.abs(vec_pos_xy)), np.max(np.abs(vec_vel_xy)))
        if max_xy > xy_limit * 0.9:
            xy_limit = max_xy * 1.5
            ax_xy.set_xlim(-xy_limit, xy_limit)
            ax_xy.set_ylim(-xy_limit, xy_limit)
            
        # --- Visualization XY ---
        if vis_mode == 0: # Vector
            line_pos.set_data([0, vec_pos_xy[0]], [0, vec_pos_xy[1]])
            line_vel.set_data([0, vec_vel_xy[0]], [0, vec_vel_xy[1]])
        else: # Point
            line_pos.set_data([], [])
            line_vel.set_data([], [])

        head_pos.set_data([vec_pos_xy[0]], [vec_pos_xy[1]])
        head_vel.set_data([vec_vel_xy[0]], [vec_vel_xy[1]])
        
        status_str = (f"POS XY: ({vec_pos_xy[0]:.2f}, {vec_pos_xy[1]:.2f})\n"
                      f"VEL XY: ({vec_vel_xy[0]:.2f}, {vec_vel_xy[1]:.2f})")
        text_info.set_text(status_str)
        
        # --- Z Visualization ---
        
        # Pos Z Bar
        bar_z_pos[0].set_height(val_pos_z)
        text_z_pos.set_text(f"{val_pos_z:.2f}m")
        text_z_pos.set_y(max(0.1, val_pos_z + 0.1))
        
        # Pos Z Scaling
        if val_pos_z > z_pos_limit * 0.9:
            z_pos_limit = val_pos_z * 1.5
            ax_z_pos.set_ylim(0, z_pos_limit)
            
        # Vel Z Bar
        bar_z_vel[0].set_height(val_vel_z)
        text_z_vel.set_text(f"{val_vel_z:.2f}m/s")
        if val_vel_z >= 0:
            text_z_vel.set_y(val_vel_z + 0.1)
        else:
            text_z_vel.set_y(val_vel_z - 0.2)
            
        # Vel Z Scaling
        if abs(val_vel_z) > z_vel_limit * 0.9:
            z_vel_limit = abs(val_vel_z) * 1.5
            ax_z_vel.set_ylim(-z_vel_limit, z_vel_limit)
        
        fig.canvas.draw_idle()
        plt.pause(0.01)

    plt.show(block=False)
    
    while plt.fignum_exists(fig.number):
        update_plot()

if __name__ == '__main__':
    main()
