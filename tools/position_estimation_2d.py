import serial
import serial.tools.list_ports
import struct
import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
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
# Two vectors/points to visualize (2D only)
vec1 = np.array([0.0, 0.0]) # Pos X, Y (Red)
vec2 = np.array([0.0, 0.0]) # Vel X, Y (Blue)
current_limit = 1.0

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
                        # Extract 2D components:
                        # Pos: X=0, Y=1
                        # Vel: X=3, Y=4
                        pos_xy = (vals[0], vals[1])
                        vel_xy = (vals[3], vals[4])
                        data_queue.put((pos_xy, vel_xy))
                    elif length == 12: # Mode 2 (Bias)
                        # Just ignore or map logic if needed
                        pass
                    
    except Exception as e:
        print(f"Serial error: {e}")

# --- GUI ---
def main():
    global is_collecting, vec1, vec2, view_btns, vis_mode
    
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    
    fig, ax = plt.subplots(figsize=(10, 10))
    plt.subplots_adjust(bottom=0.2)
    
    # Initialize visuals
    # Red: Pos XY
    line1, = ax.plot([], [], color='#FF5555', linewidth=3, label='Pos XY')
    head1, = ax.plot([], [], color='#FF5555', marker='o', markersize=8)
    
    # Blue: Vel XY
    line2, = ax.plot([], [], color='#5555FF', linewidth=3, label='Vel XY')
    head2, = ax.plot([], [], color='#5555FF', marker='o', markersize=8)
    
    text_info = ax.text(0.05, 0.95, "", transform=ax.transAxes, fontsize=12, color='white', verticalalignment='top')
    
    ax.set_aspect('equal')
    ax.grid(True, color='#333333', linestyle='--')
    ax.set_xlabel('X (m or m/s)')
    ax.set_ylabel('Y (m or m/s)')
    ax.legend(loc='upper right')
    ax.set_title('2D Position & Velocity Monitor')
    
    # Initial limit
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)

    # --- Buttons ---

    # Mode Toggle
    ax_mode = plt.axes([0.15, 0.05, 0.2, 0.075])
    btn_mode = Button(ax_mode, 'Mode: Vector', color='#444444', hovercolor='#666666')
    view_btns.append(btn_mode)

    def toggle_mode(event):
        global vis_mode
        vis_mode = 1 - vis_mode # Toggle 0 <-> 1
        btn_mode.label.set_text('Mode: Vector' if vis_mode == 0 else 'Mode: Point')

    btn_mode.on_clicked(toggle_mode)

    # Stream Control
    ax_pause = plt.axes([0.65, 0.05, 0.2, 0.075])
    btn_pause = Button(ax_pause, 'Pause/Resume', color='#444444', hovercolor='#666666')
    view_btns.append(btn_pause)

    def toggle_stream(event):
        global is_collecting
        is_collecting = not is_collecting
        btn_pause.label.set_text('Resume' if not is_collecting else 'Pause')

    btn_pause.on_clicked(toggle_stream)

    def update_plot():
        global vec1, vec2, current_limit
        
        # Consuming queue
        try:
            while not data_queue.empty():
                vals = data_queue.get_nowait()
                if is_collecting:
                    vec1 = np.array(vals[0]) # Pos XY
                    vec2 = np.array(vals[1]) # Vel XY
        except queue.Empty:
            pass
        
        # Auto-scaling logic (expand only for stability)
        max_val = max(np.max(np.abs(vec1)), np.max(np.abs(vec2)))
        if max_val > current_limit * 0.9:
            current_limit = max_val * 1.5
            ax.set_xlim(-current_limit, current_limit)
            ax.set_ylim(-current_limit, current_limit)
        elif max_val < current_limit * 0.2 and current_limit > 1.0:
            pass

        # Visualization
        if vis_mode == 0: # Vector (Line from origin)
            line1.set_data([0, vec1[0]], [0, vec1[1]])
            line2.set_data([0, vec2[0]], [0, vec2[1]])
        else: # Point (Dot only)
            line1.set_data([], [])
            line2.set_data([], [])

        head1.set_data([vec1[0]], [vec1[1]])
        head2.set_data([vec2[0]], [vec2[1]])
        
        # Stats Text
        status_str = (f"POS X: {vec1[0]:.3f} m\n"
                      f"POS Y: {vec1[1]:.3f} m\n"
                      f"VEL X: {vec2[0]:.3f} m/s\n"
                      f"VEL Y: {vec2[1]:.3f} m/s")
        text_info.set_text(status_str)
        
        fig.canvas.draw_idle()
        plt.pause(0.01)

    plt.show(block=False)
    
    while plt.fignum_exists(fig.number):
        update_plot()

if __name__ == '__main__':
    main()
