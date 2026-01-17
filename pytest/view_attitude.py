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
# Vectors: v_pred (Red), v_true (Blue)
v_pred = np.array([0.0, 0.0, 0.0])
v_true = np.array([0.0, 0.0, 0.0])

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
                    if length == 24: # 6 floats
                        vals = struct.unpack('ffffff', payload)
                        # v_pred (0-2), v_true (3-5)
                        data_queue.put(vals)
                    
    except Exception as e:
        print(f"Serial error: {e}")

# --- GUI ---
def main():
    global is_collecting, v_pred, v_true
    
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect((1, 1, 1))
    plt.subplots_adjust(bottom=0.2, right=0.75)
    
    # Initialize lines
    # Red: Prediction, Blue: True (Measurement)
    line_pred, = ax.plot([0, 0], [0, 0], [0, 0], color='red', linewidth=3, label='Prediction')
    head_pred, = ax.plot([0], [0], [0], color='red', marker='o', markersize=8)
    
    line_true, = ax.plot([0, 0], [0, 0], [0, 0], color='blue', linewidth=3, label='True (Accel)')
    head_true, = ax.plot([0], [0], [0], color='blue', marker='o', markersize=8)
    
    # Text for results
    text_ax = plt.axes([0.76, 0.2, 0.23, 0.6])
    text_ax.axis('off')
    info_text = text_ax.text(0, 1, "Waiting for data...", fontsize=10, va='top', fontfamily='monospace')
    
    # --- Buttons ---
    # View Buttons
    views = {
        'Top': (90, -90), 'Bottom': (-90, -90),
        'Front': (0, -90), 'Back': (0, 90),
        'Left': (0, 180), 'Right': (0, 0)
    }
    
    start_x = 0.05
    width = 0.06
    gap = 0.01
    row1_y = 0.1
    
    view_btns = []
    for i, (label, (elev, azim)) in enumerate(views.items()):
        b_ax = plt.axes([start_x + i*(width+gap), row1_y, width, 0.05])
        b = Button(b_ax, label)
        b.on_clicked(lambda event, e=elev, a=azim: (ax.view_init(elev=e, azim=a), plt.draw()))
        view_btns.append(b)

    # Stream Toggle
    btn_stream_ax = plt.axes([start_x, 0.04, 0.15, 0.05])
    btn_stream = Button(btn_stream_ax, 'Stop Stream')
    
    def toggle_stream(event):
        global is_collecting
        is_collecting = not is_collecting
        btn_stream.label.set_text('Start Stream' if not is_collecting else 'Stop Stream')
        
    btn_stream.on_clicked(toggle_stream)
    
    # Setup Axes
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    ax.set_title("Attitude Fusion Vectors")
    
    while True:
        # Process all pending data
        latest_vals = None
        while not data_queue.empty():
            latest_vals = data_queue.get()
        
        if is_collecting and latest_vals:
            v_pred = np.array(latest_vals[0:3])
            v_true = np.array(latest_vals[3:6])
            
            # Update plots
            line_pred.set_data([0, v_pred[0]], [0, v_pred[1]])
            line_pred.set_3d_properties([0, v_pred[2]])
            head_pred.set_data([v_pred[0]], [v_pred[1]])
            head_pred.set_3d_properties([v_pred[2]])
            
            line_true.set_data([0, v_true[0]], [0, v_true[1]])
            line_true.set_3d_properties([0, v_true[2]])
            head_true.set_data([v_true[0]], [v_true[1]])
            head_true.set_3d_properties([v_true[2]])
            
            # Update text
            info_text.set_text(
                f"Prediction (Red):\n"
                f"X: {v_pred[0]:.3f}\n"
                f"Y: {v_pred[1]:.3f}\n"
                f"Z: {v_pred[2]:.3f}\n"
                f"Mag: {np.linalg.norm(v_pred):.3f}\n\n"
                f"True/Accel (Blue):\n"
                f"X: {v_true[0]:.3f}\n"
                f"Y: {v_true[1]:.3f}\n"
                f"Z: {v_true[2]:.3f}\n"
                f"Mag: {np.linalg.norm(v_true):.3f}"
            )

        plt.pause(0.05)
        if not plt.fignum_exists(fig.number):
            break

if __name__ == "__main__":
    main()
