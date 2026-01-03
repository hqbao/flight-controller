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
PLOT_LIMIT = 75

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
raw_data_points = []
is_collecting = False
calibration_result = (np.zeros(3), np.eye(3))
last_calibration_time = 0

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
                
                if msg_id == MONITOR_DATA_ID and length == 12:
                    x, y, z = struct.unpack('fff', payload)
                    if abs(x) <= 500 and abs(y) <= 500 and abs(z) <= 500:
                        data_queue.put((x, y, z))
                    
    except Exception as e:
        print(f"Serial error: {e}")

# --- Calibration Algorithm ---
def calibrate_magnetometer(data):
    """
    Performs Hard Iron and Soft Iron calibration using ellipsoid fitting.
    Returns:
        B: Hard Iron Bias (3,)
        S: Soft Iron Matrix (3,3)
    """
    if len(data) < 10:
        return np.zeros(3), np.eye(3)

    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]

    # Ellipsoid Fit: Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
    D = np.array([x**2, y**2, z**2, x*y, x*z, y*z, x, y, z]).T
    ones = np.ones((len(data), 1))
    
    try:
        v, _, _, _ = np.linalg.lstsq(D, ones, rcond=None)
    except Exception:
        return None
        
    if v is None or np.isnan(v).any():
        return None
        
    v = v.flatten()
    
    A, B, C = v[0], v[1], v[2]
    D, E, F = v[3]/2, v[4]/2, v[5]/2
    G, H, I = v[6]/2, v[7]/2, v[8]/2
    
    A_mat = np.array([
        [A, D, E],
        [D, B, F],
        [E, F, C]
    ])
    
    B_vec = np.array([2*G, 2*H, 2*I])
    
    try:
        A_inv = np.linalg.inv(A_mat)
        center = -0.5 * np.dot(A_inv, B_vec)
    except np.linalg.LinAlgError:
        return None
        
    scale = 1 + np.dot(center.T, np.dot(A_mat, center))
    if scale <= 0:
        return None
        
    A_mat_scaled = A_mat / scale
    
    vals, vecs = np.linalg.eigh(A_mat_scaled)
    vals = np.abs(vals)
    
    sqrt_vals = np.sqrt(vals)
    D_sqrt = np.diag(sqrt_vals)
    
    # Calculate symmetric S = V * D_sqrt * V.T to preserve orientation
    S = np.dot(vecs, np.dot(D_sqrt, vecs.T))
    
    return center, S

# --- GUI ---
def main():
    global is_collecting, raw_data_points, calibration_result, last_calibration_time
    
    # State flags
    show_raw = True
    show_calib = True
    run_calibration = True
    
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect((1, 1, 1))
    plt.subplots_adjust(bottom=0.25, right=0.75) # Increased bottom margin for buttons
    
    scat_raw = ax.scatter([], [], [], c='r', marker='o', s=5, label='Raw Data')
    scat_corr = ax.scatter([], [], [], c='g', marker='o', s=5, label='Corrected Data')
    
    # Text for results
    text_ax = plt.axes([0.76, 0.2, 0.23, 0.6])
    text_ax.axis('off')
    info_text = text_ax.text(0, 1, "Press Start to collect data...", fontsize=9, va='top', fontfamily='monospace')
    
    # --- Button Layout ---
    # Row 1: Views (Top, Bottom, Front, Back, Left, Right)
    # Row 2: Toggles (Raw, Calib, Algo, Stream)
    # Row 3: Actions (Reset B/S, Clear Data)
    
    # Row 1: Views
    views = {
        'Top': (90, -90), 'Bottom': (-90, -90),
        'Front': (0, -90), 'Back': (0, 90),
        'Left': (0, 180), 'Right': (0, 0)
    }
    view_btns = []
    start_x = 0.05
    width = 0.06
    gap = 0.01
    row1_y = 0.13
    
    for i, (label, (elev, azim)) in enumerate(views.items()):
        b_ax = plt.axes([start_x + i*(width+gap), row1_y, width, 0.05])
        b = Button(b_ax, label)
        b.on_clicked(lambda event, e=elev, a=azim: (ax.view_init(elev=e, azim=a), plt.draw()))
        view_btns.append(b)

    # Row 2: Toggles
    row2_y = 0.07
    
    # Toggle Raw
    btn_raw_ax = plt.axes([start_x, row2_y, width*1.5, 0.05])
    btn_raw = Button(btn_raw_ax, 'Hide Raw')
    def toggle_raw(event):
        nonlocal show_raw
        show_raw = not show_raw
        scat_raw.set_visible(show_raw)
        btn_raw.label.set_text('Hide Raw' if show_raw else 'Show Raw')
        plt.draw()
    btn_raw.on_clicked(toggle_raw)
    
    # Toggle Calib
    btn_calib_ax = plt.axes([start_x + width*1.5 + gap, row2_y, width*1.5, 0.05])
    btn_calib = Button(btn_calib_ax, 'Hide Calib')
    def toggle_calib(event):
        nonlocal show_calib
        show_calib = not show_calib
        scat_corr.set_visible(show_calib)
        btn_calib.label.set_text('Hide Calib' if show_calib else 'Show Calib')
        plt.draw()
    btn_calib.on_clicked(toggle_calib)
    
    # Toggle Algo
    btn_algo_ax = plt.axes([start_x + (width*1.5 + gap)*2, row2_y, width*1.5, 0.05])
    btn_algo = Button(btn_algo_ax, 'Stop Algo')
    def toggle_algo(event):
        nonlocal run_calibration
        run_calibration = not run_calibration
        btn_algo.label.set_text('Stop Algo' if run_calibration else 'Run Algo')
    btn_algo.on_clicked(toggle_algo)
    
    # Toggle Stream (Collection)
    btn_stream_ax = plt.axes([start_x + (width*1.5 + gap)*3, row2_y, width*1.5, 0.05])
    btn_stream = Button(btn_stream_ax, 'Start Stream') # Default off
    def toggle_stream(event):
        global is_collecting
        is_collecting = not is_collecting
        btn_stream.label.set_text('Stop Stream' if is_collecting else 'Start Stream')
    btn_stream.on_clicked(toggle_stream)

    # Row 3: Actions
    row3_y = 0.01
    
    # Reset B/S
    btn_reset_ax = plt.axes([start_x, row3_y, width*2, 0.05])
    btn_reset = Button(btn_reset_ax, 'Reset B & S')
    def reset_calibration(event):
        global calibration_result
        calibration_result = (np.zeros(3), np.eye(3))
        B, S = calibration_result
        
        B_str = f"[{B[0]:.2f}, {B[1]:.2f}, {B[2]:.2f}]"
        S_str = "\n".join([f"[{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}]" for r in S])
        info_text.set_text(f"Points: {len(raw_data_points)}\n\nHard Iron Bias B:\n{B_str}\n\nSoft Iron Matrix S:\n{S_str}")
        
        # Reset corrected plot to raw (if visible)
        if len(raw_data_points) > 0:
            data_np = np.array(raw_data_points)
            scat_corr._offsets3d = (data_np[:, 0], data_np[:, 1], data_np[:, 2])
            plt.draw()
        print("Calibration parameters reset.")
    btn_reset.on_clicked(reset_calibration)
    
    # Clear Data
    btn_clear_ax = plt.axes([start_x + width*2 + gap, row3_y, width*2, 0.05])
    btn_clear = Button(btn_clear_ax, 'Clear All Points')
    def clear_points(event):
        global raw_data_points
        raw_data_points = []
        scat_raw._offsets3d = ([], [], [])
        scat_corr._offsets3d = ([], [], [])
        print("Points cleared.")
        plt.draw()
    btn_clear.on_clicked(clear_points)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    while True:
        points_added = False
        while not data_queue.empty():
            pt = data_queue.get()
            if is_collecting:
                raw_data_points.append(pt)
                points_added = True
        
        if points_added:
            data_np = np.array(raw_data_points)
            scat_raw._offsets3d = (data_np[:, 0], data_np[:, 1], data_np[:, 2])
            
            # Fixed scale
            ax.set_xlim(-PLOT_LIMIT, PLOT_LIMIT)
            ax.set_ylim(-PLOT_LIMIT, PLOT_LIMIT)
            ax.set_zlim(-PLOT_LIMIT, PLOT_LIMIT)
            
            # Realtime calibration every 1s
            if run_calibration and time.time() - last_calibration_time > 1.0 and len(raw_data_points) > 20:
                res = calibrate_magnetometer(data_np)
                if res is not None:
                    B, S = res
                    calibration_result = (B, S)
                last_calibration_time = time.time()
            
            # Always update corrected points display if we have B and S (even if algo is stopped)
            if len(raw_data_points) > 0:
                B, S = calibration_result
                
                # Update text
                B_str = f"[{B[0]:.2f}, {B[1]:.2f}, {B[2]:.2f}]"
                S_str = "\n".join([f"[{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}]" for r in S])
                
                raw_centered = (data_np - B).T
                corrected = np.dot(S, raw_centered).T
                
                avg_radius = np.mean(np.linalg.norm(raw_centered, axis=0))
                corrected_scaled = corrected * avg_radius
                
                scat_corr._offsets3d = (corrected_scaled[:, 0], corrected_scaled[:, 1], corrected_scaled[:, 2])
                
                if len(corrected) > 0:
                    last_pt = corrected[-1]
                    info_text.set_text(f"Points: {len(raw_data_points)}\n\n"
                                     f"Hard Iron Bias B:\n{B_str}\n\n"
                                     f"Soft Iron Matrix S:\n{S_str}\n\n"
                                     f"Latest Calibrated (Unit):\n[{last_pt[0]:.3f}, {last_pt[1]:.3f}, {last_pt[2]:.3f}]")

        plt.pause(0.05)
        if not plt.fignum_exists(fig.number):
            break

if __name__ == "__main__":
    main()
