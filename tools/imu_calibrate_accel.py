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

"""
IMU Accelerometer Calibration Tool

Interactive tool to compute the Bias (B) and Scale (S) matrix 
for accelerometer calibration using ellipsoid fitting.

Instructions:
1. Run this script.
2. Follow on-screen instructions to capture data in 6 orientations.
3. Compute Calibration.
4. Update the firmware with the result.
"""

# --- Configuration ---
SERIAL_PORT = None
BAUD_RATE = 9600
MONITOR_DATA_ID = 0x00  # From logger.c
PLOT_LIMIT = 20000 # Assuming 16-bit signed int raw values, 1g ~ 16384

# Auto-detect serial port
ports = serial.tools.list_ports.comports()
found_port = False
print("Scanning for ports...")
for port, desc, hwid in sorted(ports):
    if any(x in port for x in ['usbmodem', 'usbserial', 'SLAB_USBtoUART', 'ttyACM', 'ttyUSB']):
        SERIAL_PORT = port
        found_port = True
        print(f"Auto-selected Port: {port} ({desc})")
        break

if not found_port:
    print('----------------------------------------------------')
    print('ERROR: No compatible serial port found.')
    print('Please connect the Flight Controller and try again.')
    print('----------------------------------------------------')


# --- Global State ---
data_queue = queue.Queue()
raw_data_points = [] # List of averaged points (one per position)
current_position_samples = [] # Temporary samples for the current position
is_capturing = False
SAMPLES_PER_POSITION = 100
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
                    # Filter out huge spikes if any, but allow normal range
                    if abs(x) <= 40000 and abs(y) <= 40000 and abs(z) <= 40000:
                        data_queue.put((x, y, z))
                    
    except Exception as e:
        print(f"Serial error: {e}")

# --- Calibration Algorithm ---
def calibrate_accelerometer(data):
    """
    Performs ellipsoid fitting for accelerometer calibration.
    Target is a sphere of radius R (gravity).
    Returns:
        B: Bias (3,)
        S: Scale/Rotation Matrix (3,3)
    """
    if len(data) < 10:
        return None

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
    
    # Calculate symmetric S = V * D_sqrt * V.T
    S = np.dot(vecs, np.dot(D_sqrt, vecs.T))
    
    # Note: This S maps the ellipsoid to a unit sphere (radius 1).
    # But accelerometer data has units (e.g. 16384 for 1g).
    # We want the corrected data to have magnitude ~16384 (or whatever the average radius is).
    # However, usually calibration returns normalized vectors or we need to know the target gravity.
    # For visualization, we can scale it back.
    # For the matrix S, if we want it to map to 1g units, we might need to adjust.
    # But typically, S * (raw - B) = normalized_gravity_vector (if we want unit vectors).
    # Or S * (raw - B) = gravity_vector_in_raw_units.
    # The ellipsoid fit assumes = 1 on the RHS.
    # So S * (raw - B) will have magnitude 1.
    # If we want to preserve the magnitude, we should divide S by the expected 1/R.
    # But we don't know R exactly yet (it's the average radius).
    # Let's stick to the unit sphere mapping for S, and we can scale the output for display.
    
    return center, S

# --- GUI ---
def main():
    global is_capturing, raw_data_points, current_position_samples, calibration_result, last_calibration_time
    
    # State flags
    show_raw = True
    show_calib = True
    
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    
    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect((1, 1, 1))  # Ensure 1:1:1 aspect ratio
    plt.subplots_adjust(bottom=0.25, right=0.75) # Increased bottom margin for buttons
    
    scat_raw = ax.scatter([], [], [], c='r', marker='o', s=20, label='Captured Positions')
    scat_corr = ax.scatter([], [], [], c='g', marker='o', s=20, label='Corrected Positions')
    
    # Text for results
    text_ax = plt.axes([0.76, 0.2, 0.23, 0.6])
    text_ax.axis('off')
    instructions = (
        "INSTRUCTIONS:\n\n"
        "1. Connect drone via USB.\n"
        "2. Place drone STATIC in\n"
        "   an orientation (e.g. Flat).\n"
        "3. Click 'Capture Position'.\n"
        "   Wait for completion.\n"
        "4. Rotate to a NEW side\n"
        "   (Left, Right, Front, Back,\n"
        "   Upside Down).\n"
        "5. Repeat steps 2-4 for\n"
        "   at least 6 positions.\n"
        "6. Click 'Compute Calib'.\n"
        "7. Copy B and S to imu.c."
    )
    info_text = text_ax.text(0, 1, instructions, fontsize=9, va='top', fontfamily='monospace')
    
    # --- Button Layout ---
    # Row 1: Views (Top, Bottom, Front, Back, Left, Right)
    # Row 2: Toggles (Raw, Calib)
    # Row 3: Actions (Capture, Calibrate, Reset, Clear)
    
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
    
    # Row 3: Actions
    row3_y = 0.01
    
    # Capture Position
    btn_capture_ax = plt.axes([start_x, row3_y, width*2, 0.05])
    btn_capture = Button(btn_capture_ax, 'Capture Position')
    def capture_position(event):
        global is_capturing, current_position_samples
        if not is_capturing:
            is_capturing = True
            current_position_samples = []
            btn_capture.label.set_text('Capturing...')
            print("Capturing position...")
    btn_capture.on_clicked(capture_position)

    # Compute Calibration
    btn_calc_ax = plt.axes([start_x + width*2 + gap, row3_y, width*2, 0.05])
    btn_calc = Button(btn_calc_ax, 'Compute Calib')
    def compute_calibration(event):
        global calibration_result
        if len(raw_data_points) < 6:
            print("Need at least 6 positions for calibration.")
            return
            
        data_np = np.array(raw_data_points)
        res = calibrate_accelerometer(data_np)
        if res is not None:
            B, S = res
            calibration_result = (B, S)
            print("Calibration computed successfully.")
            
            # Update text
            B_str = f"[{B[0]:.2f}, {B[1]:.2f}, {B[2]:.2f}]"
            S_str = "\n".join([f"[{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}]" for r in S])
            info_text.set_text(f"Positions: {len(raw_data_points)}\n\nBias B:\n{B_str}\n\nScale/Rot Matrix S:\n{S_str}")
            
            # Update corrected plot
            raw_centered = (data_np - B).T
            corrected = np.dot(S, raw_centered).T
            avg_radius = np.mean(np.linalg.norm(raw_centered, axis=0))
            corrected_scaled = corrected * avg_radius
            scat_corr._offsets3d = (corrected_scaled[:, 0], corrected_scaled[:, 1], corrected_scaled[:, 2])
            plt.draw()
        else:
            print("Calibration failed (not enough spread?).")
    btn_calc.on_clicked(compute_calibration)
    
    # Reset B/S
    btn_reset_ax = plt.axes([start_x + (width*2 + gap)*2, row3_y, width*2, 0.05])
    btn_reset = Button(btn_reset_ax, 'Reset B & S')
    def reset_calibration(event):
        global calibration_result
        calibration_result = (np.zeros(3), np.eye(3))
        B, S = calibration_result
        
        B_str = f"[{B[0]:.2f}, {B[1]:.2f}, {B[2]:.2f}]"
        S_str = "\n".join([f"[{r[0]:.2f}, {r[1]:.2f}, {r[2]:.2f}]" for r in S])
        info_text.set_text(f"Positions: {len(raw_data_points)}\n\nBias B:\n{B_str}\n\nScale/Rot Matrix S:\n{S_str}")
        
        scat_corr._offsets3d = ([], [], [])
        plt.draw()
        print("Calibration parameters reset.")
    btn_reset.on_clicked(reset_calibration)
    
    # Clear Data
    btn_clear_ax = plt.axes([start_x + (width*2 + gap)*3, row3_y, width*2, 0.05])
    btn_clear = Button(btn_clear_ax, 'Clear Points')
    def clear_points(event):
        global raw_data_points
        raw_data_points = []
        scat_raw._offsets3d = ([], [], [])
        scat_corr._offsets3d = ([], [], [])
        info_text.set_text("Points cleared.\nReady to capture.")
        print("Points cleared.")
        plt.draw()
    btn_clear.on_clicked(clear_points)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    while True:
        while not data_queue.empty():
            pt = data_queue.get()
            
            if is_capturing:
                current_position_samples.append(pt)
                if len(current_position_samples) >= SAMPLES_PER_POSITION:
                    # Finished capturing this position
                    avg_pt = np.mean(current_position_samples, axis=0)
                    raw_data_points.append(avg_pt)
                    
                    print(f"Position {len(raw_data_points)} captured: {avg_pt}")
                    
                    # Reset capture state
                    is_capturing = False
                    current_position_samples = []
                    btn_capture.label.set_text('Capture Position')
                    
                    # Update Raw Plot
                    data_np = np.array(raw_data_points)
                    scat_raw._offsets3d = (data_np[:, 0], data_np[:, 1], data_np[:, 2])
                    
                    # Auto-scale
                    max_range = np.max(np.abs(data_np)) if len(data_np) > 0 else PLOT_LIMIT
                    limit = max(max_range * 1.2, PLOT_LIMIT)
                    ax.set_xlim(-limit, limit)
                    ax.set_ylim(-limit, limit)
                    ax.set_zlim(-limit, limit)
                    
                    info_text.set_text(f"Positions: {len(raw_data_points)}\n\nReady for next position.")
                    plt.draw()

        plt.pause(0.05)
        if not plt.fignum_exists(fig.number):
            break

if __name__ == "__main__":
    main()
