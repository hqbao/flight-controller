import serial
import matplotlib.pyplot as plt
import struct
import queue
from threading import Thread
import serial.tools.list_ports
from serial import Serial
import math
from matplotlib.ticker import LinearLocator, NullLocator

# --- Configuration ---
g_serial_port = None
g_baud_rate = 9600

# Auto-detect serial port
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
    if port.startswith('/dev/cu.usbmodem') or port.startswith('/dev/cu.usbserial') or port.startswith('/dev/cu.SLAB_USBtoUART'):
        g_serial_port = port
        break

if g_serial_port is None:
    print('No serial port found. Please update g_serial_port in the script.')
    exit()
# --- End Configuration ---

# Global variables
ser = serial.Serial(g_serial_port, g_baud_rate)
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Camera angles for different views
# Using slightly offset angles to avoid 90/180 degree singularities
camera_angles = {
    'top': (89.9, 0),
    'bottom': (-89.9, 0),
    'front': (0, 0),
    'back': (0, 179.9),
    'left': (0, 89.9),
    'right': (0, -89.9)
}
current_view = 'top'
view_changed = False

# 3D lines (instead of quiver for stability)
line1 = None
line2 = None
# Scatter points for vector heads
head1 = None
head2 = None

# Data queue for inter-thread communication
queue1 = queue.Queue()

# Vector data
g_vec1_x = 0.0
g_vec1_y = 0.0
g_vec1_z = 0.0
g_vec2_x = 0.0
g_vec2_y = 0.0
g_vec2_z = 0.0

def run_db_reader(in_queue):
    """Read UART data and extract frames"""
    with Serial(g_serial_port, g_baud_rate, timeout=3) as stream:
        while True:
            try:
                byte = stream.read(1)
                if len(byte) != 1:
                    continue
                    
                if byte[0] == 0x62 or byte[0] == 0x64:  # 'b' or 'd'
                    byte2 = stream.read(1)
                    if len(byte2) == 0:
                        continue
                    
                    # Verify we have the full header 'db' or 'bd'
                    if not ((byte[0] == 0x64 and byte2[0] == 0x62) or (byte[0] == 0x62 and byte2[0] == 0x64)):
                        continue
                    
                    # Read frame
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
                    
                    if size > 1024 or size == 0:
                        continue
                        
                    payload = stream.read(size)
                    if len(payload) < size:
                        continue
                        
                    checksum = stream.read(2)
                    if len(checksum) < 2:
                        continue
                    
                    # Verify checksum
                    calc_sum = ID + clazz + length[0] + length[1]
                    for b in payload:
                        calc_sum += b
                    calc_sum = calc_sum & 0xFFFF
                    
                    recv_sum = int.from_bytes(checksum, 'little')
                    
                    if recv_sum != 0 and recv_sum != calc_sum:
                        continue
                    
                    in_queue.put((clazz, ID, payload))
            except Exception as e:
                continue

def run_parser():
    """Parse UART data and update global vectors"""
    global g_vec1_x, g_vec1_y, g_vec1_z, g_vec2_x, g_vec2_y, g_vec2_z
    while True:
        try:
            clazz, ID, payload = queue1.get(timeout=0.01)

            if clazz == 0x00:
                if len(payload) == 12:
                    # Single vector: 3x float32
                    x_f, y_f, z_f = struct.unpack('<fff', payload)
                    if abs(x_f) <= 1.0 and abs(y_f) <= 1.0 and abs(z_f) <= 1.0:
                        g_vec1_x = x_f
                        g_vec1_y = y_f
                        g_vec1_z = z_f
                    
                elif len(payload) == 24:
                    # Two vectors: 6x float32
                    x1_f, y1_f, z1_f, x2_f, y2_f, z2_f = struct.unpack('<ffffff', payload)
                    if (abs(x1_f) <= 1.0 and abs(y1_f) <= 1.0 and abs(z1_f) <= 1.0 and
                        abs(x2_f) <= 1.0 and abs(y2_f) <= 1.0 and abs(z2_f) <= 1.0):
                        g_vec1_x = x1_f
                        g_vec1_y = y1_f
                        g_vec1_z = z1_f
                        g_vec2_x = x2_f
                        g_vec2_y = y2_f
                        g_vec2_z = z2_f
                        # print(f"V1=({x1_f:.3f}, {y1_f:.3f}, {z1_f:.3f}), V2=({x2_f:.3f}, {y2_f:.3f}, {z2_f:.3f})")
            
        except queue.Empty:
            pass
        except Exception:
            pass

def on_key_press(event):
    """Handle keyboard input for view changes"""
    global current_view, view_changed
    key_map = {'t': 'top', 'b': 'bottom', 'f': 'front', 'k': 'back', 'l': 'left', 'r': 'right'}
    
    if event.key in key_map:
        current_view = key_map[event.key]
        view_changed = True
        print(f"View: {current_view}")

def update():
    """Update the 3D plot"""
    global line1, line2, head1, head2, current_view, view_changed
    
    # Update Line 1 (Red)
    line1.set_data([0, g_vec1_x], [0, g_vec1_y])
    line1.set_3d_properties([0, g_vec1_z])
    head1.set_data([g_vec1_x], [g_vec1_y])
    head1.set_3d_properties([g_vec1_z])

    # Update Line 2 (Blue)
    line2.set_data([0, g_vec2_x], [0, g_vec2_y])
    line2.set_3d_properties([0, g_vec2_z])
    head2.set_data([g_vec2_x], [g_vec2_y])
    head2.set_3d_properties([g_vec2_z])

    # Apply view if changed
    if view_changed:
        try:
            # 1. Disable autoscale and force linear scale BEFORE anything else
            ax.set_autoscale_on(False)
            ax.set_xscale('linear')
            ax.set_yscale('linear')
            ax.set_zscale('linear')

            # 2. Set View
            elev, azim = camera_angles[current_view]
            ax.view_init(elev=elev, azim=azim)
            
            # 3. Re-enforce limits
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            
            # 4. Force Locators
            ax.xaxis.set_major_locator(LinearLocator(5))
            ax.yaxis.set_major_locator(LinearLocator(5))
            ax.zaxis.set_major_locator(LinearLocator(5))
            
            ax.xaxis.set_minor_locator(NullLocator())
            ax.yaxis.set_minor_locator(NullLocator())
            ax.zaxis.set_minor_locator(NullLocator())
        except Exception as e:
            print(f"View error: {e}")
        view_changed = False

    # Update title
    mag1 = math.hypot(g_vec1_x, g_vec1_y, g_vec1_z)
    mag2 = math.hypot(g_vec2_x, g_vec2_y, g_vec2_z)
    ax.set_title(f'View: {current_view.upper()} | V1(red): {mag1:.3f} | V2(blue): {mag2:.3f}\n'
                 f'[t]op [b]ottom [f]ront [k]back [l]eft [r]ight')

def main():
    """Main function"""
    global fig, ax, line1, line2, head1, head2
    
    # Start threads
    reader_thread = Thread(target=run_db_reader, args=(queue1,))
    reader_thread.daemon = True
    reader_thread.start()

    parser_thread = Thread(target=run_parser, args=())
    parser_thread.daemon = True
    parser_thread.start()

    # Setup axes
    ax.set_autoscale_on(False)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    
    # Force linear scale and disable minor ticks
    try:
        ax.set_xscale('linear')
        ax.set_yscale('linear')
        ax.set_zscale('linear')
    except:
        pass

    ax.xaxis.set_minor_locator(NullLocator())
    ax.yaxis.set_minor_locator(NullLocator())
    ax.zaxis.set_minor_locator(NullLocator())

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Vector Viewer')
    
    # Initialize lines and heads
    line1, = ax.plot([0, 0], [0, 0], [0, 0], color='red', linewidth=2, alpha=0.8)
    head1, = ax.plot([0], [0], [0], color='red', marker='o', markersize=5, alpha=0.8)
    
    line2, = ax.plot([0, 0], [0, 0], [0, 0], color='blue', linewidth=2, alpha=0.8)
    head2, = ax.plot([0], [0], [0], color='blue', marker='o', markersize=5, alpha=0.8)
    
    # Disable mouse interaction by overwriting event handlers
    # This effectively disables rotation/zoom/pan via mouse
    ax._on_move = lambda event: None
    ax._on_button_press = lambda event: None
    ax._on_button_release = lambda event: None
    
    # Disable coordinate formatting to prevent errors during updates
    ax.format_coord = lambda x, y: ""
    
    try:
        ax.set_box_aspect((1, 1, 1))
    except:
        pass
    
    print("=" * 50)
    print("3D Vector Viewer")
    print("=" * 50)
    print("Controls:")
    print("  [t] Top    [b] Bottom  [f] Front")
    print("  [k] Back   [l] Left    [r] Right")
    print("  (Mouse interaction disabled)")
    print("=" * 50)

    # Connect keyboard
    fig.canvas.mpl_connect('key_press_event', on_key_press)

    # Animation loop
    def animate():
        try:
            update()
            fig.canvas.draw_idle()
        except ValueError as e:
            # Catch the specific log-scale error and try to recover
            if "log-scaled" in str(e):
                try:
                    ax.set_xscale('linear')
                    ax.set_yscale('linear')
                    ax.set_zscale('linear')
                    ax.set_xlim(-1, 1)
                    ax.set_ylim(-1, 1)
                    ax.set_zlim(-1, 1)
                except:
                    pass
        except Exception:
            pass
    
    timer = fig.canvas.new_timer()
    timer.interval = 50
    timer.callbacks.append((animate, (), {}))
    timer.start()

    # Show
    plt.show()

if __name__ == '__main__':
    main()
