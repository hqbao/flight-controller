import sys
import struct
import socket
import math
import time

# Gazebo Transport
try:
    from gz.transport13 import Node
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    from gz.msgs10.laserscan_pb2 import LaserScan
    from gz.msgs10.navsat_pb2 import NavSat
    from gz.msgs10.twist_pb2 import Twist
    from gz.msgs10.wrench_pb2 import Wrench
    from gz.msgs10.entity_wrench_pb2 import EntityWrench
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.pose_v_pb2 import Pose_V
except ImportError as e:
    print(f"Error: Could not import gz-transport: {e}")
    print("Try: pip install gz-transport13 or similar.")
    sys.exit(1)

# Config
GZ_WORLD = "default"
MODEL_NAME = "quadcopter"
FC_IP = "127.0.0.1"
FC_PORT_SENSORS = 45454
FC_PORT_MOTORS = 45455

# Sockets
sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv.bind((FC_IP, FC_PORT_MOTORS))
sock_recv.setblocking(False)

sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# State
last_print_time = 0.0
m = [0.0, 0.0, 0.0, 0.0]  # Store last motor command
avg_thr = 0.0             # Store last avg thr
# Quaternion (w, x, y, z) - Default Identity (Upright)
rotation_q = [1.0, 0.0, 0.0, 0.0]
# Debug force values
last_fx, last_fy, last_fz = 0.0, 0.0, 0.0

sensor_data = {
    "timestamp": 0.0,
    "accel": [0.0]*3,
    "gyro": [0.0]*3,
    "mag": [0.0]*3,
    "pressure": 101325.0,
    "range": 0.0,
    "gps": [0.0]*3,
    "flow": [0.0]*2,
    "gps_vel": [0.0]*3
}

# --- State for Velocity Calculation ---
last_pose_pos = None
last_pose_time = 0.0

# --- Callbacks ---

imus_received = 0
def on_imu(msg):
    global imus_received
    imus_received += 1
    sensor_data["timestamp"] = time.time()
    # ... process accelerator ...
    sensor_data["accel"] = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
    sensor_data["gyro"] = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
    send_packet()

def on_mag(msg):
    # Field: field_tesla
    # print(f"MAG_TEST: {msg.field_tesla.x}, {msg.field_tesla.y}, {msg.field_tesla.z}")
    sensor_data["mag"] = [msg.field_tesla.x, msg.field_tesla.y, msg.field_tesla.z]

def on_pressure(msg):
    sensor_data["pressure"] = msg.pressure

def on_lidar(msg):
    # Retrieve center range
    if len(msg.ranges) > 0:
        center_idx = len(msg.ranges) // 2
        sensor_data["range"] = msg.ranges[center_idx]

def on_gps(msg):
    # Fields: latitude_deg, longitude_deg, altitude
    sensor_data["gps"] = [msg.latitude_deg, msg.longitude_deg, msg.altitude]

def on_pose(msg):
    global rotation_q
    global last_pose_pos, last_pose_time
    
    # msg has orientation (Quaternion)
    # Gazebo Pose msg orientation: x, y, z, w
    rotation_q = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
    
    # Update Altitude from Ground Truth (Pose) - Used as Barometer
    sensor_data["gps"][2] = msg.position.z

    # --- Velocity Calculation (Ground Truth) ---
    curr_time = time.time()
    curr_pos = [msg.position.x, msg.position.y, msg.position.z]

    if last_pose_pos is not None:
        dt = curr_time - last_pose_time
        if dt > 0.001: # Avoid div by zero
            # Calculate Velocity in Gazebo Frame (ENU)
            vx = (curr_pos[0] - last_pose_pos[0]) / dt
            vy = (curr_pos[1] - last_pose_pos[1]) / dt
            vz = (curr_pos[2] - last_pose_pos[2]) / dt
            
            # Map to NED (North, East, Down) for Flight Controller
            # Assumption: Gazebo World is ENU (X=East, Y=North, Z=Up) or Standard (X=Forward, Y=Left, Z=Up)
            # ROS/Gazebo Standard: X=Forward(Red), Y=Left(Green), Z=Up(Blue)
            # NED Standard: X=North, Y=East, Z=Down
            
            # Simple mapping (Identity for now, fix if needed based on flight test)
            # If Model starts facing East...
            # Let's assume standard ROS ENU -> NED conversion:
            # North (X) = Y (ROS)
            # East (Y) = X (ROS)
            # Down (Z) = -Z (ROS)
            
            # HOWEVER, in sitl_combined, we might just use X=North.
            # Let's stick to direct mapping X->N, Y->E, Z->-D for now.
            vel_n = vx
            vel_e = vy
            vel_d = -vz
            
            sensor_data["gps_vel"] = [vel_n, vel_e, vel_d]

    last_pose_pos = curr_pos
    last_pose_time = curr_time

def on_pose_v(msg):
    global rotation_q
    
    # Pose_V is a list of poses.
    # If this is /world/.../pose/info, it usually has all models.
    # If /model/.../pose, maybe just one.
    
    target_pose = None
    if len(msg.pose) == 1:
        target_pose = msg.pose[0]
    else:
        for p in msg.pose:
            if p.name == "quadcopter" or p.name == "base_link":
                 target_pose = p
                 break
        # Fallback to first if not found (and likely non-empty)
        if not target_pose and len(msg.pose) > 0:
            target_pose = msg.pose[0]

    if target_pose:
        rotation_q = [target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z]
        # Store Ground Truth Z for packet injection
        pose_z = target_pose.position.z

# --- Transport ---

def send_packet():
    # Overwrite GPS Alt with Ground Truth Z if available (for precise Baro simulation)
    # This prevents NavSat noise/lack of lock from messing up the fake baro
    if 'pose_z' in globals():
        sensor_data["gps"][2] = pose_z

    # Struct: 20 doubles (160 bytes)
    # Added gps_vel (3 doubles) to end
    pkt = struct.pack('<20d',
        sensor_data["timestamp"],
        *sensor_data["accel"],
        *sensor_data["gyro"],
        *sensor_data["mag"],
        sensor_data["pressure"],
        sensor_data["range"],
        *sensor_data["gps"],
        *sensor_data["flow"],
        *sensor_data["gps_vel"] 
    )
    sock_send.sendto(pkt, (FC_IP, FC_PORT_SENSORS))

# Main setup
node = Node()

# Dynamic Topic Discovery
print("Scanning for Gazebo topics...")
all_topics = node.topic_list()
print(f"DEBUG: Found {len(all_topics)} topics.")
for t in all_topics:
    # Print potential pose topics to help debug
    if "pose" in t.lower() or "quadcopter" in t.lower():
        print(f"  - {t}")

imu_topic = None

# Look for our IMU in the list
for t in all_topics:
    if "imu_sensor/imu" in t and "quadcopter" in t:
        imu_topic = t
        break

if imu_topic:
    print(f"Found IMU: {imu_topic}")
    # Derive base path from IMU topic
    # Format: /world/default/model/quadcopter/link/base_link/sensor/imu_sensor/imu
    # We want: /world/default/model/quadcopter/link/base_link/sensor
    base_parts = imu_topic.split("/")
    # Remove last 2 parts ("imu_sensor", "imu")
    base_parts = base_parts[:-2]
    base_topic = "/".join(base_parts)
    
    # Try to deduce World Name from IMU topic if possible
    # /world/<WORLD_NAME>/model/...
    if len(base_parts) > 2 and base_parts[1] == "world":
        GZ_WORLD = base_parts[2]
        print(f"Detected World Name: {GZ_WORLD}")
    
    print(f"Base Sensor Path: {base_topic}")
    
    node.subscribe(IMU, imu_topic, on_imu)
    # Validated Topic Names from Logs
    mag_topic = f"{base_topic}/magnetometer_sensor/magnetometer"
    navsat_topic = f"{base_topic}/navsat_sensor/navsat"

    print(f"Subscribing to Mag: {mag_topic}", flush=True)
    if node.subscribe(Magnetometer, mag_topic, on_mag):
        print("✅ Mag Subscribed", flush=True)
    else:
        print("❌ Mag Subscribe Failed", flush=True)

    node.subscribe(FluidPressure, f"{base_topic}/air_pressure_sensor/air_pressure", on_pressure)
    node.subscribe(LaserScan, f"{base_topic}/gpu_lidar/scan", on_lidar)
    
    print(f"Subscribing to NavSat: {navsat_topic}")
    node.subscribe(NavSat, navsat_topic, on_gps)
    
    # Subscribe to Pose (Try multiple variations because Gz topic names are inconsistent)
    # 1. Standard: /model/{name}/pose
    # 2. World-Prefixed: /world/{world}/model/{name}/pose
    # 3. Dynamic Model Pose (Garden/Harmonic): /model/{name}/pose (often published by default)
    
    pose_topics = [
        f"/model/{MODEL_NAME}/pose",
        f"/world/{GZ_WORLD}/model/{MODEL_NAME}/pose",
        f"/world/{GZ_WORLD}/pose/info" # Sometimes here
    ]
    
    print(f"Attempting to subscribe to Pose topics: {pose_topics}")
    for t in pose_topics:
        # Try both types
        node.subscribe(Pose, t, on_pose)
        node.subscribe(Pose_V, t, on_pose_v)
    
    pub_cmd_vel = node.advertise(f"/model/{MODEL_NAME}/wrench", EntityWrench)

else:
    print("⚠️ WARNING: Could not find IMU in topic list. Attempting fallback hardcoded path...")
    
    # Fallback for Gazebo Sim (Garden/Harmonic) standard paths
    # This path was observed in server logs
    base_topic = "/world/default/model/quadcopter/link/base_link/sensor"
    imu_topic = f"{base_topic}/imu_sensor/imu"
    
    print(f"Trying to subscribe to: {imu_topic}")
    
    if node.subscribe(IMU, imu_topic, on_imu):
        print("✅ BLIND SUBSCRIPTION SUCCESSFUL (or at least no error)")
        # Subscribe others blindly
        mag_topic = f"{base_topic}/magnetometer_sensor/magnetometer"
        print(f"Attempting Mag Sub: {mag_topic}")
        if node.subscribe(Magnetometer, mag_topic, on_mag):
             print(f"✅ Mag Subscribed")
        else:
             print(f"❌ Mag Subscribe Failed")

        node.subscribe(FluidPressure, f"{base_topic}/air_pressure_sensor/air_pressure", on_pressure)
        node.subscribe(LaserScan, f"{base_topic}/gpu_lidar/scan", on_lidar)
        node.subscribe(NavSat, f"{base_topic}/navsat/navsat", on_gps)
        
        # Subscribe to Pose (Try both short and full paths)
        pose_topic_short = f"/model/{MODEL_NAME}/pose"
        pose_topic_full = f"/world/{GZ_WORLD}/model/{MODEL_NAME}/pose"
        print(f"Subscribing to Pose: {pose_topic_short} AND {pose_topic_full}")
        node.subscribe(Pose, pose_topic_short, on_pose)
        node.subscribe(Pose, pose_topic_full, on_pose)
        
        # Publish Wrench (Force/Torque) instead of Twist/Velocity
        # Topic must match ApplyLinkWrench plugin in SDF
        pub_cmd_vel = node.advertise(f"/model/{MODEL_NAME}/wrench", EntityWrench)
    else:
        print("❌ ERROR: Blind subscription failed.")
        sys.exit(1)

print(f"Bridge running. Receiving Motors on {FC_PORT_MOTORS}, Sending Sensors to {FC_PORT_SENSORS}")

# --- Loop ---
while True:
    try:
        data, _ = sock_recv.recvfrom(1024)
        if len(data) == 16: # 4 floats
            m = struct.unpack('<4f', data) # fl, fr, bl, br
            
            # Kinematic Map -> EntityWrench
            msg = EntityWrench()
            msg.entity.name = "quadcopter::base_link"
            msg.entity.type = Entity.LINK
            
            # Throttle Avg -> Linear Z
            avg_thr = sum(m) / 4.0

            # Throttle Avg -> Linear Z Force
            # Assume mass ~1.5kg.
            # 1.5kg * 9.8m/s^2 = 14.7N to hover.
            # Let's give it generous thrust: 4g max = ~60N.
            
            # --- Thrust Rotation Logic ---
            # Thrust Vector in BODY Frame (Upward relative to drone)
            # F_body = [0, 0, Thrust]
            thrust_scalar = avg_thr * 60.0
            
            # Rotate by Quaternion q (w, x, y, z)
            # Standard formula for rotating vector v by q: v' = q * v * q_inv
            # Simplified for v=[0,0,Fz]:
            w, x, y, z = rotation_q
            
            # F_world_x = 2(xz + wy) * Fz
            # F_world_y = 2(yz - wx) * Fz
            # F_world_z = (1 - 2(x^2 + y^2)) * Fz
            
            fx = 2.0 * (x * z + w * y) * thrust_scalar
            fy = 2.0 * (y * z - w * x) * thrust_scalar
            fz = (1.0 - 2.0 * (x * x + y * y)) * thrust_scalar
            
            last_fx, last_fy, last_fz = fx, fy, fz

            # Debug Print (Throttle > 10% only)
            if avg_thr > 0.1:
                print(f"DEBUG: Thr={avg_thr:.2f} ForceZ={fz:.2f} Motors={m}")

            msg.wrench.force.x = fx
            msg.wrench.force.y = fy
            msg.wrench.force.z = fz
            
            # Pitch: (FL+FR) - (BL+BR)
            pitch_cmd = (m[0] + m[1]) - (m[2] + m[3])
            # Pitch Torque needs to generally align with body Y
            # However, Gazebo World frame torques are applied globally.
            # Usually for small angles, World Torque ~ Body Torque.
            # For 90 degree pitch, World Torque Z becomes Body Torque Y.
            # CORRECT WAY: Rotate Torque Vector too.
            # Local Torque Vector: [Tx, Ty, Tz]
            
            cmd_roll = (m[2] + m[0]) - (m[3] + m[1]) # Left - Right
            cmd_pitch = (m[0] + m[1]) - (m[2] + m[3]) # Front - Back
            cmd_yaw = (m[0] + m[3]) - (m[1] + m[2])   # CW - CCW
            
            # Scale Factors
            t_x = cmd_roll * 1.0  # Body Roll Torque
            t_y = -cmd_pitch * 1.0 # Body Pitch Torque (Nose Down is negative Y in some frames, let's keep sign consistent)
            t_z = -cmd_yaw * 0.5   # Body Yaw Torque
            
            # Rotate Torque Vector [t_x, t_y, t_z] by Quaternion to World Frame
            # v' = v + 2 * cross(r, cross(r, v) + w * v) 
            # where r = [x, y, z] is vector part of quat
            
            # Or standard matrix expansion:
            # tx_w = (1 - 2y^2 - 2z^2)tx + (2xy - 2wz)ty     + (2xz + 2wy)tz
            # ty_w = (2xy + 2wz)tx     + (1 - 2x^2 - 2z^2)ty + (2yz - 2wx)tz
            # tz_w = (2xz - 2wy)tx     + (2yz + 2wx)ty       + (1 - 2x^2 - 2y^2)tz
            
            r00 = 1 - 2*y*y - 2*z*z
            r01 = 2*x*y - 2*w*z
            r02 = 2*x*z + 2*w*y
            
            r10 = 2*x*y + 2*w*z
            r11 = 1 - 2*x*x - 2*z*z
            r12 = 2*y*z - 2*w*x
            
            r20 = 2*x*z - 2*w*y
            r21 = 2*y*z + 2*w*x
            r22 = 1 - 2*x*x - 2*y*y
            
            msg.wrench.torque.x = r00*t_x + r01*t_y + r02*t_z
            msg.wrench.torque.y = r10*t_x + r11*t_y + r12*t_z
            msg.wrench.torque.z = r20*t_x + r21*t_y + r22*t_z

            pub_cmd_vel.publish(msg)

            # VISUALS: Spin the Rotors
            rot_directions = [1.0, -1.0, -1.0, 1.0]
            for i in range(4):
                if m[i] > 0.05:
                    r_msg = EntityWrench()
                    r_msg.entity.name = f"{MODEL_NAME}::rotor_{i}"
                    r_msg.entity.type = Entity.LINK
                    # Apply torque to spin
                    r_msg.wrench.torque.z = 0.02 * m[i] * rot_directions[i]
                    pub_cmd_vel.publish(r_msg)
            
    except BlockingIOError:
        pass
    except Exception as e:
        print(f"Error: {e}")
    
    # Unified Debug Print (10Hz)
    current_time = time.time()
    if current_time - last_print_time > 0.1:
        last_print_time = current_time
        # Use simple VT100 clear line code \033[K to ensure clean overwrite
        print(f"\rQ: [{rotation_q[0]:.2f}, {rotation_q[1]:.2f}, {rotation_q[2]:.2f}, {rotation_q[3]:.2f}] | ThrF: [{last_fx:.1f}, {last_fy:.1f}, {last_fz:.1f}] \033[K", end="")

    time.sleep(0.001)
