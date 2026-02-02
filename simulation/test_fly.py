import socket
import struct
import time

# Send to Bridge
UDP_IP = "127.0.0.1"
UDP_PORT = 45455

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Sending 50% throttle to {UDP_IP}:{UDP_PORT}...")
try:
    while True:
        # float[4] motors
        motors = [10.0, 10.0, 10.0, 10.0]
        data = struct.pack('4f', *motors)
        sock.sendto(data, (UDP_IP, UDP_PORT))
        time.sleep(0.01) # 100Hz
except KeyboardInterrupt:
    print("Stopping")
