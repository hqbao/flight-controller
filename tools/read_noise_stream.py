import serial
import struct
import time
import argparse
import csv
from datetime import datetime
import os

def parse_header(header):
    if header[0] == ord('d') and header[1] == ord('b'):
        return True
    return False

def main():
    parser = argparse.ArgumentParser(description='Read Noise Measurement Stream')
    parser.add_argument('--port', type=str, default='/dev/cu.usbserial-3110', help='Serial port')
    parser.add_argument('--baud', type=int, default=9600, help='Baud rate')
    parser.add_argument('--output', type=str, help='Output CSV file (optional, defaults to timestamped file)')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"Connected to {args.port} at {args.baud}")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    # Setup CSV output
    if args.output:
        filename = args.output
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"noise_log_{timestamp}.csv"
        # ensure full path is printed so user knows where it is
        filename = os.path.abspath(filename)

    print(f"Logging data to: {filename}")
    
    csv_file = open(filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    # Write Header
    csv_writer.writerow([
        'Timestamp', 
        'Gx_Mean', 'Gy_Mean', 'Gz_Mean', 
        'Gx_Std', 'Gy_Std', 'Gz_Std', 
        'Ax_Mean', 'Ay_Mean', 'Az_Mean', 
        'Ax_Std', 'Ay_Std', 'Az_Std'
    ])

    buffer = bytearray()
    
    # Print Header
    header_fmt = "{:>10}" * 12
    print(header_fmt.format(
        "Gx Avg", "Gy Avg", "Gz Avg", 
        "Gx Std", "Gy Std", "Gz Std",
        "Ax Avg", "Ay Avg", "Az Avg",
        "Ax Std", "Ay Std", "Az Std"
    ))
    print("-" * 125)

    try:
        while True:
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting)
                buffer.extend(chunk)

                while len(buffer) >= 6: # Header (2) + ID(1) + Class(1) + Len(2)
                    # Look for header 'db'
                    if buffer[0] != ord('d') or buffer[1] != ord('b'):
                        buffer.pop(0)
                        continue
                    
                    # Parse length
                    payload_len = buffer[4] | (buffer[5] << 8)
                    total_len = 6 + payload_len + 2 # Header + Payload + Checksum
                    
                    if len(buffer) < total_len:
                        break # Wait for more data
                    
                    # Extract message
                    msg = buffer[:total_len]
                    buffer = buffer[total_len:]
                    
                    # Verify checksum (Simple sum of bytes from ID to End of Payload)
                    # Checksum is at the end
                    calc_checksum = sum(msg[2:-2]) & 0xFFFF
                    recv_checksum = msg[-2] | (msg[-1] << 8)
                    
                    if calc_checksum == recv_checksum:
                        payload = msg[6:-2]
                        
                        # Support legacy 6-float message (Gyro only) or new 12-float message (Gyro+Accel)
                        if len(payload) == 24: # 6 floats - Legacy Gyro Only
                            try:
                                values = struct.unpack('6f', payload)
                                mean_gx, mean_gy, mean_gz, std_gx, std_gy, std_gz = values
                                
                                # Use default 0 for Accel
                                mean_ax = mean_ay = mean_az = 0.0
                                std_ax = std_ay = std_az = 0.0
                                
                                # Units are already scaled in FC
                                print(f"{mean_gx:10.1f} {mean_gy:10.1f} {mean_gz:10.1f} "
                                      f"{std_gx:10.2f} {std_gy:10.2f} {std_gz:10.2f} "
                                      f"{mean_ax:10.2f} {mean_ay:10.2f} {mean_az:10.2f} "
                                      f"{std_ax:10.3f} {std_ay:10.3f} {std_az:10.3f}")
                                
                                csv_writer.writerow([time.time(), 
                                    mean_gx, mean_gy, mean_gz, std_gx, std_gy, std_gz, 
                                    mean_ax, mean_ay, mean_az, std_ax, std_ay, std_az
                                ])
                                csv_file.flush()
                                
                            except Exception as e:
                                print(f"Parse Error: {e}")

                        elif len(payload) == 48: # 12 floats - Gyro + Accel
                            try:
                                values = struct.unpack('12f', payload)
                                mean_gx, mean_gy, mean_gz, std_gx, std_gy, std_gz, \
                                mean_ax, mean_ay, mean_az, std_ax, std_ay, std_az = values
                                
                                print(f"{mean_gx:10.1f} {mean_gy:10.1f} {mean_gz:10.1f} "
                                      f"{std_gx:10.2f} {std_gy:10.2f} {std_gz:10.2f} "
                                      f"{mean_ax:10.2f} {mean_ay:10.2f} {mean_az:10.2f} "
                                      f"{std_ax:10.3f} {std_ay:10.3f} {std_az:10.3f}")
                                
                                csv_writer.writerow([time.time(), 
                                    mean_gx, mean_gy, mean_gz, std_gx, std_gy, std_gz, 
                                    mean_ax, mean_ay, mean_az, std_ax, std_ay, std_az
                                ])
                                csv_file.flush()
                                
                            except Exception as e:
                                print(f"Parse Error: {e}")
            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping...")
        if csv_file:
            csv_file.close()

if __name__ == '__main__':
    main()
