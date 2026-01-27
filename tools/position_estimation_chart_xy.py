import math
import time
import matplotlib.pyplot as plt
import queue
from threading import Thread
from serial import Serial
import numpy as np
from matplotlib import animation
import serial.tools.list_ports
import struct

# --- Style Config ---
plt.style.use('dark_background')
LINE_COLORS = ['#FF5555', '#55FF55', '#5555FF', '#FF55FF', '#55FFFF', '#FFFF00'] # Red, Green, Blue, Magenta, Cyan, Yellow
LABELS = ['Pos X', 'Pos Y', 'Pos Z', 'Vel X', 'Vel Y', 'Vel Z']

g_baud_rate = 9600
g_serial_port = None
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
  print("{}: {} [{}]".format(port, desc, hwid))
  if port.startswith('/dev/cu.usbmodem') or port.startswith('/dev/cu.usbserial') or port.startswith('/dev/cu.SLAB_USBtoUART'):
    g_serial_port = port

if g_serial_port is None:
  print('No serial port found')
  exit()

max_win_size = 300
g_line = np.linspace(start=0, stop=1, num=max_win_size)
g_val1 = np.zeros(max_win_size, float)
g_val2 = np.zeros(max_win_size, float)
g_val3 = np.zeros(max_win_size, float)
g_val4 = np.zeros(max_win_size, float)
g_val5 = np.zeros(max_win_size, float)
g_val6 = np.zeros(max_win_size, float)
g_cur_idx = 0

# Autoscaling State
g_ylim_pos_xy = [ -0.1, 0.1 ]
g_ylim_vel_xy = [ -0.1, 0.1 ]

g_clazz = 0x00
g_clazz_id = 0x00
g_payload_size = 0

def run_db_reader(in_queue):
  with Serial(g_serial_port, g_baud_rate, timeout=3) as stream:
    while True:
      byte = stream.read(1)
      if len(byte) == 1 and (byte[0] == 0x62 or byte[0] == 0x64): # 'b' or 'd'
        byte = stream.read(1)[0]
        if byte == 0x64 or byte == 0x62: # 'd' or 'b'
          clazz = stream.read(1)[0]
          ID = stream.read(1)[0]
          length = stream.read(2)
          size = int.from_bytes(length, 'little')
          payload = stream.read(size)
          checksum = stream.read(2)
          # print(clazz, ID, size)
          in_queue.put((clazz, ID, payload))

def run_parser():
  global g_val, g_cur_idx, g_clazz, g_payload_size
  while True:
    try:
      g_clazz, g_clazz_id, payload = queue1.get(timeout=0.01)
      if g_clazz == 0x00:
        g_payload_size = len(payload)

        if g_payload_size >= 4:
          z1, = struct.unpack('<f', payload[:4])
          g_val1[g_cur_idx] = z1

        if g_payload_size >= 8:
          z1, z2 = struct.unpack('<ff', payload[:8])
          g_val1[g_cur_idx] = z1
          g_val2[g_cur_idx] = z2

        if g_payload_size >= 12:
          z1, z2, z3 = struct.unpack('<fff', payload[:12])
          g_val1[g_cur_idx] = z1
          g_val2[g_cur_idx] = z2
          g_val3[g_cur_idx] = z3
        
        if g_payload_size >= 16:
          z1, z2, z3, z4 = struct.unpack('<ffff', payload[:16])
          g_val1[g_cur_idx] = z1
          g_val2[g_cur_idx] = z2
          g_val3[g_cur_idx] = z3
          g_val4[g_cur_idx] = z4

        if g_payload_size >= 20:
          z1, z2, z3, z4, z5 = struct.unpack('<fffff', payload[:20])
          g_val1[g_cur_idx] = z1
          g_val2[g_cur_idx] = z2
          g_val3[g_cur_idx] = z3
          g_val4[g_cur_idx] = z4
          g_val5[g_cur_idx] = z5

        if g_payload_size >= 24:
          z1, z2, z3, z4, z5, z6 = struct.unpack('<ffffff', payload[:24])
          g_val1[g_cur_idx] = z1
          g_val2[g_cur_idx] = z2
          g_val3[g_cur_idx] = z3
          g_val4[g_cur_idx] = z4
          g_val5[g_cur_idx] = z5
          g_val6[g_cur_idx] = z6

        g_cur_idx += 1
        if (g_cur_idx >= max_win_size): 
          g_cur_idx = 0

    except queue.Empty:
      pass

queue1 = queue.Queue()
in_thread1 = Thread(target=run_db_reader, args=(queue1,), daemon=True)
in_thread1.start()

in_thread2 = Thread(target=run_parser, args=(), daemon=True)
in_thread2.start()

# Setup 2 Subplots for XY
# Top: Pos XY
# Bot: Vel XY
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
fig.canvas.manager.set_window_title(f'Skydev Telemetry (XY) - {g_serial_port}')

def update_limits_expand_only(curr_min, curr_max, current_lims, pad_factor=0.2):
    updated = False
    # If data goes below min
    if curr_min < current_lims[0]:
        # Expand down with padding
        height = current_lims[1] - curr_min
        current_lims[0] = curr_min - (height * pad_factor)
        updated = True
        
    # If data goes above max
    if curr_max > current_lims[1]:
        # Expand up with padding
        height = curr_max - current_lims[0]
        current_lims[1] = curr_max + (height * pad_factor)
        updated = True
    return updated

def animate(i):
  global g_clazz, g_line, g_val1, g_val2, g_val3, g_val4, g_val5, g_val6, g_payload_size
  global g_ylim_pos_xy, g_ylim_vel_xy

  for ax in [ax1, ax2]:
      ax.cla()
      ax.grid(True, color='#333333', linestyle='--')
  
  # --- Configure Axes ---
  ax1.set_ylabel('Horizontal Pos (m)')
  ax1.set_title('XY Position')
  
  ax2.set_ylabel('Horizontal Vel (m/s)')
  ax2.set_xlabel('Sample Window')
  ax2.set_title('XY Velocity')

  if g_clazz == 0x00:
    # --- PLOT 1: POS XY (Channels 1,2) ---
    data_pos_xy = []
    
    if g_payload_size >= 4:
      val1 = np.concatenate((g_val1[g_cur_idx:], g_val1[:g_cur_idx]))
      ax1.plot(g_line, val1, color=LINE_COLORS[0], label=f"{LABELS[0]}: {val1[-1]:.3f}") # Pos X
      data_pos_xy.append(val1)

    if g_payload_size >= 8:
      val2 = np.concatenate((g_val2[g_cur_idx:], g_val2[:g_cur_idx]))
      ax1.plot(g_line, val2, color=LINE_COLORS[1], label=f"{LABELS[1]}: {val2[-1]:.3f}") # Pos Y
      data_pos_xy.append(val2)

    if data_pos_xy:
        all_d = np.concatenate(data_pos_xy)
        update_limits_expand_only(np.min(all_d), np.max(all_d), g_ylim_pos_xy)
        ax1.set_ylim(g_ylim_pos_xy[0], g_ylim_pos_xy[1])
    ax1.legend(loc='upper left', fontsize='small')

    # --- PLOT 2: VEL XY (Channels 4,5) ---
    data_vel_xy = []

    if g_payload_size >= 16:
      val4 = np.concatenate((g_val4[g_cur_idx:], g_val4[:g_cur_idx]))
      ax2.plot(g_line, val4, color=LINE_COLORS[3], label=f"{LABELS[3]}: {val4[-1]:.3f}") # Vel X
      data_vel_xy.append(val4)

    if g_payload_size >= 20:
      val5 = np.concatenate((g_val5[g_cur_idx:], g_val5[:g_cur_idx]))
      ax2.plot(g_line, val5, color=LINE_COLORS[4], label=f"{LABELS[4]}: {val5[-1]:.3f}") # Vel Y
      data_vel_xy.append(val5)

    if data_vel_xy:
        all_d = np.concatenate(data_vel_xy)
        update_limits_expand_only(np.min(all_d), np.max(all_d), g_ylim_vel_xy)
        ax2.set_ylim(g_ylim_vel_xy[0], g_ylim_vel_xy[1])
    ax2.legend(loc='upper left', fontsize='small')

anim = animation.FuncAnimation(fig, animate, frames=len(g_line) + 1, interval=33, blit=False)
plt.show()
