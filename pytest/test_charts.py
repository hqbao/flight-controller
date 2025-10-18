import math
import time
import matplotlib.pyplot as plt
import queue
from threading import Thread
from serial import Serial
import numpy as np
from matplotlib import animation
import serial.tools.list_ports

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

max_win_size = 128
g_line = np.linspace(start=0, stop=1, num=max_win_size)
g_val1 = np.zeros(max_win_size, int)
g_val2 = np.zeros(max_win_size, int)
g_val3 = np.zeros(max_win_size, int)
g_val4 = np.zeros(max_win_size, int)
g_val5 = np.zeros(max_win_size, int)
g_val6 = np.zeros(max_win_size, int)
g_cur_idx = 0

g_clazz = 0x00
g_clazz_id = 0x00

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
  global g_val, g_cur_idx, g_clazz
  while True:
    try:
      g_clazz, g_clazz_id, payload = queue1.get(timeout=0.001)
      if g_clazz == 0x00: # IMU
        if g_clazz_id == 0x05: # Acceleration
          x = int.from_bytes(payload[:4], 'little', signed='True')
          y = int.from_bytes(payload[4:8], 'little', signed='True')
          z = int.from_bytes(payload[8:12], 'little', signed='True')
          #print('X: {:.3f}\tY: {:.3f}\tZ: {:.3f}'.format(x, y, z))

          g_val1[g_cur_idx] = x
          g_val2[g_cur_idx] = y
          g_val3[g_cur_idx] = z
          g_cur_idx += 1
          if (g_cur_idx >= max_win_size): 
            g_cur_idx = 0

      elif g_clazz == 0x01: # IMU
        if g_clazz_id == 0x00:
          x = int.from_bytes(payload[:4], 'little', signed='True')
          y = int.from_bytes(payload[4:8], 'little', signed='True')
          z = int.from_bytes(payload[8:12], 'little', signed='True')
          w = int.from_bytes(payload[8:12], 'little', signed='True')
          #print('X: {:.3f}\tY: {:.3f}\tZ: {:.3f}'.format(x, y, z))

          g_val1[g_cur_idx] = x
          g_val2[g_cur_idx] = y
          g_val3[g_cur_idx] = z
          g_val4[g_cur_idx] = w
          g_cur_idx += 1
          if (g_cur_idx >= max_win_size): 
            g_cur_idx = 0

    except queue.Empty:
      pass

queue1 = queue.Queue()
in_thread1 = Thread(target=run_db_reader, args=(queue1,))
in_thread1.start()

in_thread2 = Thread(target=run_parser, args=())
in_thread2.start()

fig, ax = plt.subplots(1, 1, figsize=(6, 6))

def animate(i):
  global g_clazz, g_line, g_val1, g_val2, g_val3, g_val4, g_val5, g_val6

  ax.cla() # clear the previous image

  if g_clazz == 0x00:
    val1 = np.concatenate((g_val1[g_cur_idx:], g_val1[:g_cur_idx]))
    val2 = np.concatenate((g_val2[g_cur_idx:], g_val2[:g_cur_idx]))
    val3 = np.concatenate((g_val3[g_cur_idx:], g_val3[:g_cur_idx]))
    ax.plot(g_line, val1, color='blue')
    ax.plot(g_line, val2, color='red')
    ax.plot(g_line, val3, color='green')
    # ax.set_ylim([-90, 90])

  elif g_clazz == 0x01:
    val1 = np.concatenate((g_val1[g_cur_idx:], g_val1[:g_cur_idx]))
    val2 = np.concatenate((g_val2[g_cur_idx:], g_val2[:g_cur_idx]))
    val3 = np.concatenate((g_val3[g_cur_idx:], g_val3[:g_cur_idx]))
    val4 = np.concatenate((g_val4[g_cur_idx:], g_val4[:g_cur_idx]))
    ax.plot(g_line, val1, color='blue')
    ax.plot(g_line, val2, color='red')
    ax.plot(g_line, val3, color='green')
    ax.plot(g_line, val4, color='yellow')
    # ax.set_ylim([-500, 500])

anim = animation.FuncAnimation(fig, animate, frames=len(g_line) + 1, interval=1, blit=False)
plt.show()


