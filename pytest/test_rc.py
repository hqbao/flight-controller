import math
import time
import matplotlib.pyplot as plt
import queue
import numpy as np
import struct
import serial.tools.list_ports
from threading import Thread
from serial import Serial
from matplotlib import animation
from matplotlib.widgets import Button, Slider


g_baud_rate = 9600
g_serial_port = None
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
  print("{}: {} [{}]".format(port, desc, hwid))
  if port.startswith('/dev/cu.usbmodem') or port.startswith('/dev/cu.usbserial') or port.startswith('/dev/cu.SLAB_USBtoUART'):
    g_serial_port = port

if g_serial_port is None:
  print('No serial port found')
  # exit()

g_thread1_running = True
g_thread2_running = True

max_win_size = 64
g_line = np.linspace(start=0, stop=1, num=max_win_size)
g_val1 = np.zeros(max_win_size, int)
g_val2 = np.zeros(max_win_size, int)
g_val3 = np.zeros(max_win_size, int)
g_cur_idx = 0

g_clazz = 0x00
g_clazz_id = 0x00

g_stream = None

g_roll = 0
g_pitch = 0
g_yaw = 0

g_setpoint_roll = 0
g_setpoint_pitch = 0
g_setpoint_yaw = 0
g_set_point_alt = 0


def run_db_reader(in_queue):
  global g_thread1_running, g_stream

  while g_thread1_running:
    try:
      with Serial(g_serial_port, g_baud_rate, timeout=3) as stream:
        g_stream = stream
        while g_thread1_running:
          byte = stream.read(1)
          if len(byte) == 1 and (byte[0] == 0x64 or byte[0] == 0x62): # 'd' or 'b'
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

          time.sleep(0.00001)

    except:
      stream = None
      g_stream = None
      pass

    time.sleep(0.1)

def run_parser(in_queue):
  global g_thread2_running, g_cur_idx, g_clazz, g_val1, g_val2, g_val3, g_roll, g_pitch, g_yaw

  while g_thread2_running:
    try:
      g_clazz, g_clazz_id, payload = in_queue.get(timeout=0.001)
      if g_clazz == 0x02 and g_clazz_id == 0x01:
        g_roll = struct.unpack('<f', payload[:4])[0]
        g_pitch = struct.unpack('<f', payload[4:8])[0]
        g_yaw = struct.unpack('<f', payload[8:12])[0]
        # print(g_roll, g_pitch, g_yaw)

        g_val1[g_cur_idx] = g_roll
        g_val2[g_cur_idx] = g_pitch
        g_val3[g_cur_idx] = g_yaw

        g_cur_idx += 1
        if (g_cur_idx >= max_win_size): 
          g_cur_idx = 0

    except queue.Empty:
      pass

    time.sleep(0.00001)


def animate(i):
  global g_clazz, g_line, g_val1, g_val2, g_val3

  ax.cla() # clear the previous image

  if g_clazz == 0x02:
    val1 = np.concatenate((g_val1[g_cur_idx:], g_val1[:g_cur_idx]))
    val2 = np.concatenate((g_val2[g_cur_idx:], g_val2[:g_cur_idx]))
    val3 = np.concatenate((g_val3[g_cur_idx:], g_val3[:g_cur_idx]))
    ax.plot(g_line, val1, color='red')
    ax.plot(g_line, val2, color='blue')
    ax.plot(g_line, val3, color='green')
    # ax.set_ylim([-2000, 2000])

fig, ax = plt.subplots(1, 1, figsize=(16, 6))
anim = animation.FuncAnimation(fig, animate, frames=len(g_line) + 1, interval=1, blit=False)


def on_roll_update(value):
  global g_stream, g_setpoint_roll, g_setpoint_pitch, g_setpoint_yaw, g_set_point_alt
  g_setpoint_roll = value

  if g_stream is not None:
    data = bytearray([0x64, 0x62, 0x01, 0x02, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    data[6:10] = int(g_setpoint_roll).to_bytes(4, 'little', signed=True)
    data[10:14] = int(g_setpoint_pitch).to_bytes(4, 'little', signed=True)
    data[14:18] = int(g_setpoint_yaw).to_bytes(4, 'little', signed=True)
    data[18:22] = int(g_set_point_alt).to_bytes(4, 'little', signed=True)
    g_stream.write(data)
    print('Wrote to stream', data.hex())

ax_roll = fig.add_axes([0.01, 0.1, 0.02, 0.8])
g_roll_slider = Slider(
  ax=ax_roll,
  label="Roll",
  valmin=-900,
  valmax=900,
  valstep=1,
  valinit=0,
  orientation="vertical"
)
g_roll_slider.on_changed(on_roll_update)


def on_pitch_update(value):
  global g_stream, g_setpoint_roll, g_setpoint_pitch, g_setpoint_yaw, g_set_point_alt
  g_setpoint_pitch = value

  if g_stream is not None:
    data = bytearray([0x64, 0x62, 0x01, 0x02, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    data[6:10] = int(g_setpoint_roll).to_bytes(4, 'little', signed=True)
    data[10:14] = int(g_setpoint_pitch).to_bytes(4, 'little', signed=True)
    data[14:18] = int(g_setpoint_yaw).to_bytes(4, 'little', signed=True)
    data[18:22] = int(g_set_point_alt).to_bytes(4, 'little', signed=True)
    g_stream.write(data)
    print('Wrote to stream', data.hex())

ax_pitch = fig.add_axes([0.03, 0.1, 0.02, 0.8])
g_pitch_slider = Slider(
  ax=ax_pitch,
  label="Pitch",
  valmin=-900,
  valmax=900,
  valstep=1,
  valinit=0,
  orientation="vertical"
)
g_pitch_slider.on_changed(on_pitch_update)


def on_yaw_update(value):
  global g_stream, g_setpoint_roll, g_setpoint_pitch, g_setpoint_yaw, g_set_point_alt
  g_setpoint_yaw = value

  if g_stream is not None:
    data = bytearray([0x64, 0x62, 0x01, 0x02, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    data[6:10] = int(g_setpoint_roll).to_bytes(4, 'little', signed=True)
    data[10:14] = int(g_setpoint_pitch).to_bytes(4, 'little', signed=True)
    data[14:18] = int(g_setpoint_yaw).to_bytes(4, 'little', signed=True)
    data[18:22] = int(g_set_point_alt).to_bytes(4, 'little', signed=True)
    g_stream.write(data)
    print('Wrote to stream', data.hex())

ax_yaw = fig.add_axes([0.05, 0.1, 0.02, 0.8])
g_yaw_slider = Slider(
  ax=ax_yaw,
  label="Yaw",
  valmin=-900,
  valmax=900,
  valstep=1,
  valinit=0,
  orientation="vertical"
)
g_yaw_slider.on_changed(on_yaw_update)


def on_alt_update(value):
  global g_stream, g_setpoint_roll, g_setpoint_pitch, g_setpoint_yaw, g_set_point_alt
  g_set_point_alt = value

  if g_stream is not None:
    data = bytearray([0x64, 0x62, 0x01, 0x02, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    data[6:10] = int(g_setpoint_roll).to_bytes(4, 'little', signed=True)
    data[10:14] = int(g_setpoint_pitch).to_bytes(4, 'little', signed=True)
    data[14:18] = int(g_setpoint_yaw).to_bytes(4, 'little', signed=True)
    data[18:22] = int(g_set_point_alt).to_bytes(4, 'little', signed=True)
    g_stream.write(data)
    print('Wrote to stream', data.hex())

ax_alt = fig.add_axes([0.07, 0.1, 0.02, 0.8])
g_alt_slider = Slider(
  ax=ax_alt,
  label="Yaw",
  valmin=-1000,
  valmax=1000,
  valstep=1,
  valinit=0,
  orientation="vertical"
)
g_alt_slider.on_changed(on_alt_update)


def on_reset_clicked(event):
  print('on_reset_clicked')
  g_roll_slider.reset()
  g_pitch_slider.reset()
  g_yaw_slider.reset()
  g_alt_slider.reset()

btn1_frame = plt.axes([0.125, 0.0, 0.05, 0.05])
btn1 = Button(btn1_frame, 'Reset', color='Gray', hovercolor='White')
btn1.on_clicked(on_reset_clicked)


def on_plot_close(event):
  global g_thread1_running, g_thread2_running
  print("Close")
  g_thread1_running = False
  g_thread2_running = False

fig.canvas.mpl_connect('close_event', on_plot_close)


queue1 = queue.Queue()
in_thread1 = Thread(target=run_db_reader, args=(queue1,))
in_thread1.start()
in_thread2 = Thread(target=run_parser, args=(queue1,))
in_thread2.start()


plt.show()
