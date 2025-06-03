import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# === CONFIG ===
PORT = 'COM3'  # Or '/dev/ttyUSB0' or similar for Linux/WSL
BAUD = 115200
NUM_VALUES = 5
WINDOW_SIZE = 100  # Number of points to display

# === SETUP ===
ser = serial.Serial(PORT, BAUD, timeout=1)

data_buffers = [deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE) for _ in range(NUM_VALUES)]

fig, ax = plt.subplots()
lines = [ax.plot([], [])[0] for _ in range(NUM_VALUES)]
ax.set_ylim(-500, 500)  # Set appropriately
ax.set_xlim(0, WINDOW_SIZE)

def update(frame):
    try:
        line = ser.readline().decode('utf-8').strip()
        values = [float(val) for val in line.split(',')]
        if len(values) != NUM_VALUES:
            return lines

        for i in range(NUM_VALUES):
            data_buffers[i].append(values[i])
            lines[i].set_data(range(WINDOW_SIZE), data_buffers[i])

    except Exception as e:
        print("Error:", e)

    return lines

ani = animation.FuncAnimation(fig, update, interval=50)
plt.show()
