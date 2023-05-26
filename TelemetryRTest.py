import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque


ser = serial.Serial('/dev/ttyACM0', 115200)  # replace with your serial port and baud rate

# Create a figure and axis
fig = plt.figure()

# Add 2 subplots in a 2x1 grid
ax = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 1, 2)

# Set labels and titles
ax.set_xlabel('Time')
ax.set_ylabel('Heading')

ax2.set_xlabel('Time')
ax2.set_ylabel('Torque')

ax.set_title('Heading')
ax2.set_title('Torque')




# Initialize a deque to store data with a maximum length of 100
data_buffer = deque(maxlen=100)
data_buffer2 = deque(maxlen=100)

# Initialize an empty line object (we will update this later)
line, = ax.plot([], [])
line2, = ax2.plot([], [])

# Define the update function to be called for each animation frame
def update(frame):
    data = ser.readline().decode().strip()  # Read data from serial port

    # split data by whitespace and convert to float
    data = data.split()

    if len(data) == 4:  # Ensure data is not empty
        value = float(data[1])  # Convert data to float
        data_buffer.append(value)  # Append value to the data buffer
        value = float(data[3])
        data_buffer2.append(value)

    line.set_data(range(len(data_buffer)), data_buffer)  # Update the line object with new data
    line2.set_data(range(len(data_buffer2)), data_buffer2)
    ax.set_xlim(0, len(data_buffer))  # Set X-axis limits
    ax.set_ylim(0, 360)  # Set Y-axis limits

    ax2.set_xlim(0, len(data_buffer2))  # Set X-axis limits
    ax2.set_ylim(-300,300)  # Set Y-axis limits

    return line,

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=100, interval=100)

# Display the plot
plt.show()        