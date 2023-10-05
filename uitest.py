import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import serial 
from collections import deque
import random

# Connects to Serial port. Should change this to autodetect the device, also this is OS specific. 
ser = serial.Serial('/dev/ttyACM0', 115200)

# Create a UI main window
root = tk.Tk()
root.title("Real-time Matplotlib Plot")

# Global variable for the random toggle switch.
toggle_var = tk.BooleanVar()

# Create a Matplotlib figure with subplots, numbers change the size of plots and thus the UI 
fig = plt.figure(figsize=(18, 8))

# Add 2 subplots in a 2x1 grid
ax = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 1, 2)

# Initialize a deque to store data with a maximum length of 100
# This stores data to be displayed on the plot, increase lenght to get more datapoints
lengthOfDeque = 100   # Deques have to have same lenth, otherwise matplot will throw an error. 
data_buffer = deque(maxlen=lengthOfDeque)
data_buffer2 = deque(maxlen=lengthOfDeque)
desiredHeadingAr = deque(maxlen=lengthOfDeque)

# Fill deque with zeroes. This makes plots look better at program start 
# Plotting function will throw an error if deque is empty (Only on first loop)

while len(data_buffer) < data_buffer.maxlen:
    data_buffer.append(0)

while len(data_buffer2) < data_buffer2.maxlen:
    data_buffer2.append(0)

while len(desiredHeadingAr) < desiredHeadingAr.maxlen:
    desiredHeadingAr.append(0)


# Define the update function to be called for each animation frame
def update(frame):
    try: 
        data = ser.readline()
        
        data = data.decode().strip()  # Read data from serial port

        # split data by whitespace and convert to float
        data = data.split()

        if len(data) == 4:  # Ensure data is not empty
            value = float(data[1])  # Convert data to float
            data_buffer.append(value)  # Append value to the data buffer
            value = float(data[3])
            data_buffer2.append(value)
            desiredHeadingAr.append(desiredHeading) # Add desired heading from UI 
    except UnicodeDecodeError as e:
        print(f"Serial port error: {e}")

    finally: 
        pass

    # clears plots 
    ax.cla()     
    ax2.cla()

    # Plots data 

    ax.set_title('Heading')
    ax2.set_title('Output')

    ax.plot(range(len(data_buffer)),data_buffer)
    ax.plot(range(len(data_buffer)),desiredHeadingAr)
    ax2.plot(range(len(data_buffer2)),data_buffer2)

    ax.set_xlim(0, len(data_buffer))  # Set X-axis limits
    ax.set_ylim(0, 360)  # Set Y-axis limits

    ax2.set_xlim(0, len(data_buffer2))  # Set X-axis limits
    ax2.set_ylim(-300,300)  # Set Y-axis limits



# Create a tkinter canvas that can display Matplotlib figures
canvas = FigureCanvasTkAgg(fig, master=root)
canvas_widget = canvas.get_tk_widget()
canvas_widget.pack()

ani = animation.FuncAnimation(fig, update, frames=100, interval=25) 

def toggle():   # Random heading button
    global is_toggle_active
    is_toggle_active = toggle_var.get()

    if is_toggle_active:
        execute_periodically()

# Function to execute code every 15 seconds
def execute_periodically():
    if is_toggle_active:
        randomHeading = random.randint(0,359)

        sendSetRandomHeadingCommand(randomHeading)

    
        # Schedule the next execution
        root.after(15000, execute_periodically)



# Start code for the packet (CuteSat will reject commands that dont start with these two bytes.)
def addStartCode(data): 
    data.append(b'\xAB')
    data.append(b'\xCD')
    return data

# End code for the packet (CuteSat will reject commands that dont end with these two bytes.)
def addEndCode(data):
    data.append(b'\xEF')
    data.append(b'\x01')
    return data

# Function to transmit data to cutesat 
def sendCommand(data):
    dataToSend = b''

    for i in data:
        dataToSend += i

    ser.write(dataToSend)

# Function for calibrate command. 
def sendCalibrateCommmand():
    data = []
    data = addStartCode(data)
        
    data.append(b'\x01')    # Byte for calibrate mode 
    data.append(b'\x01')    # Message lenght byte
    data.append(b'\x01')    # byte for start calibration

    data = addEndCode(data)

    sendCommand(data)

# Function for idle command. 
def sendIdleCommand():
    data = []
    data = addStartCode(data)

    data.append(b'\x02')    # Byte for mode command 
    data.append(b'\x01')    # Message lenght byte

    data.append(b'\x00')    # Byte for idle mode

    data = addEndCode(data)

    sendCommand(data)

# Function for Detumble command. 
def sendDetumbleCommand():
    data = []
    data = addStartCode(data)

    data.append(b'\x02')    # Byte for mode command
    data.append(b'\x01')    # Message lenght byte

    data.append(b'\x01')    # Byte for detumble mode

    data = addEndCode(data)

    sendCommand(data)

# Function for Heading command 
def sendHeadingHoldCommand():
    data = []
    data = addStartCode(data)

    data.append(b'\x02')    # Byte for mode command 
    data.append(b'\x01')    # Message lenght byte 

    data.append(b'\x02')    # Byte for heading mode 

    data = addEndCode(data)

    sendCommand(data)

# Function for set heading 
def sendSetHeadingCommand(): 
    data = []
    data = addStartCode(data)

    data.append(b'\x03')    # Bytes for heading command 
    data.append(b'\x02')    # Message lenght byte

    heading = slider.get()  # Get value from slider 

    if heading >=360:       
        heading = 0

    global desiredHeading   # Set global variable (Used for UI)
    desiredHeading = heading

    # Convert to bytes (2 bytes)
    heading = heading.to_bytes(2, byteorder='big', signed=False)

    data.append(heading)

    data = addEndCode(data)
    sendCommand(data)

# Sets random heading 
def sendSetRandomHeadingCommand(heading): 
    data = []
    data = addStartCode(data)

    data.append(b'\x03')    # Bytes for heading command 
    data.append(b'\x02')    # Message lenght byte

    global desiredHeading
    desiredHeading = heading

    heading = heading.to_bytes(2, byteorder='big', signed=False)
    
    data.append(heading)

    data = addEndCode(data)
    sendCommand(data)


# Create multiple buttons and pack them next to each other horizontally
button1 = tk.Button(root, text="Calibrate", command=sendCalibrateCommmand)
button1.pack(side="left", padx=5)  # Use side="left" to place buttons horizontally with padding

button2 = tk.Button(root, text="Mode: Idle", command=sendIdleCommand)
button2.pack(side="left", padx=5)

button3 = tk.Button(root, text="Mode: Detumble", command=sendDetumbleCommand)
button3.pack(side="left", padx=5)

button4 = tk.Button(root, text="Mode: Heading", command=sendHeadingHoldCommand)
button4.pack(side="left", padx=5)

button5 = tk.Button(root, text="Set Heading", command=sendSetHeadingCommand)
button5.pack(pady=10)

slider = tk.Scale(root, from_=0, to=360, orient="horizontal", label="Heading")
slider.pack(pady=10)

toggle_button = tk.Checkbutton(root, text="Random Heading", variable=toggle_var, command=toggle)
toggle_button.pack()

is_toggle_active = False

desiredHeading = 0 

# Start the main event loop
root.mainloop()

