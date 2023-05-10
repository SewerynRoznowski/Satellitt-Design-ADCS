import serial
import time

ser = serial.Serial('/dev/ttyACM2', 115200)  # replace with your serial port and baud rate


while True:
# read two bytes from the serial port
    data_bytes = ser.readline().decode().strip()

    print(f'{data_bytes}')

        