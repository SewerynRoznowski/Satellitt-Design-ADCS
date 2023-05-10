import serial
import time

ser = serial.Serial('/dev/ttyACM2', 115200)  # replace with your serial port and baud rate

value1 = 0xAB
value1 = value1.to_bytes(1, byteorder='big', signed=False)

value2 = 0xCD
value2 = value2.to_bytes(1, byteorder='big', signed=False)

value3 = 0xEF
value3 = value3.to_bytes(1, byteorder='big', signed=False)

value4 = 0x01
value4 = value4.to_bytes(1, byteorder='big', signed=False)

command = 0x02
command = command.to_bytes(1, byteorder='big', signed=False)

datalenght = 0x02
datalenght = datalenght.to_bytes(1, byteorder='big', signed=False)

direction = 0x00
direction = direction.to_bytes(1, byteorder='big', signed=False)

speed = 150 
speed = speed.to_bytes(1, byteorder='big', signed=False)
print(speed)

print (value1+value2+command+datalenght+direction+speed+value3+value4)
ser.write(value1+value2+command+datalenght+direction+speed+value3+value4)