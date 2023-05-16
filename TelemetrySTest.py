import serial
import time
import struct

ser = serial.Serial('/dev/ttyACM0', 115200)  # replace with your serial port and baud rate

def addStartCode(data):
    data.append(b'\xAB')
    data.append(b'\xCD')
    return data

def addEndCode(data):
    data.append(b'\xEF')
    data.append(b'\x01')
    return data

while True:
# read two bytes from the serial port
    user_input = input("Enter a command: ")
    data = []
    data = addStartCode(data)
    if user_input == "Calibrate":
        data.append(b'\x01')
        data.append(b'\x01')
        data.append(b'\x01')
        
        data = addEndCode(data)
        
        print("Sending Calibrate command")
        
        dataToSend = b''

        for i in data:
            dataToSend += i
        
        ser.write(dataToSend)

    elif user_input == "Set mode":
        user_input = input("Set mode, 1. none, 2. detumble, 3. heading hold: ")

        user_input = int(user_input)

        data.append(b'\x02')
        data.append(b'\x01')
        if user_input < 0: 
            data.append(b'\x00')
        elif user_input > 3:
            data.append(b'\x00')
        else: 
            user_input = user_input.to_bytes(1, byteorder='big', signed=False)
            data.append(user_input)
        
        data = addEndCode(data)

        print("Sending Set speed command")

        dataToSend = b''

        for i in data:
            dataToSend += i

        ser.write(dataToSend)

    elif user_input == "Set heading":
        user_input = input("Enter a heading: ")

        user_input = int(user_input)    

        if user_input < 0: 
            user_input = 0

        if user_input > 359:
            user_input = 359

        data.append(b'\x03')
        data.append(b'\x02')

        user_input = user_input.to_bytes(2, byteorder='big', signed=False) 

        print(user_input)

        data.append(user_input)
    

        data = addEndCode(data)

        print("Sending Set heading command")

        dataToSend = b''
        for i in data:
            dataToSend += i
        
        ser.write(dataToSend)
    
    elif user_input == "Set gains":
        
        data.append(b'\x05')
        data.append(b'\x03')

        user_input = input("Enter a Kp: ")

        user_input = int(user_input)

        if user_input < 0:
            user_input = 0

        if user_input > 255:
            user_input = 255

        user_input = user_input.to_bytes(1, byteorder='big', signed=False)

        data.append(user_input)

        user_input = input("Enter a Ki: ")

        user_input = int(user_input)

        if user_input < 0:
            user_input = 0

        if user_input > 255:
            user_input = 255

        user_input = user_input.to_bytes(1, byteorder='big', signed=False)

        data.append(user_input)

        user_input = input("Enter a Kd: ")

        user_input = int(user_input)

        if user_input < 0:
            user_input = 0

        if user_input > 255:
            user_input = 255

        user_input = user_input.to_bytes(1, byteorder='big', signed=False)

        data.append(user_input)

        data = addEndCode(data)

        print("Sending Set gains command")

        dataToSend = b''
        for i in data:
            dataToSend += i
        
        ser.write(dataToSend)

    elif user_input == "Reset PID":
        
        data.append(b'\x06')
        data.append(b'\x01')

        data.append(b'\x01')

        data = addEndCode(data)

        print("Sending Reset PID command")

        dataToSend = b''
        for i in data:
            dataToSend += i

        ser.write(dataToSend)
        
    

    

        