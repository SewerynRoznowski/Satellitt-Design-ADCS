import serial
import time

ser = serial.Serial('/dev/ttyACM2', 115200)  # replace with your serial port and baud rate

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

    elif user_input == "Set speed":
        user_input = input("Enter a speed: ")

        user_input = int(user_input)

        data.append(b'\x02')
        data.append(b'\x02')
        if user_input < 0: 
            data.append(b'\x00')
        else:
            data.append(b'\x01')

        user_input = abs(user_input)
        if  user_input > 255:
            user_input = 255
        
        user_input = user_input.to_bytes(1, byteorder='big', signed=False)
        
        data.append(user_input)
        

        data = addEndCode(data)

        print("Sending Set speed command")

        dataToSend = b''

        for i in data:
            dataToSend += i

        ser.write(dataToSend)
    

        