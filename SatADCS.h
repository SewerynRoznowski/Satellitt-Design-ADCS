/*
 * SatADCS.h
 *
 *  Created on: May 9, 2023
 *      Author: sew
 */
#include <Wire.h>
#include <Energia.h>
#ifndef SATADCS_H_
#define SATADCS_H_

const int accelAddr = 0x6b;
const int magAddr = 0x30;

const int motorPWM = P1_4;
const int motorEnA = P3_2;
const int motorEnB = P2_7;

const int maxPWM = 255; 
const int minPWM = 30;

int currentPWM = 0;

bool calibrationMode = false;

int commandBuffer[10];

// Class contaning all functions related to the sensors
class SatSensorSingelton { 
    public: 

    // Initializes the accelerometer and gyroscope
    void initializeAccelGyro(){
        Wire.beginTransmission(accelAddr);  // Write to accelerometer and gyroscope
        Wire.write(0x10);                   // Set register pointer to 0x10
        Wire.write(0xA0);                   // Set register 0x10 to 0xA0 (see datasheet for detials)
        Wire.endTransmission();             // End transmission
    }

    // Initializes the magnetometer
    void initializeMag(){
        Wire.beginTransmission(magAddr);    // Write to magnetometer
        Wire.write(0x0A);                   // Ser register pointer to 0x0A
        Wire.write(0x00);                   // Set register 0x0A to 0x00 (see datasheet for detials)
        Wire.write(0xBD);                   // Set register 0x0B to 0xBD (register pointer increases by 1 after each write)
        Wire.endTransmission();             // End transmission
    }

    // Returns the raw magnetometer data, pass by reference
    void getMagData (int& magX , int& magY , int& magZ) {
        
        Wire.beginTransmission(magAddr);    // Write to magnetometer
        Wire.write(0x00);                   // Set register pointer to 0x00
        Wire.endTransmission();             // End transmission
        
        Wire.requestFrom(magAddr,6);        // Request 6 bytes from magnetometer

        // Read 6 bytes from magnetometer
        int magXhRaw = Wire.read();
        int magXlRaw = Wire.read();
        int magYhRaw = Wire.read();
        int magYlRaw = Wire.read();
        int magZhRaw = Wire.read();
        int magZlRaw = Wire.read();

        // Merge together registers
        uint umagX = (magXhRaw << 8) | magXlRaw;
        uint umagY = (magYhRaw << 8) | magYlRaw;
        uint umagZ = (magZhRaw << 8) | magZlRaw;

        // Subtract 32768 to get signed 16-bit value
        magX = umagX - 32768;
        magY = umagY - 32768;
        magZ = umagZ - 32768;
    }

    // Returns the raw accelerometer data, pass by reference
    void getAccelData(int& accelX, int& accelY, int& accelZ){
        
        Wire.beginTransmission(accelAddr);  // Write to accelerometer and gyroscope
        Wire.write(0x28);                   // Set register pointer to 0x28
        Wire.endTransmission();             // End transmission

        Wire.requestFrom(accelAddr,6);      // Request 6 bytes from accelerometer

        // Read 6 bytes from accelerometer
        int accelXl = Wire.read();
        int accelXh = Wire.read();
        int accelYl = Wire.read();
        int accelYh = Wire.read();
        int accelZl = Wire.read();
        int accelZh = Wire.read();

        // Merge together registers
        accelX = (accelXh << 8) | accelXl;
        accelY = (accelYh << 8) | accelYl;
        accelZ = (accelZh << 8) | accelZl; 
    }

    void getGyroData(int& gyroX, int& gyroY, int& gyroZ){
        
        Wire.beginTransmission(accelAddr);  // Write to accelerometer and gyroscope
        Wire.write(0x22);                   // Set register pointer to 0x22
        Wire.endTransmission();             // End transmission

        Wire.requestFrom(accelAddr,6);      // Request 6 bytes from gyroscope

        // Read 6 bytes from gyroscope
        int gyroXl = Wire.read();
        int gyroXh = Wire.read();
        int gyroYl = Wire.read();
        int gyroYh = Wire.read();
        int gyroZl = Wire.read();
        int gyroZh = Wire.read();

        // Merge together registers
        gyroX = (gyroXh << 8) | gyroXl;
        gyroY = (gyroYh << 8) | gyroYl;
        gyroZ = (gyroZh << 8) | gyroZl;

    }
};

// Class contaning all functions related to the reaction wheel
class SatReactionWheelSingleton{

public:
    
    // Constructor set all the pins for the reaction wheel
    SatReactionWheelSingleton(){
        pinMode(motorPWM, OUTPUT);
        pinMode(motorEnA, OUTPUT);
        pinMode(motorEnB, OUTPUT);
    }

    // Changes the speed of the reaction wheel
    void changeMotorSpeed(int speedDelta){
        currentPWM = currentPWM + speedDelta;
        
        // Check if the speed is within the limits
        if (currentPWM > maxPWM){       
            currentPWM = maxPWM;
        }
        else if (currentPWM < -255){
            currentPWM = -maxPWM;
        }

        // Set the speed of the reaction wheel
        if (abs(currentPWM) < minPWM){      
            digitalWrite(motorEnA, LOW);
            digitalWrite(motorEnB, LOW);
            analogWrite(motorPWM, 0);
        }
        else if (currentPWM > 0){
            digitalWrite(motorEnA, HIGH);
            digitalWrite(motorEnB, LOW);
            analogWrite(motorPWM, currentPWM);
        }
        else if (currentPWM < 0){
            digitalWrite(motorEnA, LOW);
            digitalWrite(motorEnB, HIGH);
            analogWrite(motorPWM, abs(currentPWM));
        }
    }
    
    // Sets the speed of the reaction wheel
    void setMotorSpeed(int speed){
        currentPWM = speed;
        if (currentPWM > maxPWM){
            currentPWM = maxPWM;
        }
        else if (currentPWM < -255){
            currentPWM = -maxPWM;
        }

        if (abs(currentPWM) < minPWM){
            digitalWrite(motorEnA, LOW);
            digitalWrite(motorEnB, LOW);
            analogWrite(motorPWM, 0);
        }
        else if (currentPWM > 0){
            digitalWrite(motorEnA, HIGH);
            digitalWrite(motorEnB, LOW);
            analogWrite(motorPWM, currentPWM);
        }
        else if (currentPWM < 0){
            digitalWrite(motorEnA, LOW);
            digitalWrite(motorEnB, HIGH);
            analogWrite(motorPWM, abs(currentPWM));
        }
    }

    // Returns the current speed of the reaction wheel
    int getCurrentPWM(){
        return currentPWM;
    }
};

SatReactionWheelSingleton SatReactionWheel;

class SatTelemetrySingleton{
    public: 

    void recieveData(){
        if (Serial1.available() > 0){
            if (checkStartCode()){
                int command = Serial1.read();
                int dataLength = Serial1.read();
                int data[dataLength];
                for (int i = 0; i < dataLength; i++){
                    data[i] = Serial1.read();
                } 
                if(checkEndCode()){
                    commandBuffer[0] = command;
                    for (int i = 0; i < dataLength; i++){
                        commandBuffer[i+1] = data[i];
                    }
                }
            }

        }
    }

    void sendData(){
    
    }
    bool checkStartCode(){
        
        int inByte = Serial1.read();

        if (inByte == 0xAB){
            int inByte2 = Serial1.read();
            if (inByte2 == 0xCD){
                
                return 1;
            } else {
                return 0;
            }
        } else {
            return 0;
        }
    }

    bool checkEndCode(){
        int inByte = Serial1.read();
        if (inByte == 0xEF){
            int inByte2 = Serial1.read();
            if (inByte2 == 0x01){
                return 1;
            } else {
                return 0;
            }
        } else {
            return 0;
        }
    }

    void getData(){
    }
};

// Class containing all the commands that can be sent to the satellite
class SatCommandsSingleton{
    public: 
    void commandSelect(){
        switch (commandBuffer[0])
        {
        case 0x00: // do nothing            
            break;
        
        case 0x01: // Enable calibration mode
            if (commandBuffer[1] == 0x01){
                calibrationMode = true;
            } else if (commandBuffer[1] == 0x00){
                calibrationMode = false;
            }
            
            commandBuffer[0] = 0x00; // Reset command buffer1
            break;

        case 0x02: // Set reaction wheel speed
            if (commandBuffer[1] == 0x00){
                SatReactionWheel.setMotorSpeed(-commandBuffer[2]);
            } else if (commandBuffer[1] == 0x01){
                SatReactionWheel.setMotorSpeed(commandBuffer[2]);
            }
            commandBuffer[0] = 0x00; // Reset command buffer
            break;

        default: // do nothing if no command is selected
            break;
        }
    }
};


SatSensorSingelton SatSensor;
SatTelemetrySingleton SatTelemetry;
SatCommandsSingleton SatCommands;



#endif /* SATADCS_H_ */
