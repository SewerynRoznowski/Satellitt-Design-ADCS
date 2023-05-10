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
        
        // Request 6 bytes from magnetometer
        Wire.requestFrom(magAddr,6);

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

    void getAccelData(int& accelX, int& accelY, int& accelZ){
        // Get accelerometer data
        Wire.beginTransmission(accelAddr);
        Wire.write(0x28);
        Wire.endTransmission();

        Wire.requestFrom(accelAddr,6);

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
        // Get gyroscope data
        Wire.beginTransmission(accelAddr);
        Wire.write(0x22);
        Wire.endTransmission();

        Wire.requestFrom(accelAddr,6);

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
    
    SatReactionWheelSingleton(){
        pinMode(motorPWM, OUTPUT);
        pinMode(motorEnA, OUTPUT);
        pinMode(motorEnB, OUTPUT);
    }

    
    void changeMotorSpeed(int speedDelta){
        currentPWM = currentPWM + speedDelta;
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

    int getCurrentPWM(){
        return currentPWM;
    }
};

SatSensorSingelton SatSensor;
SatReactionWheelSingleton SatReactionWheel;



#endif /* SATADCS_H_ */
