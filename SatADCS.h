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


// Class contaning all functions related to the sensors
class SatSensorSingelton { 
private:
    static const int accelAddr = 0x6b;
    static const int magAddr = 0x30;
public: 

    int magXRaw; 
    int magYRaw;
    int magZRaw;

    int accelXRaw;
    int accelYRaw;
    int accelZRaw;

    int gyroXRaw;
    int gyroYRaw;
    int gyroZRaw;

    float heading; 

    int magXOffset;
    int magYOffset;
    
    SatSensorSingelton (){
    }

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

    void updateSensorData(){
        getMagData();
        getAccelData();
        getGyroData();
        calcHeading();
    }

    // Returns the raw magnetometer data, pass by reference
    void getMagData () {
        
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
        magXRaw = umagX - 32768;
        magYRaw = umagY - 32768;
        magZRaw = umagZ - 32768;
    }

    // Returns the raw accelerometer data, pass by reference
    void getAccelData(){
        
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
        accelXRaw = (accelXh << 8) | accelXl;
        accelYRaw = (accelYh << 8) | accelYl;
        accelZRaw = (accelZh << 8) | accelZl; 
    }

    void getGyroData(){
        
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
        gyroXRaw = (gyroXh << 8) | gyroXl;
        gyroYRaw = (gyroYh << 8) | gyroYl;
        gyroZRaw = (gyroZh << 8) | gyroZl;

    }

    void calcHeading(){
        int magX = magXRaw - magXOffset;
        int magY = magYRaw - magYOffset;
        
        // Calculate heading from magnetometer data
        heading = atan2(-magY, magX) * 180/PI;

        // Correct for when signs are reversed.
        if(heading < 0)
        {
            heading = 360 + heading;
        }
    }
};

// Class contaning all functions related to the reaction wheel
class SatReactionWheelSingleton{

private: 
    static const int motorPWM = P1_4;
    static const int motorEnA = P3_2;
    static const int motorEnB = P2_7;

    static const int maxPWM = 80;
    static const int minPWM = 5;
    
    int currentPWM;

public:
    // Constructor set all the pins for the reaction wheel
    SatReactionWheelSingleton(){
        pinMode(motorPWM, OUTPUT);
        pinMode(motorEnA, OUTPUT);
        pinMode(motorEnB, OUTPUT);
        
        currentPWM = 0;
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
        else if (currentPWM < -maxPWM){
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

class SatTelemetrySingleton{
private:
    int telemetryMode; 

public: 

    SatTelemetrySingleton(){
        telemetryMode = 0;
    }

    void recieveData(int* commandBuffer){
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

    void setTelemetryMode(int mode){
        telemetryMode = mode;
    }
};

class SatPIDSingleton{
private: 
    float Kp;
    float Ki;
    float Kd;
    
    float angularVelocity;
    float angularVelocityOld;

    float currentHeading;
    float oldHeading; 
    float desiredHeading;
    float headingError;

    float integral;
    float integralOld;

    unsigned long timeOld;
    unsigned long timeNew;
    double timeStep;

    float output;
public: 

    int modeOfOperation;
    SatPIDSingleton(){
    Kp = 2;
    Ki = 0.1;
    Kd = 0.7;

    angularVelocity = 0;
    angularVelocityOld = 0;

    currentHeading = 0;
    oldHeading = 0;
    desiredHeading = 0;
    headingError = 0;

    integral = 0;
    integralOld = 0;

    timeOld = 0;
    timeNew = 0;
    timeStep = 0;

    output = 0;

    modeOfOperation = 0;
    }
    
    void setKp(float Kp){
        this->Kp = Kp;
    }

    void setKi(float Ki){
        this->Ki = Ki;
    }

    void setKd(float Kd){
        this->Kd = Kd;
    }

    void resetIntergral(){
        integral = 0;
        integralOld = 0;
    }

    void setDesiredHeading(float desiredHeading){
        this->desiredHeading = desiredHeading;
    }
    
    float headingHoldLoop(float heading){
        timeNew = millis();
        timeStep = (timeNew - timeOld);
        double timeStep2 = timeStep / 1000;

        currentHeading = heading;
        headingError = currentHeading - desiredHeading;
        if (headingError > 180){
            headingError = headingError - 360;
        }
        else if (headingError < -180){
            headingError = headingError + 360;
        }

        float headingStep = currentHeading - oldHeading;
        if (headingStep > 180){
            headingStep = headingStep - 360;
        }
        else if (headingStep < -180){
            headingStep = headingStep + 360;
        }

        oldHeading = currentHeading;

        angularVelocity = headingStep / timeStep2;

        timeOld = timeNew;

        if (abs(output) < 245){
            integral = integralOld + (timeStep2 * Ki * headingError);    
        }

        integralOld = integral;

        //output = (-Kp*headingError-Kd*angularVelocity + integral);
        output = (-Kp*headingError-Kd*(angularVelocity) - integral);

        return output;
    }

    float detumbleLoop(float heading){
        timeNew = millis();
        timeStep = (timeNew - timeOld);
        double timeStep2 = timeStep / 1000;

        currentHeading = heading;
        float headingStep = currentHeading - oldHeading;
        if (headingStep > 180){
            headingStep = headingStep - 360;
        }
        else if (headingStep < -180){
            headingStep = headingStep + 360;
        }

        oldHeading = currentHeading;
        timeOld = timeNew;
        
        angularVelocity = headingStep / timeStep2;

        double angularAcceleration = (angularVelocity - angularVelocityOld) / timeStep2;

        angularVelocityOld = angularVelocity;

        if (abs(output) < 245){
            integral = integralOld + (timeStep2 * Ki * angularVelocity);    
        }

        integralOld = integral;

        output = (-Kp*angularVelocity*0.1-Kd*angularAcceleration*0.1 - integral);

        if (abs(angularVelocity) < 5){
            modeOfOperation = 2;
            desiredHeading = currentHeading;
            Serial1.println("Detumble complete");
        }

        return output;
    }
};


// Class containing all the commands that can be sent to the satellite
class SatCommandsSingleton{
private:
    bool calibrationMode;

    void clearCommandBuffer(){
        for (int i = 0; i < 32; i++){
            commandBuffer[i] = 0x00;
        }
    }

public: 
    int commandBuffer[32];
    SatCommandsSingleton(){
        for (int i = 0; i < 32; i++){
            commandBuffer[i] = 0x00;
        }
        calibrationMode = false;
    }

    int test1;

    SatSensorSingelton SatSensor;
    SatTelemetrySingleton SatTelemetry;
    SatReactionWheelSingleton SatReactionWheel;
    SatPIDSingleton SatPID;

    void commandSelect(){

        float heading = 0;
        float gainP = 0;
        float gainI = 0;
        float gainD = 0;
        int temp = 0;

        switch (commandBuffer[0])
        {
        case 0x00: // do nothing            
            break;
        
        case 0x01: // Enable calibration mode
            if (commandBuffer[1] == 0x01){
                calibrateMag();
            }
            clearCommandBuffer(); // Reset command buffer1
            break;

        case 0x02: // Mode of operation
            
            SatPID.modeOfOperation = commandBuffer[1];
            SatPID.resetIntergral();
            Serial1.println(commandBuffer[1]);
            clearCommandBuffer(); // Reset command buffer
            break;

        case 0x03: 
            // combine 2 8bit values into 1 16bit value
            heading = (commandBuffer[1] << 8) | commandBuffer[2];
            SatPID.setDesiredHeading(heading);
            Serial1.println(heading);
            clearCommandBuffer(); // Reset command buffer
            break; 
        
        case 0x04: // Set telemetry mode
            SatTelemetry.setTelemetryMode(commandBuffer[1]);
            clearCommandBuffer(); // Reset command buffer
            break;

        case 0x05: // Set PID gains 

            temp = commandBuffer[1];
            gainP = temp / 10.0;

            temp = commandBuffer[2];
            gainI = temp / 10.0;

            temp = commandBuffer[3];
            gainD = temp / 10.0;

            SatPID.setKp(gainP);
            SatPID.setKi(gainI);
            SatPID.setKd(gainD);
            clearCommandBuffer(); // Reset command buffer
            break; 
            
        case 0x06: // Reset PID
            SatPID.resetIntergral();
            clearCommandBuffer(); // Reset command buffer
            break;

        case 0x07: // place holder
            clearCommandBuffer(); // Reset command buffer
            break;

        default: // do nothing if no command is selected
            clearCommandBuffer(); // Reset command buffer
            break;
        }
    }

    void calibrateMag(){
        Serial1.println("Calibrating magnetometer");       
        Serial1.println("Resetting offsets");

        // Reset offsets
        SatSensor.magXOffset = 0;
        SatSensor.magYOffset = 0;

        // Get magnetometer data
        SatSensor.getMagData();

        int magXPosOffset = SatSensor.magXRaw;
        int magXNegOffset = SatSensor.magXRaw;
        int magYPosOffset = SatSensor.magYRaw;
        int magYNegOffset = SatSensor.magYRaw;
        
        int calibrationTime = 5000;

        delay(3000);

        Serial1.println("Spin left");

        SatReactionWheel.setMotorSpeed(150);

        unsigned long startTime = millis();

        while (millis() - startTime < calibrationTime){
            
            SatSensor.getMagData();

            if (SatSensor.magXRaw > magXPosOffset){
                magXPosOffset = SatSensor.magXRaw;
            } 
            if (SatSensor.magXRaw < magXNegOffset){
                magXNegOffset = SatSensor.magXRaw;
            }

            if (SatSensor.magYRaw > magYPosOffset){
                magYPosOffset = SatSensor.magYRaw;
            }
            if (SatSensor.magYRaw < magYNegOffset){
                magYNegOffset = SatSensor.magYRaw;
            }

            // Print offsets to serial
            Serial1.println("Offsets: " + String(magXPosOffset) + " " + String(magXNegOffset) + " " + String(magYPosOffset) + " " + String(magYNegOffset));
            delay(100);
        }

        Serial1.println("Stop spin");
        SatReactionWheel.setMotorSpeed(0);
        calibrationMode = false;

        SatSensor.magXOffset = (magXPosOffset + magXNegOffset)/2;
        SatSensor.magYOffset = (magYPosOffset + magYNegOffset)/2;

        Serial1.println("Calibration done");
        Serial1.println("Offsets: " + String(SatSensor.magXOffset) + " " + String(SatSensor.magYOffset));

        SatPID.resetIntergral();
        
        delay(3000);
    }

    void programLoop(){
        SatSensor.updateSensorData();

        Serial.println("test " + String(test1));
        test1 = test1 + 1; 

        float heading = SatSensor.heading;
        float torque; 
        
        if (SatPID.modeOfOperation == 2){
            torque = SatPID.headingHoldLoop(heading);
        } else if (SatPID.modeOfOperation == 1){
            torque = SatPID.detumbleLoop(heading);
        } else {
            torque = 0;
        }

        SatTelemetry.recieveData(commandBuffer);
        commandSelect();
        
        Serial1.println("Heading: " + String(heading) + " torque: " + String(torque));
        SatReactionWheel.setMotorSpeed(torque);
    }

};


SatCommandsSingleton SatCommand;



#endif /* SATADCS_H_ */
