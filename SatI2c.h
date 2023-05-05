/*
 * SatI2c.h
 *
 *  Created on: Apr 20, 2023
 *      Author: sew
 */
#include <Wire.h>
#ifndef SATI2C_H_
#define SATI2C_H_

const int accelAddr = 0x6b;
const int magAddr = 0x30;

class SatSensorSingelton {
    public: 
    // Returns the raw magnetometer data, pass by reference
    void getMagData (int& magX , int& magY , int& magZ) {
        
        // Request magnetometer to register 0x00
        Wire.beginTransmission(magAddr);
        Wire.write(0x00);
        Wire.endTransmission();
        
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
};

SatSensorSingelton SatSensor;



#endif /* SATI2C_H_ */
