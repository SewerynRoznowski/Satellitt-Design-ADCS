/*
 * SatI2c.h
 *
 *  Created on: Apr 20, 2023
 *      Author: sew
 */
#include <Wire.h>
#ifndef SATI2C_H_
#define SATI2C_H_


class SatI2c {
    public: 
    int* i2cReadfromMem (int slaveAddress, int registerAddress, int messageLength) {
        
        int message[messageLength];

        Wire.beginTransmission(slaveAddress);
        Wire.write(registerAddress);
        Wire.endTransmission();
    
        Wire.requestFrom(slaveAddress, messageLength);
        int i = 0;

        while (Wire.available()) {
            message[i] = Wire.read();
            i++;
        }
        return message;
    }
};


SatI2c SatI2cObj;



#endif /* SATI2C_H_ */
