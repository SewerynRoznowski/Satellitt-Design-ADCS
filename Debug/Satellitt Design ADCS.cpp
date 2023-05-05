#include "Energia.h"

#line 1 "/home/sew/workspace_v10/Satellitt Design ADCS/Satellitt Design ADCS.ino"
#include <Energia.h>
#include <Wire.h>
#include <SatI2c.h>


void setup();
void loop();

#line 6
int magXOffset = 0;
int magYOffset = 0;



const int motorPWM = P1_4;
const int motorEnA = P3_2;
const int motorEnB = P2_7;
const int calibrationToggle = P2_6;

void setup() {

    Wire.begin();

    Serial1.begin(115200);

    
    Wire.beginTransmission(accelAddr);
    Wire.write(0x10);
    Wire.write(0xA0);
    Wire.endTransmission();

    
    
    pinMode(motorPWM, OUTPUT);
    pinMode(motorEnA, OUTPUT);
    pinMode(motorEnB, OUTPUT);

    
    digitalWrite(motorPWM, LOW);
    digitalWrite(motorEnA, HIGH);
    digitalWrite(motorEnB, LOW);

    
    Wire.beginTransmission(magAddr);
    Wire.write(0x0A);
    Wire.write(0x00);
    Wire.write(0xBD);
    Wire.endTransmission();

    
    pinMode(calibrationToggle, INPUT_PULLUP);

}


void loop() {
    
    if (digitalRead(calibrationToggle) == LOW){
        
        
        Serial1.println("Calibrating magnetometer");
        delay(3000);
        
        
        magXOffset = 0;
        magYOffset = 0;

        
        int magXc, magYc, magZc;
        SatSensor.getMagData(magXc, magYc, magZc);

        int magXPosOffset = magXc;
        int magXNegOffset = magXc;
        int magYPosOffset = magYc;
        int magYNegOffset = magYc;
        
        delay(100);

        while (digitalRead(calibrationToggle) == LOW){
            
            int magXc, magYc, magZc;

            SatSensor.getMagData(magXc, magYc, magZc);

            if (magXc > magXPosOffset){
                magXPosOffset = magXc;
            } 
            if (magXc < magXNegOffset){
                magXNegOffset = magXc;
            }

            if (magYc > magYPosOffset){
                magYPosOffset = magYc;
            }
            if (magYc < magYNegOffset){
                magYNegOffset = magYc;
            }
            
            Serial1.println("Offsets: " + String(magXPosOffset) + " " + String(magXNegOffset) + " " + String(magYPosOffset) + " " + String(magYNegOffset));

            delay(100);
        }
        magXOffset = (magXPosOffset + magXNegOffset)/2;
        magYOffset = (magYPosOffset + magYNegOffset)/2;

        Serial1.println("Calibration done");
        Serial1.println("Offsets: " + String(magXOffset) + " " + String(magYOffset));

        delay(3000);
    }
    
    
    
    int magX, magY, magZ;
    SatSensor.getMagData(magX, magY, magZ);

    magX = magX - magXOffset;
    magY = magY - magYOffset;

    
    float heading = atan2(-magY, magX);

    
    heading = heading * 180/PI;

    
    if(heading < 0)
    {
        heading = 360 + heading;
    }

    
    Serial1.println("Heading: " + String(heading));

    
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


    
    int accelX = (accelXh << 8) | accelXl;

    int accelY = (accelYh << 8) | accelYl;

    int accelZ = (accelZh << 8) | accelZl;  

    delay(5000);

    digitalWrite(motorEnA, HIGH);
    digitalWrite(motorEnB, LOW);
    analogWrite(motorPWM, 220);

    delay(5000);

    digitalWrite(motorEnA, LOW);
    digitalWrite(motorEnB, LOW);
    analogWrite(motorPWM, 0);

    delay(5000);

    digitalWrite(motorEnA, LOW);
    digitalWrite(motorEnB, HIGH);
    analogWrite(motorPWM, 220);

    delay(5000);

    digitalWrite(motorEnA, LOW);
    digitalWrite(motorEnB, LOW);
    analogWrite(motorPWM, 0);




    delay(100);           
}



