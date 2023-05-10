#include "Energia.h"

#line 1 "/home/sew/workspace_v10/Satellitt Design ADCS/Satellitt Design ADCS.ino"
#include <Energia.h>
#include <Wire.h>
#include <SatADCS.h>


void setup();
void loop();

#line 6
int magXOffset = 0;
int magYOffset = 0;



const int calibrationToggle = P2_6;

void setup() {

    Wire.begin();

    Serial1.begin(115200);

    
    SatSensor.initializeAccelGyro();

    
    SatSensor.initializeMag();

    
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

    if (heading < 180){
        SatReactionWheel.changeMotorSpeed(10);
    }   else {
        SatReactionWheel.changeMotorSpeed(-10);
    }

    Serial1.println("CurrentPWM: " + String(SatReactionWheel.getCurrentPWM()));

    
    Serial1.println("Heading: " + String(heading));

    
    





    
    







    delay(100);           
}



