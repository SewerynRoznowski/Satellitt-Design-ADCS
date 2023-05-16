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

    
    SatCommand.SatSensor.initializeAccelGyro();

    
    SatCommand.SatSensor.initializeMag();

    
    pinMode(calibrationToggle, INPUT_PULLUP);

}


void loop() {
    
    SatCommand.SatTelemetry.recieveData(SatCommand.commandBuffer);
    SatCommand.commandSelect();
    
    
    
    int magX, magY, magZ;
    SatCommand.SatSensor.getMagData(magX, magY, magZ);

    magX = magX - SatCommand.SatSensor.magXOffset;
    magY = magY - SatCommand.SatSensor.magYOffset;

    
    float heading = atan2(-magY, magX);

    
    heading = heading * 180/PI;

    
    if(heading < 0)
    {
        heading = 360 + heading;
    }

    float torque = 0; 
    
    if (SatCommand.modeofOperation == 2){
        torque = SatCommand.SatPID.headingHoldLoop(heading);
    } else if (SatCommand.modeofOperation == 1){
        torque = SatCommand.SatPID.detumbleLoop(heading);
    } else {
        torque = 0;
    }

    

    SatCommand.SatReactionWheel.setMotorSpeed(torque);

    

    
    

    
    





    
    







    delay(100);           
}



