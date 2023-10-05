#include "Energia.h"

#line 1 "/home/sew/workspace_v10/Satellitt Design ADCS/Satellitt Design ADCS.ino"
#include <Energia.h>
#include <Wire.h>
#include <SatADCS.h>


void setup();
void loop();

#line 6
unsigned long oldTime = 0; 

void setup() {

    Wire.begin();

    Serial.begin(115200);
    Serial1.begin(115200);

    
    SatCommand.SatSensor.initializeAccelGyro();

    
    SatCommand.SatSensor.initializeMag();

}


void loop() {

    unsigned long currentTime = millis();

    if (currentTime - oldTime >= 100) {
        oldTime = currentTime;
        SatCommand.programLoop();
    }

}



