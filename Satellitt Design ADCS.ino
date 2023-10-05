#include <Energia.h>
#include <Wire.h>
#include <SatADCS.h>

// Define variables for magnetometer calibration
unsigned long oldTime = 0; 

void setup() {

    Wire.begin();

    Serial.begin(115200);
    Serial1.begin(115200);

    // Start IMU
    SatCommand.SatSensor.initializeAccelGyro();

    // Configure magnetometer
    SatCommand.SatSensor.initializeMag();

}


void loop() {

    unsigned long currentTime = millis();

    if (currentTime - oldTime >= 100) {
        oldTime = currentTime;
        SatCommand.programLoop();
    }

}
