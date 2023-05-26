#include <Energia.h>
#include <Wire.h>
#include <SatADCS.h>

// Define variables for magnetometer calibration
unsigned long oldTime = 0; 

// Define pins for motor controller
const int calibrationToggle = P2_6;

void setup() {

    Wire.begin();

    Serial1.begin(115200);

    // Start IMU
    SatCommand.SatSensor.initializeAccelGyro();

    // Configure magnetometer
    SatCommand.SatSensor.initializeMag();

    // Configure calibration switch
    pinMode(calibrationToggle, INPUT_PULLUP);

}


void loop() {

    unsigned long currentTime = millis();

    if (currentTime - oldTime >= 100) {
        oldTime = currentTime;
        SatCommand.programLoop();
    }

}
