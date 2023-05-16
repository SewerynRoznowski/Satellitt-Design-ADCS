#include <Energia.h>
#include <Wire.h>
#include <SatADCS.h>

// Define variables for magnetometer calibration
int magXOffset = 0;
int magYOffset = 0;


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
    
    SatCommand.SatTelemetry.recieveData(SatCommand.commandBuffer);
    SatCommand.commandSelect();
    
    // Get magnetometer data
    
    int magX, magY, magZ;
    SatCommand.SatSensor.getMagData(magX, magY, magZ);

    magX = magX - SatCommand.SatSensor.magXOffset;
    magY = magY - SatCommand.SatSensor.magYOffset;

    // Calculate heading from magnetometer data
    float heading = atan2(-magY, magX);

    // Convert to degrees 
    heading = heading * 180/PI;

    // Correct for when signs are reversed.
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

    //Serial1.println(SatCommand.modeofOperation);

    SatCommand.SatReactionWheel.setMotorSpeed(torque);

    //Serial1.println("CurrentPWM: " + String(SatReactionWheel.getCurrentPWM()));

    // Print heading
    //Serial1.println("Heading: " + String(heading) + " Torque: " + String(torque));

    // Get accelerometer data
    /*int accelX, accelY, accelZ;
    SatSensor.getAccelData(accelX, accelY, accelZ);

    // Print accelerometer data
    Serial1.println("Accel: " + String(accelX) + " " + String(accelY) + " " + String(accelZ));*/

    // Get gyroscope data
    /*int gyroX, gyroY, gyroZ;
    SatSensor.getGyroData(gyroX, gyroY, gyroZ);

    // Print gyroscope data
    Serial1.println("Gyro: " + String(gyroX) + " " + String(gyroY) + " " + String(gyroZ));*/



    delay(100);           // wait for next
}
