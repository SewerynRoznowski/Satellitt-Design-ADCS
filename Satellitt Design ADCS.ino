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
    SatSensor.initializeAccelGyro();

    // Configure magnetometer
    SatSensor.initializeMag();

    // Configure calibration switch
    pinMode(calibrationToggle, INPUT_PULLUP);

}


void loop() {
    


    SatTelemetry.recieveData();
    SatCommands.commandSelect();
    
    if (calibrationMode == true){
        // 
        Serial1.println("Calibrating magnetometer");       
        Serial1.println("Resetting offsets");

        // Reset offsets
        magXOffset = 0;
        magYOffset = 0;

        // Get magnetometer data
        int magXc, magYc, magZc;
        SatSensor.getMagData(magXc, magYc, magZc);

        int magXPosOffset = magXc;
        int magXNegOffset = magXc;
        int magYPosOffset = magYc;
        int magYNegOffset = magYc;
        
        int calibrationTime = 5000;

        delay(3000);

        Serial1.println("Spin left");

        SatReactionWheel.setMotorSpeed(150);

        unsigned long startTime = millis();

        while (millis() - startTime < calibrationTime){
            
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
            // Print offsets to serial
            Serial1.println("Offsets: " + String(magXPosOffset) + " " + String(magXNegOffset) + " " + String(magYPosOffset) + " " + String(magYNegOffset));

            delay(100);
        }

        Serial1.println("Stop spin");
        SatReactionWheel.setMotorSpeed(0);
        calibrationMode = false;

        magXOffset = (magXPosOffset + magXNegOffset)/2;
        magYOffset = (magYPosOffset + magYNegOffset)/2;

        Serial1.println("Calibration done");
        Serial1.println("Offsets: " + String(magXOffset) + " " + String(magYOffset));

        delay(3000);
    }
    
    // Get magnetometer data
    
    int magX, magY, magZ;
    SatSensor.getMagData(magX, magY, magZ);

    magX = magX - magXOffset;
    magY = magY - magYOffset;

    // Calculate heading from magnetometer data
    float heading = atan2(-magY, magX);

    // Convert to degrees 
    heading = heading * 180/PI;

    // Correct for when signs are reversed.
    if(heading < 0)
    {
        heading = 360 + heading;
    }

    //Serial1.println("CurrentPWM: " + String(SatReactionWheel.getCurrentPWM()));

    // Print heading
    Serial1.println("Heading: " + String(heading));

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
