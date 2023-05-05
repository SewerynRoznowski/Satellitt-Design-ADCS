#include <Energia.h>
#include <Wire.h>
#include <SatI2c.h>

// Define variables for magnetometer calibration
int magXOffset = 0;
int magYOffset = 0;


// Define pins for motor controller
const int motorPWM = P1_4;
const int motorEnA = P3_2;
const int motorEnB = P2_7;
const int calibrationToggle = P2_6;

void setup() {

    Wire.begin();

    Serial1.begin(115200);

    // Start IMU
    Wire.beginTransmission(accelAddr);
    Wire.write(0x10);
    Wire.write(0xA0);
    Wire.endTransmission();

    // Configure motor controller
    // Set motor pins to output
    pinMode(motorPWM, OUTPUT);
    pinMode(motorEnA, OUTPUT);
    pinMode(motorEnB, OUTPUT);

    // Set motor enable pins to LOW
    digitalWrite(motorPWM, LOW);
    digitalWrite(motorEnA, HIGH);
    digitalWrite(motorEnB, LOW);

    // Configure magnetometer
    Wire.beginTransmission(magAddr);
    Wire.write(0x0A);
    Wire.write(0x00);
    Wire.write(0xBD);
    Wire.endTransmission();

    // Configure calibration switch
    pinMode(calibrationToggle, INPUT_PULLUP);

}


void loop() {
    
    if (digitalRead(calibrationToggle) == LOW){
        
        // 
        Serial1.println("Calibrating magnetometer");
        delay(3000);
        
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
            // Print offsets to serial
            Serial1.println("Offsets: " + String(magXPosOffset) + " " + String(magXNegOffset) + " " + String(magYPosOffset) + " " + String(magYNegOffset));

            delay(100);
        }
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

    // Print heading
    Serial1.println("Heading: " + String(heading));

    // Get accelerometer data
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


    // Merge together registers
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




    delay(100);           // wait for next
}
