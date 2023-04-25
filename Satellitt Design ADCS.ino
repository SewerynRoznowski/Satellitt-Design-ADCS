// --------------------------------------
// i2c_scanner
//
// Modified from https://playground.arduino.cc/Main/I2cScanner/
// --------------------------------------

#include <Energia.h>
#include <Wire.h>

// Set I2C bus to use: Wire, Wire1, etc.
#define Wire Wire
const int accelAddr = 0x6b;

void setup() {

    Wire.begin();

    Serial.begin(115200);

    // Start IMU
    Wire.beginTransmission(accelAddr);
    Wire.write(0x10);
    Wire.write(0xA0);
    Wire.endTransmission();

}


void loop() {

    // Read WHO AM I? This is just for debug
    Wire.beginTransmission(accelAddr);
    Wire.write(0x0F);
    Wire.endTransmission();

    Wire.requestFrom(accelAddr,1);
    Serial.print(Wire.read());


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
    if (accelX > 32767){
        accelX -= 65536;
    }

    int accelY = (accelYh << 8) | accelYl;
    if (accelX > 32767){
        accelX -= 65536;
    }

    int accelZ = (accelZh << 8) | accelZl;
    if (accelX > 32767){
        accelX -= 65536;
    }


    // Print out values via serial to terminal
    Serial.print("X: ");
    Serial.print(accelX);
    Serial.print(", Y: ");
    Serial.print(accelY);
    Serial.print(", Z: ");
    Serial.println(accelZ);

  delay(100);           // wait for next
}
