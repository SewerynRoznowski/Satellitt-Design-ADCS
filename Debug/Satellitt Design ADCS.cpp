#include "Energia.h"

#line 1 "/home/sew/workspace_v10/Satellitt Design ADCS/Satellitt Design ADCS.ino"






#include <Energia.h>
#include <Wire.h>


#define Wire Wire
void setup();
void loop();

#line 12
const int accelAddr = 0x6b;

void setup() {

    Wire.begin();

    Serial.begin(115200);

    Wire.beginTransmission(accelAddr);
    Wire.write(0x10);
    Wire.write(0xA0);
    Wire.endTransmission();

}


void loop() {

    Wire.beginTransmission(accelAddr);
    Wire.write(0x0F);
    Wire.endTransmission();

    Wire.requestFrom(accelAddr,1);
    Serial.print(Wire.read());


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

    Serial.print("X: ");
    Serial.print(accelX);
    Serial.print(", Y: ");
    Serial.print(accelY);
    Serial.print(", Z: ");
    Serial.println(accelZ);

  delay(100);           
}



