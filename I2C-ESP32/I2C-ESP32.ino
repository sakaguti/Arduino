
extern "C" {
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
}

#include "esp32-hal-i2c.h"
#include "Wire.h"
#include "Arduino.h"

#define AS5600_AS5601_DEV_ADDRESS      0x36
#define AS5600_AS5601_REG_RAW_ANGLE    0x0C

unsigned long time0, time1, time2;

double xAngle,xSpeed=0.0;
double yAngle,ySpeed=0.0;

double xSetAngle=235.0, xSetAngleDt=1.0, xSetAngleOld=0.0;
double ySetAngle=90.0;

double xSetAxis=4.0;
double ySetAxis=0.0;

TwoWire Wire1 = TwoWire(1);
TwoWire Wire0 = TwoWire(0);

void setup() {  
  
  Serial.begin (115200);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
    // Read RAW_ANGLE value from encoder
    Wire0.begin(27,14,400000);
    Wire1.begin(22,23,400000);    
}

double GetEncPosX(){
  double angle=0.0;
    Wire0.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire0.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire0.endTransmission(false);
    Wire0.requestFrom(AS5600_AS5601_DEV_ADDRESS, 2);
    uint16_t RawAngle = 0;
    RawAngle  = ((uint16_t)Wire0.read() << 8) & 0x0F00;
    RawAngle |= (uint16_t)Wire0.read();
    if( RawAngle > 4096 ){
      Wire0.reset();
      return GetEncPosX();
    }
    angle=(double)RawAngle*360.0/4096.0;
    if(angle > 180.0 ) angle -= 360.0;
  //Wire0.reset();
  return angle;
}

double GetEncPosY(){
  double angle=0.0;
    Wire1.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire1.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire1.endTransmission(false);
    Wire1.requestFrom(AS5600_AS5601_DEV_ADDRESS, 2);
    uint16_t RawAngle = 0;
    RawAngle  = ((uint16_t)Wire1.read() << 8) & 0x0F00;
    RawAngle |= (uint16_t)Wire1.read();
     if( RawAngle > 4096 ){
      Wire1.reset();
      return GetEncPosY();
    }
    angle=(double)RawAngle*360.0/4096.0;
    if(angle > 180.0 ) angle -= 360.0;
  //Wire1.reset();
  return angle;
}


void loop() {
   time0 = micros();
    xAngle=GetEncPosX();
    yAngle=GetEncPosY();
        // Raw angle value (0 ~ 4095) is stored in RawAngle
    time2 = micros();
 //  time2=millis();
   Serial.print(" xAngle=");
   Serial.print(xAngle);
   Serial.print(" yAngle=");
   Serial.print(yAngle);
   Serial.print(" time=");
   Serial.print((double)(time2-time0)/1000.0);
   Serial.println("msec"); 
   delay(100);
}
