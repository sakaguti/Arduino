
extern "C" {
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
}

#include "esp32-hal-i2c.h"
#include "Wire.h"
#include "Arduino.h"
#include <SPI.h>

// SPI CS pins
#define S1 5
#define S2 15
#define S3 25

#define AS5600_AS5601_DEV_ADDRESS      0x36
#define AS5600_AS5601_REG_RAW_ANGLE    0x0C

double  c= (360.0/16383.0);
unsigned char values[2];

unsigned long time0, time1, time2;

double xAngle,xSpeed=0.0;
double yAngle,ySpeed=0.0;

double xSetAngle=235.0, xSetAngleDt=1.0, xSetAngleOld=0.0;
double ySetAngle=90.0;

double xSetAxis=0.0;
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

      // SPI　(vSPI)
  SPI.begin();//int8_t sck, int8_t miso, int8_t mosi, int8_t ss
  SPI.setFrequency(1000000); //SSD1331 のSPI Clock Cycle Time 50nsec
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
}

double GetEncPos(int ch){
  double angle=0.0, x=0.0;
  //  y axis 
  digitalWrite(ch, LOW);
  delayMicroseconds(10);
  for(int16_t i=0; i<2; i++) {
   values[i] = SPI.transfer(0x00); // MISIは使わないので、引数は任意
  }
  digitalWrite(ch, HIGH);
  x = ((values[0]<<8) + values[1]) & 0x3fff;
    angle=(double)x*c;
  if(angle > 180.0 ) angle -= 360.0;
 
  return angle;
}

void loop() {
   time0 = micros();
    xAngle=GetEncPos(S1);
    yAngle=GetEncPos(S2);
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
