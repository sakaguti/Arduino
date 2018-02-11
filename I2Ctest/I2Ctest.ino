#include<Wire.h>
#define AS5600_AS5601_DEV_ADDRESS      0x36
#define AS5600_AS5601_REG_RAW_ANGLE    0x0C

TwoWire Wire2 = TwoWire();

void setup() {//  UART
    Serial.begin(115200);
}

unsigned long time0, time1, time2;

void loop() {
    time0 = micros();
 //   time0 = millis();
    // Read RAW_ANGLE value from encoder
    // i2C master 1
    Wire.begin(2,0);
    Wire.setClock(400000);    
    Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_AS5601_DEV_ADDRESS, 2);
    uint16_t RawAngle1 = 0;
    RawAngle1  = ((uint16_t)Wire.read() << 8) & 0x0F00;
    RawAngle1 |= (uint16_t)Wire.read();

    // Raw angle value (0 ~ 4095) is stored in RawAngle
    // Read RAW_ANGLE value from encoder
    // I2C master 2
    Wire.begin(5,4);
    Wire.setClock(400000);
    Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_AS5601_DEV_ADDRESS, 2);
    uint16_t RawAngle2 = 0;
    RawAngle2  = ((uint16_t)Wire.read() << 8) & 0x0F00;
    RawAngle2 |= (uint16_t)Wire.read();
    // Raw angle value (0 ~ 4095) is stored in RawAngle
    time2 = micros();
 //  time2=millis();
   Serial.print(" RawAngle1=");
   Serial.print((double)RawAngle1*360.0/4096.0);
   Serial.print(" RawAngle2=");
   Serial.print((double)RawAngle2*360.0/4096.0);
   Serial.print(" time=");
   Serial.print((double)(time2-time0)/1000.0);
 //Serial.print(time2-time0);
   Serial.println("msec");
}
