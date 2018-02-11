/*
 * VSPI
 * SCLK #18
 * MOSI #23
 * MISO #19
 * CS(SS) #5
 * 
 * AS5048A.py
 *  360deg  16383  0.022 deg/unit  -> +/- 0.022  error 0.05deg
 * 
 */
#include <SPI.h>
 
#define S1 5
#define S2 15
unsigned char values[2];
int16_t x,y;
float  c= (360.0/16383.0);
 
void setup() {
  SPI.begin();//int8_t sck, int8_t miso, int8_t mosi, int8_t ss
  SPI.setFrequency(1000000); //SSD1331 のSPI Clock Cycle Time 50nsec
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  Serial.begin(115200);
}

// range 0- 16383  0-360deg  0021974 deg/unit

void loop() {
  //delay(250);
  digitalWrite(S1, LOW);
  delayMicroseconds(10);
  for(int16_t i=0; i<2; i++) {
   values[i] = SPI.transfer(0x00); // MISIは使わないので、引数は任意
  }
  x = ((values[0]<<8) + values[1]) & 0x3fff;
  Serial.print(" x=");
  Serial.print(float(x)*c);
  digitalWrite(S1, HIGH);
  
  digitalWrite(S2, LOW);
  delayMicroseconds(10);
  for(int16_t i=0; i<2; i++) {
   values[i] = SPI.transfer(0x00);
  }
  y = ((values[0]<<8) + values[1]) & 0x3fff;
  Serial.print(" y=");
  Serial.println(float(y)*c);
  digitalWrite(S2, HIGH);
}

