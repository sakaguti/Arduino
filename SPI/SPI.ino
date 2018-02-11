/*
 * VSPI
 * SCLK #18
 * MOSI #23
 * MISO #19
 * CS(SS) #5
 * 
 * AS5048A.py
 * 
 */
#include <SPI.h>
 
#define S1 5 //デフォルトから設定を変更して再定義(IO15 -> IO5)
#define S2 15 //デフォルトから設定を変更して再定義(IO15 -> IO5)

unsigned char values[2];
int16_t x,y;
int16_t bx,by;
 
void setup() {
  SPI.begin();//int8_t sck, int8_t miso, int8_t mosi, int8_t ss
  SPI.setFrequency(1000000); //SSD1331 のSPI Clock Cycle Time 50nsec
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  Serial.begin(115200);
}
 
void loop() {
  delay(500);
  digitalWrite(S1, LOW);
  delayMicroseconds(10);
  for(int16_t i=0; i<2; i++) {
   values[i] = SPI.transfer(0x00);
   delayMicroseconds(3);
  }
  x = ((values[0]<<8) + values[1]) & 0x3fff;
  Serial.print(" x=");
  Serial.print(x);
  digitalWrite(S1, HIGH);
  
  digitalWrite(S2, LOW);
  delayMicroseconds(10);
  for(int16_t i=0; i<2; i++) {
   values[i] = SPI.transfer(0x00);
   delayMicroseconds(3);
  }
  y = ((values[0]<<8) + values[1]) & 0x3fff;
  Serial.print(" y=");
  Serial.println(y);
  digitalWrite(S2, HIGH);
}

