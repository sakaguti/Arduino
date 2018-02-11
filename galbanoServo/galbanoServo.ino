/*
 * DRV8835 端子
 * M2PWM    Pin  35
 * M1PWM    Pin  34
 * M2DIR    Pin  33
 * M1DIR    Pin  32
 * 
 * VSPI 端子
 * GPIO #18  —– SCLK ( SPI Clock ) // 
 * GPIO #23  —– MOSI ( Master Output Slave Input ) 
 * GPIO #19  —– MISO ( Master Input Slave Output ) 
 * GPIO #05  —– CS1 ( Chip Select )
 * GPIO #15  —– CS2 ( Chip Select )
 * 
 */
#include <AutoPID.h>
#include <SPI.h>

#include <DRV8835MotorShield.h>
DRV8835MotorShield motors;
 
#define S1 5
#define S2 15

// Delay msec for motor rotate time
#define xDelay 20
#define yDelay 20

unsigned char values[2];
int16_t x,y;
float  c= (360.0/16383.0);
int speed=0;

#define OUTPUT_MIN -400
#define OUTPUT_MAX  400
#define KP 0.12
#define KI 0.0004
#define KD 0.00005

double xAngle, xSetPoint, xOutputVal;
double yAngle, ySetPoint, yOutputVal;

unsigned long time0, time1, time2;
//input/output variables passed by reference, so they are updated automatically
AutoPID xPID(&xAngle, &xSetPoint, &xOutputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
//input/output variables passed by reference, so they are updated automatically
AutoPID yPID(&yAngle, &ySetPoint, &yOutputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

void setup() {
  // SPI
  SPI.begin();//int8_t sck, int8_t miso, int8_t mosi, int8_t ss
  SPI.setFrequency(1000000); //SSD1331 のSPI Clock Cycle Time 50nsec
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  Serial.begin(115200);
  // PID
  //if angle is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  xPID.setBangBang(16.3);
  //set PID update interval to 10ms
  xPID.setTimeStep(xDelay);
  //
  //if angle is more than 4 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  yPID.setBangBang(16.3);
  //set PID update interval to 10ms
  yPID.setTimeStep(yDelay);
}


// range 0- 16383  0-360deg  0021974 deg/unit

void loop() {
// get xSetPoint, ySetPoint
  xSetPoint=90.0;
  ySetPoint=90.0;
  
/*  y axis  */  
  time0 = micros();
  //delay(250);
  digitalWrite(S1, LOW);
  delayMicroseconds(10);
  for(int16_t i=0; i<2; i++) {
   values[i] = SPI.transfer(0x00); // MISIは使わないので、引数は任意
  }
  digitalWrite(S1, HIGH);
  x = ((values[0]<<8) + values[1]) & 0x3fff;
  xAngle=float(x)*c;
//
  xPID.run();
//
  speed=xOutputVal;
//
  motors.setM1Speed(speed);
  delay(xDelay);
  
  Serial.print(" x=");
  Serial.print(xAngle);
  Serial.print(" xs=");
  Serial.print(speed);
  time1 = micros();

/*  y axis  */  
  digitalWrite(S2, LOW);
  delayMicroseconds(10);
  for(int16_t i=0; i<2; i++) {
   values[i] = SPI.transfer(0x00);
  }
  digitalWrite(S2, HIGH);
  y = ((values[0]<<8) + values[1]) & 0x3fff;
  yAngle=float(y)*c;
//
  yPID.run();
//
  speed=yOutputVal;
//
  motors.setM2Speed(speed);
  delay(yDelay);

  Serial.print(" y=");
  Serial.print(yAngle);
  Serial.print(" ys=");
  Serial.print(speed);
  
  time2 = micros();

  Serial.print(" time=");
  Serial.print(float(time2-time0)/1000.0);
  Serial.println("msec");
}

