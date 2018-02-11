#include "DRV8835MotorShield.h"

// do not use 34,35 pin
const unsigned char DRV8835MotorShield::_M1DIR = 26;
const unsigned char DRV8835MotorShield::_M2DIR = 25;
const unsigned char DRV8835MotorShield::_M1PWM = 32;
const unsigned char DRV8835MotorShield::_M2PWM = 33;
boolean DRV8835MotorShield::_flipM1 = false;
boolean DRV8835MotorShield::_flipM2 = false;
// Motor 100Hz Coil 100000Hz
const unsigned int Freq = 100000; // 100000

#define maxVolt 4.5

void DRV8835MotorShield::initPinsAndMaybeTimer()
{
  // Initialize the pin states used by the motor driver shield
  // digitalWrite is called before and after setting pinMode.
  // It called before pinMode to handle the case where the board
  // is using an ATmega AVR to avoid ever driving the pin high, 
  // even for a short time.
  // It is called after pinMode to handle the case where the board
  // is based on the Atmel SAM3X8E ARM Cortex-M3 CPU, like the Arduino
  // Due. This is necessary because when pinMode is called for the Due
  // it sets the output to high (or 3.3V) regardless of previous
  // digitalWrite calls.

    
  // １：ledcSetup(チャンネル, 周波数, PWMの範囲);
  ledcSetup(1, Freq, 16);
  ledcSetup(2, Freq, 16);
  ledcSetup(3, Freq, 16);
  ledcSetup(4, Freq, 16);
  // ２：ledcAttachPin(ピン番号, チャンネル);
  ledcAttachPin(_M1PWM, 1);
  ledcAttachPin(_M2PWM, 2);
  ledcAttachPin(_M1DIR, 3);
  ledcAttachPin(_M2DIR, 4);
    
}

// speed should be a number between -400 and 400
void DRV8835MotorShield::setM1Speed(double speed)
{
  init(); // initialize if necessary
    
  boolean reverse = 0;
  
  if (speed < 0.0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  }
    if (speed > 1.0)  // max
        speed = 1.0;
    
   //ledcWrite(1, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
   //ledcAnalogWrite(1, speed * 51 / 80);
   uint32_t duty = (int)(speed*65535.0/5.0*(double)maxVolt);
   // write duty to LEDC
   
    
    if (reverse){ // flip if speed was negative or _flipM1 setting is active, but not both
        ledcWrite(1, 0);
        ledcWrite(3, duty);
    } else {
        ledcWrite(1, duty);
        ledcWrite(3, 0);
    }
}

// speed should be a number between -400 and 400
void DRV8835MotorShield::setM2Speed(double speed)
{
  init(); // initialize if necessary
    
  boolean reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // make speed a positive quantity
    reverse = 1;  // preserve the direction
  }
    if (speed > 1.0)  // max
        speed = 1.0;
    
   //ledcWrite(2, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
   //ledcAnalogWrite(2, speed * 51 / 80);
   uint32_t duty = (int)(speed*65535.0/5.0*(double)maxVolt);
   // write duty to LEDC
    if (reverse){ // flip if speed was negative or _flipM1 setting is active, but not both
        ledcWrite(2, 0);
        ledcWrite(4, duty);
    } else {
        ledcWrite(2, duty);
        ledcWrite(4, 0);
    }
}

// set speed for both motors
// speed should be a number between -400 and 400
void DRV8835MotorShield::setSpeeds(double m1Speed, double m2Speed){
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}


void DRV8835MotorShield::breakeM1(){
    ledcWrite(1, 65535/5*maxVolt);
    ledcWrite(3, 65535/5*maxVolt);
}

void DRV8835MotorShield::breakeM2(){
    ledcWrite(2, 65535/5*maxVolt);
    ledcWrite(4, 65535/5*maxVolt);
}
void DRV8835MotorShield::breakes(){
    breakeM1();
    breakeM2();
}
void DRV8835MotorShield::flipM1(boolean flip)
{
  _flipM1 = flip;
}

void DRV8835MotorShield::flipM2(boolean flip)
{
  _flipM2 = flip;
}
