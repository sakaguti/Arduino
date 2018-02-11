#include <DRV8835MotorShield.h>

/*
 * This example uses the DRV8835MotorShield library to drive each motor with the
 * Pololu DRV8835 Dual Motor Driver Shield for Arduino forward, then backward. 
 * The yellow user LED is on when a motor is set to a positive speed and off when
 * a motor is set to a negative speed.
 */

#define LED_PIN 2
#define MAXS 200

DRV8835MotorShield motors;
// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LASER_PIN            13
#define LASER_CHANNEL        10
uint16_t laser_pwm=0;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //
  motors.flipM1(true);
  motors.flipM2(true);

  // １：ledcSetup(チャンネル, 周波数, PWMの範囲);
  ledcSetup(LASER_CHANNEL, 100000, 16);
  // ２：ledcAttachPin(ピン番号, チャンネル);
  ledcAttachPin(LASER_PIN, LASER_CHANNEL);
  // LASER OFF
  //ledcWrite(LASER_CHANNEL, (int)(1.0/65535.0*100.0));
  //ledcWrite(LASER_CHANNEL, 0);
 
      Serial.begin(115200);

  ledcWrite(LASER_CHANNEL, (int)(65535.0*0.8));
}

void loop()
{
  // run M1 motor with positive speed
    Serial.println("M1");
  //digitalWrite(LED_PIN, HIGH);
  /*
  for (int speed = 0; speed <= MAXS; speed++)
  {
    motors.setM1Speed(speed);
    delay(2);
  }

  for (int speed = MAXS; speed >= 0; speed--)
  {
    motors.setM1Speed(speed);
    delay(2);
  }
*/

   for(int i=0;i<20;i++){
   //motors.setM1Speed(0.6);//right
   motors.setM2Speed(0.6);
    delay(500);        
    //motors.setM2Speed(0);
   //delay(2);
   //motors.setM1Speed(-0.6 );
   //motors.setM2Speed(-0.6);// WColi 前傾斜 50msec max
    delay(500);
   }
   
   /*
   for(int i=0;i<30;i+=3){
   motors.setM1Speed(i);
    delay(100);
   motors.setM1Speed(-i);
    delay(100);
    Serial.println(i);
   }
   
      for(int i=30;i>0;i-=3){
   motors.setM1Speed(i);
    delay(100);
   motors.setM1Speed(-i);
    delay(100);
    Serial.println(i);
   }
*/
/*
  // motors.setM1Speed(0);
       motors.setM1Speed(-100);
      delay(300);
Serial.println("break");
    // break
    for(int i=0;i<500;i++){
      motors.setM1Speed(100);
      delay(1);
      motors.setM1Speed(-100);
      delay(1);
    }
*/
  /*
  delay(1000);
  // run M1 motor with negative speed

  digitalWrite(LED_PIN, LOW);
  
  for (int speed = 0; speed >= -MAXS; speed--)
  {
    motors.setM1Speed(speed);
    delay(2);
  }
  
  for (int speed = -MAXS; speed <= 0; speed++)
  {
    motors.setM1Speed(speed);
    delay(2);
  }
  delay(1000);
  // run M2 motor with positive speed
  
  Serial.println("M2");

  digitalWrite(LED_PIN, HIGH);
  
  for (int speed = 0; speed <= MAXS; speed++)
  {
    motors.setM2Speed(speed);
    delay(2);
  }

  for (int speed = MAXS; speed >= 0; speed--)
  {
    motors.setM2Speed(speed);
    delay(2);
  }
  delay(1000);
  // run M2 motor with negative speed
  
  digitalWrite(LED_PIN, LOW);
  
  for (int speed = 0; speed >= -MAXS; speed--)
  {
    motors.setM2Speed(speed);
    delay(2);
  }
  
  for (int speed = -MAXS; speed <= 0; speed++)
  {
    motors.setM2Speed(speed);
    delay(2);
  }
  delay(1000);

Serial.println("brakeing");
    motors.breakeM1();
    motors.breakeM2();
    delay(1000);
Serial.println("run");
*/
}
