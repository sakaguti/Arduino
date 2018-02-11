const unsigned char _M1DIR = 34;
const unsigned char _M2DIR = 35;
const unsigned char _M1PWM = 32;
const unsigned char _M2PWM = 33;
//const unsigned char _M2PWM = 2;

void setup() {
  digitalWrite(_M1DIR, LOW);
  pinMode(_M1DIR, OUTPUT);
  digitalWrite(_M1DIR, LOW);
    
  digitalWrite(_M2DIR, LOW);
  pinMode(_M2DIR, OUTPUT);
  digitalWrite(_M2DIR, LOW);
    
  // １：ledcSetup(チャンネル, 周波数, PWMの範囲);
  ledcSetup(1, 500, 13);
  ledcSetup(2, 500, 13);

  // ２：ledcAttachPin(ピン番号, チャンネル);
  ledcAttachPin(_M1PWM, 1);
  ledcAttachPin(_M2PWM, 2);
}

void loop() {
   boolean reverse = 0;

  for(int speed = -5;speed <= 0; speed++){
  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1;    // preserve the direction
  } else reverse=0;
  if (speed > 400)  // max 
    speed = 400;
    
   //ledcWrite(1, speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
   uint32_t duty = (8191 / 255) * min(speed, 255);
   // write duty to LEDC
   ledcWrite(2, duty);
    
  if (reverse) // flip if speed was negative or _flipM1 setting is active, but not both
    digitalWrite(_M2DIR, HIGH);
  else
    digitalWrite(_M2DIR, LOW);
  }
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  //digitalWrite(_M1DIR, LOW);
  delay(1000);
}
