int sp = 0;
int delta = 1;


// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     5000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            2

int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by


// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);
  // write duty to LEDC
  ledcWrite(channel, duty);
}

void setup() {

  Serial.begin(115200);
  Serial.println("start");


  // １：ledcSetup(チャンネル, 周波数, PWMの範囲);
  ledcSetup(1, 5000, 13);
  //ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    // ２：ledcAttachPin(ピン番号, チャンネル);
  ledcAttachPin(2, 1);
  //ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
}


void loop() {
  // put your main code here, to run repeatedly:
  //ledcAnalogWrite(1, brightness); 
  //ledcAnalogWrite(LEDC_CHANNEL_0, brightness);
  ledcAnalogWrite(1, brightness);
  
  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }
  
   Serial.println(brightness);
   delay(20);
}
