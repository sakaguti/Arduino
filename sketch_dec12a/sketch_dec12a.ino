#define LED_PIN1 9
#define LED_PIN2 10
 
// 初期化
void setup(){
 
  // LEDのピンを出力に
  pinMode(LED_PIN1, OUTPUT); 
  pinMode(LED_PIN2, OUTPUT); 
 
}
 
// メインループ
void loop() {
 
  // LED1を最大値で点灯
 analogWrite(LED_PIN1, 255);
 
  // LED2を減光して点灯
 analogWrite(LED_PIN2, 10);
 
}
