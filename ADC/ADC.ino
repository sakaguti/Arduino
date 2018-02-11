void setup() {
  Serial.begin(115200);
}

void loop() {
  int adc_key_in = analogRead(14);
  Serial.println(adc_key_in);  
}
