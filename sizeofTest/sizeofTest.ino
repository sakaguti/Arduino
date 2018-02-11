void setup() {
  // put your setup code here, to run once:
// draw image

  Serial.begin(115200);
double dataX[]={-0.1,  0.1, -0.1, 0.1, -0.1, 0.1};
double dataY[]={-0.1, -0.1,  0.0, 0.0,  0.1, 0.1};
 Serial.println("sizeof=");
 Serial.println(sizeof(dataX)/sizeof(double));
}

void loop() {
  // put your main code here, to run repeatedly:

}
