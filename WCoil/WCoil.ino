/*
 * PWM_SDK_SAMPLE
 *  	The PWM of Arduino Esp8266 have issues about "flickering"
 * 		This solution uses the PWM of Expressif SDK and
 * 		a new solution of PWM - see https://github.com/StefanBruens/ESP8266_new_pwm
 *
 * 		Attention!  - To use this new solution You have put the file "pwm.c" together at your ".ino" file
 * 					- You can use 8 PWM, but not in GPIO16 (D0)
 * 					- After changes in PWM, you must call "pwm_start" to commit this changes
 * 					- This PWM uses the period to set dutys: 100% -> 5000, 10% -> 500 (for default period)
 *
 * 		This sample dimmer the leds in D5 and D6
 */

// Include pf Arduino

#include "Arduino.h"

// Includes of Expressif SDK

extern "C"{
	#include "pwm.h"
	#include "user_interface.h"
}

////// PWM

// Period of PWM frequency -> default of SDK: 5000 -> * 200ns ^= 1 kHz

#define PWM_PERIOD 5000

// PWM channels

#define PWM_CHANNELS 4
// Delay msec for motor rotate time
#define Delay 0 // 10 - 20msec

// PWM setup (choice all pins that you use PWM)

#include<Wire.h>
#define AS5600_AS5601_DEV_ADDRESS      0x36
#define AS5600_AS5601_REG_RAW_ANGLE    0x0C

unsigned long time0, time1, time2;
boolean sSW=true;  // servo SW
boolean dSW=false; // whether image draw or not.

#define MAXDRAW 5 // How many draw image repeatable
int patternNo=0; // draw image pattern.
int drawCount=0; // How many draw image
int imgCount=0;
#define WNO  4000
#define WNO2 250

#include <SoftwareSerial.h>                     // to use pin 4 and 5 as RX and TX ports

#define sw_serial_rx_pin 4 //  Connect this pin to TX on the esp8266
#define sw_serial_tx_pin 5 //  Connect this pin to RX on the esp8266

uint32 io_info[PWM_CHANNELS][3] = {
	// MUX, FUNC, PIN
//	{PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5,   5}, // D1
//	{PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4,   4}, // D2
//	{PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0,   0}, // D3
//	{PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2,   2}, // D4
	{PERIPHS_IO_MUX_MTMS_U,  FUNC_GPIO14, 14}, // D5
	{PERIPHS_IO_MUX_MTDI_U,  FUNC_GPIO12, 12}, // D6
	{PERIPHS_IO_MUX_MTCK_U,  FUNC_GPIO13, 13}, // D7
	{PERIPHS_IO_MUX_MTDO_U,  FUNC_GPIO15 ,15}, // D8
											   // D0 - not have PWM :-(
};

// PWM initial duty: all off

uint32 pwm_duty_init[PWM_CHANNELS];

// Dimmer variables

int16_t duty = 0;
int16_t step = 1;
double maxVolt=4.0;

//

#include <WiFi.h>
#include <WiFiUdp.h>

char ssid[] = "sakaguti-network3";      //  your network SSID (name)
char pass[] = "sakaguti55";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);

void setM1Speed(double speed)
{
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
        pwm_set_duty(12,0);
        pwm_set_duty(13,speed);
    } else {
        pwm_set_duty(12,speed);
        pwm_set_duty(13,0);
    }
}

void setM2Speed(double speed)
{
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
        pwm_set_duty(14,0);
        pwm_set_duty(15,speed);
    } else {
        pwm_set_duty(14,speed);
        pwm_set_duty(15,0);
    }
}

/*working variables*/
unsigned long lastTimeM1;
unsigned long lastTimeM2;
double errSumM1=0.0, lastErrM1;
double errSumM2=0.0, lastErrM2;

#include <SoftwareSerial.h>                     // to use pin 4 and 5 as RX and TX ports

#define sw_serial_rx_pin 4 //  Connect this pin to TX on the esp8266
#define sw_serial_tx_pin 5 //  Connect this pin to RX on the esp8266


#define KP 0.1
#define KI 0.01
#define KD 0.0

double Px=0.04,Ix=0.001,Dx=0.00;// 0.0025,0.0001,0.000025
double Py=KP,Iy=KI,Dy=KD;

// MAX is 5V   
#define MAXS 3.0  // 3V max
double X_OUTPUT_MIN= -MAXS;
double X_OUTPUT_MAX=  MAXS;
double Y_OUTPUT_MIN= -MAXS;
double Y_OUTPUT_MAX=  MAXS;

double xAngle,xSpeed=0.0;
double yAngle,ySpeed=0.0;

double xSetAngle=9.22, xSetAngleDt=1.0, xSetAngleOld=0.0;
double ySetAngle=0.0;

double xSetAxis=4.0;
double ySetAxis=0.0;


double XYtoDeg(double x){
  return  atan(x/12.0)/M_PI*90.0;
}

void SetPIDx(double Kp, double Ki, double Kd)
{
   Px = Kp;
   Ix = Ki;
   Dx = Kd;
   lastErrM1=0.0;
   errSumM1=0.0;
   xSpeed=0.0;
}

void SetPIDy(double Kp, double Ki, double Kd)
{
   Py = Kp;
   Iy = Ki;
   Dy = Kd;
   lastErrM2=0.0;
   errSumM2=0.0;
   ySpeed=0.0;
}

int stx,enx,sty,eny;
float sx=0,sy=0;

 void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

double Computex(double xAngle, double xSetAngle)
{
   // How long since we last calculated
   unsigned long now = micros();
   double timeChange = (double)(now - lastTimeM1)/1000000.0;
   // Compute all the working error variables
   double error = xSetAngle - xAngle;
   errSumM1 += (error * timeChange);
   double dErr = (error - lastErrM1) / timeChange;
  
  // kick gain
  #ifdef SERVO_KICK
   xSetAngleDt = 1.0 + 2.0*(xSetAngle - xSetAngleOld);
  #endif
  
   // Compute PID Output
   xSpeed = xSetAngleDt * (Px * error + Ix * errSumM1 + Dx * dErr);
  
   // Remember some variables for next time
   lastErrM1 = error;
   lastTimeM1 = now;
   return xSpeed;
}

double Computey(double yAngle, double ySetAngle)
{
   // How long since we last calculated
   unsigned long now = micros();
   double timeChange = (double)(now - lastTimeM2)/1000000.0;

   // Compute all the working error variables
   double error = ySetAngle - yAngle;
   errSumM2 += (error * timeChange);
   double dErr = (error - lastErrM2) / timeChange;
  
   // Compute PID Output
   ySpeed = Py * error + Iy * errSumM2 + Dy * dErr;
  
   // Remember some variables for next time
   lastErrM2 = error;
   lastTimeM2 = now;
   return ySpeed;
}
// Setup

String cLN = "";

void setup()
{
     int cnt = 0; 
	// Initialize the serial

	Serial.begin (115200);


	// Set pins (Important! All Pins must be initialized, the PWM SDK not works without this

	pinMode(12, OUTPUT);
	digitalWrite(12, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);

  Serial.println("WCoil start");
	////// Initialize the PWM

	// Initial duty -> all off
	for (uint8_t channel = 0; channel < PWM_CHANNELS; channel++) {
		pwm_duty_init[channel] = 0;
	}

	// Period

	uint32_t period = PWM_PERIOD;

	// Initialize

	pwm_init(period, pwm_duty_init, PWM_CHANNELS, io_info);

	// Commit
	pwm_start();

//
  Serial.println("WiFi start");

  // WiFi
   // attempt to connect to Wifi network:
  WiFi.begin(ssid, pass);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("Server start");
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status

  Serial.println("PWM start");
  delay(2000);
  time1 = micros(); // start time
  pwm_set_duty(0, 0);
  pwm_set_duty(0, 1);
  pwm_set_duty(0, 2);
  pwm_set_duty(0, 3);
  pwm_start();
}

// Loop
double GetEncPos(int ch){
  double angle=0.0;
    // Read RAW_ANGLE value from encoder
    // i2C master 1
    if(ch == 1 ){
    Wire.begin(2,0);
    } else {
    Wire.begin(5,4);      
    }
    Wire.setClock(400000);    
    Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_AS5601_DEV_ADDRESS, 2);
    uint16_t RawAngle1 = 0;
    RawAngle1  = ((uint16_t)Wire.read() << 8) & 0x0F00;
    RawAngle1 |= (uint16_t)Wire.read();
    
  angle=(double)RawAngle1*2*360.0/4096.0;
  //if(angle > 180.0 ) angle -= 360.0;
 
  return angle;
}

void loop()
{
    time0 = micros();
    xAngle=GetEncPos(1);
    yAngle=GetEncPos(2);
 
    // Raw angle value (0 ~ 4095) is stored in RawAngle
    time2 = micros();
 //  time2=millis();
   Serial.print(" xAngle=");
   Serial.print(xAngle);
   Serial.print(" yAngle=");
   Serial.print(yAngle);
   Serial.print(" time=");
   Serial.print((double)(time2-time0)/1000.0);
   Serial.println("msec");

//
//
  xSpeed=Computex(xAngle, xSetAngle);
  if(xSpeed < X_OUTPUT_MIN) xSpeed = X_OUTPUT_MIN;
  if(xSpeed > X_OUTPUT_MAX) xSpeed = X_OUTPUT_MAX;
//
  ySpeed=Computey(yAngle, ySetAngle);
  
  if(ySpeed < Y_OUTPUT_MIN ) ySpeed = Y_OUTPUT_MIN;
  if(ySpeed > Y_OUTPUT_MAX ) ySpeed = Y_OUTPUT_MAX;

  
  if(sSW){
    setM1Speed(xSpeed);
    setM2Speed(ySpeed);
  } else {
    setM1Speed(0.0);
    setM2Speed(0.0);    
  }
  
    delay(Delay);
    
  if(time0 % 500 == 0 ){
  Serial.print(" x=");
  Serial.print(xAngle);
  Serial.print(" tx=");
  Serial.print(xSetAngle);
  Serial.print(" xs=");
  Serial.printf("%5.4f",xSpeed);
  Serial.print(" P=");
  Serial.print(Px);
  Serial.print(" I=");
  Serial.print(Ix);
  Serial.print(" D=");
  Serial.print(Dx);
  
  Serial.print(" y=");
  Serial.print(yAngle);
  Serial.print(" ty=");
  Serial.print(ySetAngle);
  Serial.print(" ys=");
  Serial.printf("%5.4f",ySpeed);
  Serial.print(" P=");
  Serial.print(Py);
  Serial.print(" I=");
  Serial.print(Iy);
  Serial.print(" D=");
  Serial.print(Dy);
  
  Serial.print(" time=");
  Serial.print(float(time2-time0)/1000.0);
  Serial.println("msec");
  }

//
   WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            //client.println(currentLineNow);
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            //
            client.println("<p>Parameter setting</p>");
            client.println("<form action='http://");
            client.println(WiFi.localIP());
            client.println("' method='put'>");
            
            // angle set
            client.println("<p>xSetAngle=<input type='text' name='xa=' value='");
            client.println(xSetAngle);
            client.println("' size='20' />");
            client.println("ySetAngle=<input type='text' name='ya=' value='");
            client.println(ySetAngle);
            client.println("' /></p>" );
            
            
            // xy set
            client.println("<p>xPosition=<input type='text' name='xp=' value='");
            client.println(xSetAxis);
            client.println("' size='20' />");
            client.println("yPosition=<input type='text' name='yp=' value='");
            client.println(ySetAxis);
            client.println("' />" );
            client.println("</p>");
       
            
            client.println("<p>Px=<input type='text' name='Px=' value='");
            client.printf("%5.4f",Px);
            client.println("' size='20' />");
            client.println("Ix=<input type='text' name='Ix=' value='");
            client.printf("%10.9f",Ix);
            client.println("' />" );
            client.println("Dx=<input type='text' name='Dx=' value='");
            client.printf("%10.9f",Dx);
            client.println("' />" );
            client.println("</p>");
            client.println("<p>Py=<input type='text' name='Py=' value='");
            client.printf("%5.4f",Py);
            client.println("' size='20' />");
            client.println("Iy=<input type='text' name='Iy=' value='");
            client.printf("%10.9f",Iy);
            client.println("' />" );
            client.println("Dy=<input type='text' name='Dy=' value='");
            client.printf("%10.9f",Dy);
            client.println("' />" );
            client.println("</p>");

            client.println("<p>sSW=<input type='text' name='sSW=' value='");
            client.printf("%d",sSW);
            client.println("' />" );

            client.println("dSW=<input type='text' name='dSW=' value='");
            client.printf("%d",dSW);
            client.println("' />" );    

            client.println("pN=<input type='text' name='pN=' value='");
            client.printf("%d",patternNo);
            client.println("' /></p>" );     
             
            client.println("<input type='submit' name='' value='submit' /></p>");
            client.println("</form>");

            //
            // the content of the HTTP response follows the header:
 
            client.print(" time=");
            client.print(float(time2-time0)/1000.0);
            client.print("msec");
            client.println("<br>");
            //
            client.print(" xa=");
            client.print(xAngle);
            client.print(" txa=");
            client.print(xSetAngle);
            client.print(" xsp=");
            client.printf("%5.4f",xSetAxis);
            client.print(" xSpeed=");
            client.printf("%5.4f",xSpeed);
            client.print(" ya=");
            client.printf("%5.4f",yAngle);
            client.print(" tya=");
            client.print(ySetAngle);
            client.print(" ysp=");
            client.printf("%5.4f",ySetAxis);
            client.print(" ySpeed=");
            client.printf("%5.4f",ySpeed);
            client.print("<br>");
            // PID
            client.print(" Px=");
            client.printf("%5.4f",Px); 
            client.print(" Ix="); 
            client.printf("%10.9f",Ix); 
            client.print(" Dx="); 
            client.printf("%10.9f",Dx);
            client.print("<br>");
            client.print(" Py="); 
            client.printf("%5.4f",Py); 
            client.print(" Iy="); 
            client.printf("%10.9f",Iy); 
            client.print(" Dy="); 
            client.printf("%10.9f",Dy);  
            client.print("<br>");
            // The HTTP response ends with another blank line:
            client.print(" sSW="); 
            client.printf("%d",sSW);
            //
            client.print(" dSW="); 
            client.printf("%d",dSW);           
            client.println();

            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        if (currentLine.endsWith("HTTP/1.1")){
          currentLine.replace("?", "");
          currentLine.replace("%3D", "");
          currentLine.replace("&", ",");
          
          //client.println(currentLine);
          //
          // x=***.**,y=***.**,P=*.***,I=***,D=****
          if( currentLine.indexOf("xa=") >= 0){
            stx=int(currentLine.indexOf("xa="))+3;
            enx=int(currentLine.indexOf(",",stx));
            sx=currentLine.substring(stx, enx).toFloat();
          xSetAngle=sx;
          }
          if( currentLine.indexOf("ya=") >= 0){
            sty=int(currentLine.indexOf(",ya=",0))+4;
              if( currentLine.indexOf(",xa=",sty) == -1 ){
                eny=int(currentLine.indexOf(' ',sty));
              } else {
                eny=int(currentLine.indexOf(',Px=',sty))+3;
              }
            sy=currentLine.substring(sty, eny).toFloat();
          ySetAngle=sy;
          }
          
          /// Axis
          if( currentLine.indexOf("xp=") >= 0){
            stx=int(currentLine.indexOf("xp="))+3;
            enx=int(currentLine.indexOf(",",stx));
            sx=currentLine.substring(stx, enx).toFloat();
            xSetAxis=sx;
            xSetAngle=XYtoDeg(xSetAxis);
          }
          if( currentLine.indexOf("yp=") >= 0){
            sty=int(currentLine.indexOf(",yp=",0))+4;
              if( currentLine.indexOf(",Px=",sty) == -1 ){
                eny=int(currentLine.indexOf(' ',sty));
              } else {
                eny=int(currentLine.indexOf(',Px=',sty))+3;
              }
            sy=currentLine.substring(sty, eny).toFloat();
            ySetAxis=sy;
            ySetAngle=XYtoDeg(ySetAxis);
          }
          
          ///
          // PID  Px,Ix,Dx, Py,Iy,Dy
          if( currentLine.indexOf("Px=") >= 0){
                stx=int(currentLine.indexOf(",Px=",0))+4;
                enx=int(currentLine.indexOf(",",stx));
                Px=currentLine.substring(stx, enx).toFloat();
          }
          if( currentLine.indexOf("Ix=") >= 0){
                stx=int(currentLine.indexOf(",Ix="))+4;
                enx=int(currentLine.indexOf(",",stx));
                Ix=currentLine.substring(stx, enx).toFloat();
          }
          if( currentLine.indexOf("Dx=") >= 0){
                stx=int(currentLine.indexOf(",Dx="))+4;
                enx=int(currentLine.indexOf(",",stx));
                Dx=currentLine.substring(stx, enx).toFloat();
                xSpeed = ySpeed = 0;
                setM1Speed(xSpeed);
                setM2Speed(ySpeed);
                SetPIDx(Px,Ix,Dx);
          }
          if( currentLine.indexOf("Py=") >= 0){
                stx=int(currentLine.indexOf(",Py="))+4;
                enx=int(currentLine.indexOf(",",stx));
                Py=currentLine.substring(stx, enx).toFloat();
          }
          if( currentLine.indexOf("Iy=") >= 0){
                stx=int(currentLine.indexOf(",Iy="))+4;
                enx=int(currentLine.indexOf(",",stx));
                Iy=currentLine.substring(stx, enx).toFloat();
          }
          if( currentLine.indexOf("Dy=") >= 0){
                stx=int(currentLine.indexOf(",Dy="))+4;
                enx=int(currentLine.indexOf(",",stx));
                Dy=currentLine.substring(stx, enx).toFloat();
                xSpeed = ySpeed = 0;
                setM1Speed(xSpeed);
                setM2Speed(ySpeed);
                SetPIDy(Py,Iy,Dy);
          }
          if( currentLine.indexOf("sSW=") >= 0){
                stx=int(currentLine.indexOf(",sSW="))+5;
                enx=int(currentLine.indexOf(",",stx));
                sSW=currentLine.substring(stx, enx).toInt();
          }
          if( currentLine.indexOf("dSW=") >= 0){
                stx=int(currentLine.indexOf(",dSW="))+5;
                enx=int(currentLine.indexOf(" ",stx));
                dSW=currentLine.substring(stx, enx).toInt();
                drawCount=0; // How many draw image
                imgCount=0;
          }
          if( currentLine.indexOf("pN=") >= 0){
                stx=int(currentLine.indexOf(",pN="))+4;
                enx=int(currentLine.indexOf(" ",stx));
                patternNo=currentLine.substring(stx, enx).toInt();
                drawCount=0; // How many draw image
                imgCount=0;
          }
          // projection ball command
          
            cLN=currentLine;                 
                      }
                  }
              }
          }
}
