/*
 * DRV8835 端子
 * M2PWM    Pin  35     B2
 * M1PWM    Pin  34     A2
 * M2DIR    Pin  33     B1
 * M1DIR    Pin  32　　　A1
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
#include <WiFi.h>
#include <WiFiUdp.h>
#include "FS.h"
#include "SPIFFS.h"

#include <SoftwareSerial.h>

// UART
SoftwareSerial swSer(1, 3, false, 256);

// make file data.txt
//#define FILEWRITE

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT  13

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ     5000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            2
#define MAXS 1.0  // 3V max


// max 400 is 5V output

#define SPRT

char ssid[] = "sakaguti-network3";      //  your network SSID (name)
char pass[] = "sakaguti55";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);

#include <DRV8835MotorShield.h>
DRV8835MotorShield motors;
 
#define S1 5
#define S2 15
#define S3 25

// Delay msec for motor rotate time
#define Delay 0 // 10 - 20msec

unsigned char values[2];
int16_t x,y;
double  c= (360.0/16383.0);

#define KP 0.2
#define KI 0.0001
#define KD 0.000025

double Px=0.05,Ix=0.0001,Dx=-0.000001;// 0.0025,0.0001,0.000025
double Py=KP,Iy=KI,Dy=KD;

// 400 is 5V   
double X_OUTPUT_MIN= -(MAXS/3.0);
double X_OUTPUT_MAX=  (MAXS/3.0);
double Y_OUTPUT_MIN= -MAXS;
double Y_OUTPUT_MAX=  MAXS;

double xAngle,xSpeed=0.0;
double yAngle,ySpeed=0.0;

double xSetAngle=0.0;
double ySetAngle=180.0;


//SPIClass hspi(HSPI);

unsigned long time0, time1, time2;

/*working variables*/
unsigned long lastTimeM1;
unsigned long lastTimeM2;
double errSumM1=0.0, lastErrM1;
double errSumM2=0.0, lastErrM2;

boolean sSW=true;  // servo SW

double Computex(double xAngle, double xSetAngle)
{
   /*How long since we last calculated*/
   unsigned long now = micros();
   double timeChange = (double)(now - lastTimeM1)/1000000.0;
   /*
   if( timeChange == 0 ){
    Serial.print("Error timeChange in Computex ");
    Serial.print(now);
    Serial.print(" ");
    Serial.println(lastTimeM1);
   }
   */
   /*Compute all the working error variables*/
   double error = xSetAngle - xAngle;
   errSumM1 += (error * timeChange);
   double dErr = (error - lastErrM1) / timeChange;
  
   /*Compute PID Output*/
   xSpeed = Px * error + Ix * errSumM1 + Dx * dErr;
  
   /*Remember some variables for next time*/
   lastErrM1 = error;
   lastTimeM1 = now;
   return xSpeed;
}
double Computey(double yAngle, double ySetAngle)
{
   /*How long since we last calculated*/
   unsigned long now = micros();
   double timeChange = (double)(now - lastTimeM2)/1000000.0;
   /*
     if( timeChange == 0 ){
      Serial.print("Error timeChange in Computey ");
      Serial.print(now);
      Serial.print(" ");
      Serial.println(lastTimeM2);
     }
     */
   /*Compute all the working error variables*/
   double error = ySetAngle - yAngle;
   errSumM2 += (error * timeChange);
   double dErr = (error - lastErrM2) / timeChange;
  
   /*Compute PID Output*/
   ySpeed = Py * error + Iy * errSumM2 + Dy * dErr;
  
   /*Remember some variables for next time*/
   lastErrM2 = error;
   lastTimeM2 = now;
   return ySpeed;
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

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t minval=value;
  if( minval > valueMax ) minval=valueMax;
  uint32_t duty = (8191 / valueMax) * minval;

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void readFileHTML(fs::FS &fs, const char * path, WiFiClient client){

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        Serial.println("Failed to open file for reading");
        return;
    }

    while(file.available()){
        client.printf("%s",file.read());  
    }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
       // Serial.println("Message appended");
    } else {
       // Serial.println("Append failed");
    }
}

boolean fileReadSw=false;

#include <NTPClient.h>
#include <WiFiUdp.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void ntpsetup(){
timeClient.begin();
timeClient.update();
Serial.println(timeClient.getFormattedTime());
}

void blinkSmartConfig() {
    digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(50);              // wait for a second 
    digitalWrite(2, LOW);    // turn the LED off by making the voltage LOW
    delay(50);
}

void setup() {
   int cnt = 0; 
    Serial.begin(115200);
//  UART
    swSer.begin(115200);
//
#ifdef FILEWRITE
    if(!SPIFFS.begin(true)){
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    deleteFile(SPIFFS, "/data.txt");
#endif

//
  pinMode(LED_PIN, OUTPUT);      // set the LED pin mode
  digitalWrite(LED_PIN, HIGH);
  motors.flipM1(true);
  motors.flipM2(true);

  // SPI　(vSPI)
  SPI.begin();//int8_t sck, int8_t miso, int8_t mosi, int8_t ss
  SPI.setFrequency(1000000); //SSD1331 のSPI Clock Cycle Time 50nsec
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  
  // SPI (hSPI)
  //pinMode(25, OUTPUT);
  //hspi.begin(14, 12, 13, 15);
  //hspi.setFrequency(7000000); //SSD1331 のSPI Clock Cycle Time 最低150ns
  //hspi.setDataMode(SPI_MODE3);


  // WiFi
   // attempt to connect to Wifi network:
  for(int i=0;i<3;i++){
    if(status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
    }
  }

  // smartConfig
  if(status != WL_CONNECTED) {
    Serial.println("SmartConfig Start.");
      // if wifi cannot connect start smartconfig
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if(cnt++ >= 15){
       WiFi.beginSmartConfig();
       while(1){
           delay(500);
           if(WiFi.smartConfigDone()){
             Serial.println("SmartConfig Success");
             blinkSmartConfig();
             break;
            }
          }
        }
    }
  }
  
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status
  delay(2000);
  //
}

int deltax=1;
int deltay=1;

// range 0- 16383  0-360deg  0021974 deg/unit
String currentLineNow = "";
char buf[256];
String uartCom="";

double GetEncPos(int ch){
  double angle=0.0;
  //  y axis 
  digitalWrite(ch, LOW);
  delayMicroseconds(10);
  for(int16_t i=0; i<2; i++) {
   values[i] = SPI.transfer(0x00); // MISIは使わないので、引数は任意
  }
  digitalWrite(ch, HIGH);
  x = ((values[0]<<8) + values[1]) & 0x3fff;
  angle=(double)x*c;
  if(ch == S1 ){
  if(angle > 180.0 ) angle -= 360.0;
  }
  return angle;
}

void loop() { 
  
// read UART command if avairable
    while (swSer.available() > 0) {
      uartCom += swSer.read();
    }
    if(uartCom.length() > 0 ){
      // do command processing
      //prjCmd(uartCom);
    }
  
  time0 = micros();
  xAngle=GetEncPos(S1);
//
  xSpeed=Computex(xAngle, xSetAngle);
  if(xSpeed < X_OUTPUT_MIN) xSpeed = X_OUTPUT_MIN;
  if(xSpeed > X_OUTPUT_MAX) xSpeed = X_OUTPUT_MAX;

  time1 = micros();
  yAngle=GetEncPos(S2);  
//
  ySpeed=Computey(yAngle, ySetAngle);
  
  if(ySpeed < Y_OUTPUT_MIN ) ySpeed = Y_OUTPUT_MIN;
  if(ySpeed > X_OUTPUT_MAX ) ySpeed = X_OUTPUT_MAX;

  
  if(sSW){
    motors.setM1Speed(xSpeed);
    digitalWrite(LED_PIN, LOW);

    motors.setM2Speed(ySpeed);
    digitalWrite(LED_PIN, LOW);
    
  } else {
    motors.setM1Speed(0.0);
    motors.setM2Speed(0.0);    
  }
  
  
  delay(Delay);
  time2 = micros();
 
//
#ifdef FILEWRITE
    sprintf(buf,"%d,%f,%f,%d,%f,%f\n",time1,xAngle,xSetAngle,time2,yAngle,ySetAngle);
    appendFile(SPIFFS, "/data.txt", buf);
#endif

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
            client.println("xSetAnglet=<input type='text' name='x=' value='");
            client.println(xSetAngle);
            client.println("' size='20' />");
            client.println("ySetAngle=<input type='text' name='y=' value='");
            client.println(ySetAngle);
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
            client.println("</p>");
            
            client.println("<p><input type='submit' name='' value='submit' /></p>");
            client.println("</form>");

            //
            // the content of the HTTP response follows the header:
 
            client.print(" time=");
            client.print(float(time2-time0)/1000.0);
            client.print("msec");
            client.println("<br>");
            //
            client.print(" x=");
            client.print(xAngle);
            client.print(" tx=");
            client.print(xSetAngle);
            client.print(" xs=");
            client.printf("%5.4f",xSpeed);
            client.print("<br>");
            client.print(" y=");
            client.printf("%5.4f",yAngle);
            client.print(" ty=");
            client.print(ySetAngle);
            client.print(" ys=");
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
          if( currentLine.indexOf("x=") >= 0){
            stx=int(currentLine.indexOf("x="))+2;
            enx=int(currentLine.indexOf(",",stx));
            sx=currentLine.substring(stx, enx).toFloat();
          xSetAngle=sx;
          }
          if( currentLine.indexOf("y=") >= 0){
            sty=int(currentLine.indexOf(",y=",0))+3;
              if( currentLine.indexOf(",px=",sty) == -1 ){
                eny=int(currentLine.indexOf(' ',sty));
              } else {
                eny=int(currentLine.indexOf(',Px=',sty))+3;
              }
            sy=currentLine.substring(sty, eny).toFloat();
          ySetAngle=sy;
          }
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
                motors.setSpeeds(xSpeed,ySpeed);
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
                enx=int(currentLine.indexOf(" ",stx));
                Dy=currentLine.substring(stx, enx).toFloat();
                xSpeed = ySpeed = 0;
                motors.setSpeeds(xSpeed,ySpeed);
                SetPIDy(Py,Iy,Dy);
          }
          if( currentLine.indexOf("sSW=") >= 0){
                stx=int(currentLine.indexOf(",sSW="))+5;
                enx=int(currentLine.indexOf(" ",stx));
                sSW=currentLine.substring(stx, enx).toInt();
          }
          // projection ball command
          
            currentLineNow=currentLine;
                          
                      }
                  }
              }
          }
}

 


