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
#define LASER_PIN            2

// max 400 is 5V output

#define SPRT


char ssid[] = "sakaguti-network3";      //  your network SSID (name)
char pass[] = "sakaguti55";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);

#include <DRV8835MotorShield.h>
DRV8835MotorShield motors;

// SPI CS pins
#define S1 5
#define S2 15
#define S3 25

// Delay msec for motor rotate time
#define Delay 0 // 10 - 20msec

unsigned char values[2];
int16_t x,y;
double  c= (360.0/16383.0);

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

//SPIClass hspi(HSPI);

unsigned long time0, time1, time2;

/*working variables*/
unsigned long lastTimeM1;
unsigned long lastTimeM2;
double errSumM1=0.0, lastErrM1;
double errSumM2=0.0, lastErrM2;

boolean sSW=true;  // servo SW
boolean dSW=false; // whether image draw or not.

#define MAXDRAW 5 // How many draw image repeatable
int patternNo=0; // draw image pattern.
int drawCount=0; // How many draw image
int imgCount=0;
#define WNO  4000
#define WNO2 250
//#define SERVO_KICK

double XYtoDeg(double x){
  return  atan(x/12.0)/M_PI*90.0;
}

// draw image
double dataX[]={-0.05,  0.05, -0.05, 0.05, -0.05, 0.05, -0.05, 0.05};
double dataY[]={-0.02, -0.02,  0.0, 0.0,  0.02, 0.02,  0.0, 0.0 };

void drawImage(int pn)
{
  //
  static double xsetang,ysetang;
  static double CX,CY;
  int ic=0;
  static double R=1.0;
  double oR=5.0,dR=0.5;
  double C=2.5;
  
  if (drawCount == MAXDRAW){
    xSetAngle=xsetang;
    ySetAngle=ysetang;
    xSetAxis=CX;
    ySetAxis=CY;
    imgCount=0;
    drawCount=0;
    //R = oR;
    return;
  }
  if(drawCount == 0 && imgCount == 0){
    xsetang=xSetAngle;
    ysetang=ySetAngle;
    CX=xSetAxis;
    CY=ySetAxis;
    R = oR;
  } 
  switch(pn){
    case 0:
    // draw square
      if(imgCount>=4*WNO){
        imgCount=0;
      }
      //
      ic=(int)(imgCount/WNO);
      switch(ic){
        case 0:
          xSetAxis = -0.1*R+CX;
          ySetAxis = -0.1*C*R+CY;
          break;
        case 1:
          xSetAxis =  0.1*R+CX;
          ySetAxis = -0.1*C*R+CY;
          break;
        case 2:
          xSetAxis =  0.1*R+CX;
          ySetAxis =  0.1*C*R+CY;
          break;
        case 3:
          xSetAxis = -0.1*R+CX;
          ySetAxis =  0.1*C*R+CY;
          break;
        }
        xSetAngle=XYtoDeg(xSetAxis);
        ySetAngle=XYtoDeg(ySetAxis);
        imgCount++;
        if(imgCount >= 4*WNO){
            imgCount = 0;
            R += dR;
            drawCount++;
          }
        break;
    case 1:
    // draw circle
        xSetAxis = 0.15*R*cos(M_PI*(double)imgCount/(double)WNO)+CX;
        ySetAxis = 0.8*0.15*C*R*sin(M_PI*(double)imgCount/(double)WNO)+CY;
        xSetAngle=XYtoDeg(xSetAxis);
        ySetAngle=XYtoDeg(ySetAxis);
        imgCount++;  
        if(imgCount >= WNO*2){
            imgCount = 0;
            R += dR;
            drawCount++;
          }
        break;
    case 2:
    // draw data
        ic = imgCount/WNO2;
        xSetAxis = R*dataX[ic]+CX;
        ySetAxis = R*C*dataY[ic]+CY;
        xSetAngle=XYtoDeg(xSetAxis);
        ySetAngle=XYtoDeg(ySetAxis);
        imgCount++;
        if(imgCount >= sizeof(dataX)/sizeof(double)*WNO2){
            imgCount = 0;
            drawCount++;
          }
      break;
    default:
      break;
  }

  //
}

// read command from UART
void prjCmd(String currentLine){
          int stx,sty,enx,eny;
           if( currentLine.indexOf("cen=") >= 0){
           // move command (X,Y)
                stx=int(currentLine.indexOf("cen="))+4;
                enx=int(currentLine.indexOf(",",stx));
                xSetAxis=currentLine.substring(stx, enx).toFloat();
                stx=enx+1;
                enx=int(currentLine.indexOf("\0",stx));
                ySetAxis=currentLine.substring(stx, enx).toFloat();
                xSetAngle=XYtoDeg(xSetAxis);
                ySetAngle=XYtoDeg(ySetAxis);
          }
          if( currentLine.indexOf("srt") >= 0){
            // start command
            sSW=true;
          }
          if( currentLine.indexOf("stp") >= 0){
            // stop command
            sSW=false;
          }
          if( currentLine.indexOf("lpw") >= 0){
            // laser power (PWM)
          }
          if( currentLine.indexOf("sw0") >= 0){
            // draw pattern 0
            dSW=true;
            patternNo=0;
          }
          if( currentLine.indexOf("sw1") >= 0){
            // draw pattern 1
            dSW=true;
            patternNo=1;
          }
          if( currentLine.indexOf("sw2") >= 0){
            // draw pattern 1
            dSW=true;
            patternNo=2;
          }
         ///
         if( currentLine.indexOf("xa=") >= 0){
            stx=int(currentLine.indexOf("xa="))+3;
            enx=int(currentLine.indexOf(",",stx));
            xSetAngle==currentLine.substring(stx, enx).toFloat();
          }
          if( currentLine.indexOf("ya=") >= 0){
            sty=int(currentLine.indexOf(",ya=",0))+4;
              if( currentLine.indexOf(",xa=",sty) == -1 ){
                eny=int(currentLine.indexOf(' ',sty));
              } else {
                eny=int(currentLine.indexOf(',Px=',sty))+3;
              }
            ySetAngle=currentLine.substring(sty, eny).toFloat();
          }
          
          /// Axis
          if( currentLine.indexOf("xp=") >= 0){
            stx=int(currentLine.indexOf("xp="))+3;
            enx=int(currentLine.indexOf(",",stx));
            xSetAxis=currentLine.substring(stx, enx).toFloat();
            xSetAngle=XYtoDeg(xSetAxis);
          }
          if( currentLine.indexOf("yp=") >= 0){
            sty=int(currentLine.indexOf(",yp=",0))+4;
              if( currentLine.indexOf(",Px=",sty) == -1 ){
                eny=int(currentLine.indexOf(' ',sty));
              } else {
                eny=int(currentLine.indexOf(',Px=',sty))+3;
              }
            ySetAxis=currentLine.substring(sty, eny).toFloat();
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
                enx=int(currentLine.indexOf(",",stx));
                Dy=currentLine.substring(stx, enx).toFloat();
                xSpeed = ySpeed = 0;
                motors.setM1Speed(xSpeed);
                motors.setM2Speed(ySpeed);
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
  pinMode(LASER_PIN, OUTPUT);      // set the Laser pin mode
  digitalWrite(LASER_PIN, LOW);
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
  WiFi.begin(ssid, pass);
  for(int i=0;i<10;i++){
    if(WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    // wait 10 seconds for connection:
    delay(500);
    Serial.print(".");
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
  time1 = micros(); // start time
  //
}

int deltax=1;
int deltay=1;

// range 0- 16383  0-360deg  0.0021974 deg/unit
String currentLineNow = "";
char buf[256];
String uartCom="";

double x_bit0=0.0,x_bit1=0.0,x_bit2=0.0,x_bit3=0.0,x_bit4=0.0,x_bit5=0.0,x_bit6=0.0,x_bit7=0.0;
double sum0=0.0, sum1=0.0;
double x_res0=0.0, x_res1=0.0;

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
  if(angle > 180.0 ) angle -= 360.0;
 
  return angle;
}

void loop() {
 
// read UART command if avairable
    while (swSer.available() > 0) {
      uartCom += swSer.read();
    }
    if(uartCom.length() > 0 ){
      // do command processing
      prjCmd(uartCom);
       swSer.write("OK");
    }
  
  time0 = micros();
  xAngle=GetEncPos(S1);
  yAngle=GetEncPos(S2);  

    //dx buffer
  x_bit6=x_bit4; x_bit7=x_bit5;
  x_bit4=x_bit2; x_bit5=x_bit3;
  x_bit2=x_bit0; x_bit3=x_bit1;

  x_bit0=xAngle;
  x_bit1=yAngle;

  sum0=(x_bit0+x_bit2+x_bit4+x_bit6);
  sum1=(x_bit1+x_bit3+x_bit5+x_bit7);
  x_res0=sum0/4.0;
  x_res1=sum1/4.0;

  xAngle = x_res0;
//
  xSpeed=Computex(xAngle, xSetAngle);
  if(xSpeed < X_OUTPUT_MIN) xSpeed = X_OUTPUT_MIN;
  if(xSpeed > X_OUTPUT_MAX) xSpeed = X_OUTPUT_MAX;

  //time1 = micros();
  yAngle = x_res1;
//
  ySpeed=Computey(yAngle, ySetAngle);
  
  if(ySpeed < Y_OUTPUT_MIN ) ySpeed = Y_OUTPUT_MIN;
  if(ySpeed > Y_OUTPUT_MAX ) ySpeed = Y_OUTPUT_MAX;

  
  if(sSW){
    motors.setM1Speed(xSpeed);
    motors.setM2Speed(ySpeed);
    
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

 // keep current xSetAngle
   xSetAngleOld=xSetAngle;
   
  if( time2 - time1 > 5000 ){

  if(dSW) drawImage(patternNo);// draw image
  
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
                enx=int(currentLine.indexOf(",",stx));
                Dy=currentLine.substring(stx, enx).toFloat();
                xSpeed = ySpeed = 0;
                motors.setSpeeds(xSpeed,ySpeed);
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
          
            currentLineNow=currentLine;
                          }
                      }
                  }
              }
          }
         
}

 


