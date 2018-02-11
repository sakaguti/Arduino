#include <SoftwareSerial.h>
#include<stdio.h>
#include<stdlib.h>

SoftwareSerial swSer(1, 3, false, 256);

void prjCmd(String currentLine){
          int stx,enx;
           if( currentLine.indexOf("cen=") >= 0){
           // move command (X,Y)
                stx=int(currentLine.indexOf("cen="))+("cen=".length());
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
            dSW=True;
            patternNo=0;
          }
          if( currentLine.indexOf("sw1") >= 0){
            // draw pattern 1
            dSW=True;
            patternNo=1;
          }
          if( currentLine.indexOf("sw2") >= 0){
            // draw pattern 1
            dSW=True;
            patternNo=2;
          }          
}

String uartCom="";

void setup() {
  Serial.begin(115200);
  swSer.begin(115200);

  Serial.println("\nSoftware serial test started");

  for (char ch = ' '; ch <= 'z'; ch++) {
    swSer.write(ch);
  }
  swSer.println("");

}

void loop() {
// read UART command if avairable
    while (swSer.available() > 0) {
      uartCom += swSer.read();
    }
    if(uartCmd.length() > 0 ){
      // do command processing
      prjCmd(uartCom);
    }
}
