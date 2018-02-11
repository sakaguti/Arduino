/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInput
*/

int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
#include <WiFi.h>
#include <WiFiUdp.h>

char ssid[] = "sakaguti-network3";      //  your network SSID (name)
char pass[] = "sakaguti55";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);
IPAddress ip(192, 168, 1, 52); //ip
IPAddress gateway(192,168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress DNS(192, 168, 1, 1);

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

void setup() {
  int cnt=0;
    // start serial port
  Serial.begin(115200);
 // WiFi
   // attempt to connect to Wifi network:
  WiFi.config(ip, gateway, subnet, DNS); //static_ip
  WiFi.begin(ssid, pass);  
 //  tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, "myhostname");
  for(int i=0;i<100;i++){
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
             break;
            }
          }
        }
    }
  }
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status
  
}

int             numberOfDevices=1;
String currentLineNow = "";

void loop() {
  float moist=0.0;
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  moist = (float)sensorValue/4095.0*100.0; 
  Serial.println(moist); 
  
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
            
            numberOfDevices=1;
            //
            client.println("<p>Moisture Sensor</p>");
            for(int i=0;i<numberOfDevices;i++){
              client.println("<p>No.");
              client.println(i);
              client.println("   Moist ");
              client.println(moist);
              client.println(" %");
              client.println("</p>");
            }
            
            //client.println("<form action='http://");
            //client.println(WiFi.localIP());
            //client.println("' method='put'>");
            //
            // angle set
            //client.println("<p>xSetAngle=<input type='text' name='xa=' value='");
            //client.println();
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
            //stx=int(currentLine.indexOf("xa="))+3;
            //enx=int(currentLine.indexOf(",",stx));
            //sx=currentLine.substring(stx, enx).toFloat();
          }
                    
            currentLineNow=currentLine;
                          }
                      }
                }
          }
}
