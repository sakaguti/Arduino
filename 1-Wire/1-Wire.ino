#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <WiFiUdp.h>
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 27
int numberOfDevices; // Number of temperature devices found

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
char ssid[] = "sakaguti-network3";      //  your network SSID (name)
char pass[] = "sakaguti55";   // your network password
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);
IPAddress ip(192, 168, 1, 53); //ip
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

void setup(void)
{
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
  
  // Start up the library
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
  // Grab a count of devices on the wire
  //numberOfDevices = sensors.getDeviceCount();  ????  numberOfDevices is zero

}


String currentLineNow = "";
char buf[256];

void loop(void)
{ 
  float temp[8];
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  //Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.println("DONE");
  delay(400); 
  numberOfDevices=8;
  for(int i=0;i<numberOfDevices;i++){
  portDISABLE_INTERRUPTS();
  temp[i]=sensors.getTempCByIndex(i);
  portENABLE_INTERRUPTS();
  }

  /*
  for(int i=0;i<numberOfDevices;i++){
  //Serial.print("Temperature for Device ");
  Serial.print(i);
  Serial.print(" ");
  Serial.println(temp[i]); // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  }  
 */

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
            client.println("<p>DS18B20</p>");
            for(int i=0;i<numberOfDevices;i++){
              client.println("<p>No.");
              client.println(i);
              client.println("   Temp ");
              client.println(temp[i]);
              client.println(" deg");
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
