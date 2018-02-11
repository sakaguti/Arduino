// Copyright Nat Weerawan 2015-2016
// MIT License

#include <Arduino.h>
#include <WiFiConnector.h>
#include <ESP8266WebServer.h>

#ifndef WIFI_SSID
  #define WIFI_SSID       "Nat"
  #define WIFI_PASSPHRASE "1234567890"
#endif

WiFiConnector wifi(WIFI_SSID, WIFI_PASSPHRASE);
ESP8266WebServer server(80);

void init_hardware()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(10);
  Serial.println();
  Serial.println("BEGIN");
}

void init_webserver() {
  server.on ("/", []() {
    static String responseHTML = F(""
    "<!doctype html>"
    "<html>"
    "  <head>"
    "    <script src='https://ajax.googleapis.com/ajax/libs/angularjs/1.5.3/angular.min.js'></script>" ""
    ""
    "<script type=\"text/javascript\">"
    "   angular.module(\"NatApp\", [])"
    "     .controller(\"CMMCController\", function($scope, $http, $interval) { "
    "       var successCallback = function(response) {                 "
    "          console.log(\"success\", response.data.millis);         "
    "          $scope.millis = response.data.millis;                   "
    "       };                                                         "
    "       var errorCallback = function(response) {                   "
    "          console.log(\"error\", response)                        "
    "       };                                                         "
    "       var polling = function() {                                 "
    "         $http({                                                  "
    "           method: 'GET',                                         "
    "           url: '/millis'                                         "
    "         }).then(successCallback, errorCallback);                 "
    "       };                                                         "
    "                                                                  "
    "       $interval(polling, 500);                                   "
    "       console.log($scope);                                       "
    "   });"
    "</script>"
    "  </head>"
    "  <body ng-app=\"NatApp\">"
    "  <h1>Hello World!</h1><p>This is an esp8266 webpage example."
    "  that you can embeded angular application inside! </p>"
    "  <h1>ESP8266 WebServer + AngularJS</h1>"
    "   <div ng-controller=\"CMMCController\">"
    "      <label>Input your name:</label>"
    "      <input type='text' ng-model='yourName' placeholder='Enter a name here'>"
    "      <hr>"
    "      <h1>[{{millis}}] Hello, {{yourName}}</h1>"
    "      <img src=\"https://www.cmmakerclub.com/wp-content/uploads/2014/07/logo1.png\" />"
    "    </div>"
    "  </body>"
    "</html>");
    server.send (200, "text/html", responseHTML.c_str() );
  });

  server.on ( "/millis", []() {
    char buff[100];
    String ms = String(millis());
    sprintf(buff, "{\"millis\": %s }", ms.c_str());
    server.send ( 200, "text/plain", buff );
  });
}

void init_wifi() {
  wifi.init();
  wifi.on_connecting([&](const void* message)
  {
    Serial.print("Connecting to ");
    Serial.println(wifi.get("ssid") + ", " + wifi.get("password"));
    delay(200);
  });

  wifi.on_connected([&](const void* message)
  {
    // Print the IP address
    Serial.print("WIFI CONNECTED => ");
    Serial.println(WiFi.localIP());

    init_webserver();
    server.begin();
    Serial.println("SERVER Started.");
  });

  wifi.on_disconnected([&](const void* message) {
    server.close();
  });
}

void setup()
{
  init_hardware();
  init_wifi();
  wifi.connect();
}

void loop()
{
  wifi.loop();
  server.handleClient();
}
