#include "asi.h"
#include "DemoProject.h"
#include "Lora.h"
#include "PacketForwarder.h"
#include <ESP8266React.h>
#include <FS.h>

#define SERIAL_BAUD_RATE 115200

AsyncWebServer server(80);
ESP8266React esp8266React(&server, &SPIFFS);
DemoProject demoProject = DemoProject(&server, &SPIFFS, esp8266React.getSecurityManager());
LoraSettings loraSettings = LoraSettings(&server, &SPIFFS, esp8266React.getSecurityManager());
PFSettings pfSettings = PFSettings(&server, &SPIFFS, esp8266React.getSecurityManager());

void asi_begin() {
#ifdef ESP32
  SPIFFS.begin(true);
#elif defined(ESP8266)
  SPIFFS.begin();
#endif

  // start the framework and demo project
  esp8266React.begin();

  // start the demo project
  demoProject.begin();
  loraSettings.begin();
  pfSettings.begin();

  // start the server
  server.begin();
}

void asi_loop() {
  // run the framework's loop function
  esp8266React.loop();

  // run the demo project's loop function
  demoProject.loop();
}

bool cadGet(){
  LoraListadeSettings s = loraSettings.fetch();
  return s._cad;
}

void asi_forceGetAllRadioSettings(){
  loraSettings.onConfigUpdated();
}