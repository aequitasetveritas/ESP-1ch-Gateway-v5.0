#ifndef SensoresStatus_h
#define SensoresStatus_h

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <ESPAsyncWebServer.h>
#include <SecurityManager.h>

#define SENSORES_STATUS_SERVICE_PATH "/rest/sensores"

class SensoresStatus {
 public:
  SensoresStatus(AsyncWebServer* server, SecurityManager* securityManager);
 private:
  void sensoresStatus(AsyncWebServerRequest* request);
};

#endif
