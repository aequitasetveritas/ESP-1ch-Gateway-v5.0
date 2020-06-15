#ifndef GprsStatus_h
#define GprsStatus_h

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <ESPAsyncWebServer.h>
#include <SecurityManager.h>

#define GPRS_STATUS_SERVICE_PATH "/rest/gprsStatus"

class GprsStatus {
 public:
  GprsStatus(AsyncWebServer* server, SecurityManager* securityManager);

 private:
  void gprsStatus(AsyncWebServerRequest* request);
};

#endif
