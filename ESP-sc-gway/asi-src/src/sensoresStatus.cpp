#include "sensoresStatus.h"
#include "../../estado.h"
#include <TimeLib.h>

SensoresStatus::SensoresStatus(AsyncWebServer* server, SecurityManager* securityManager) {
  server->on(SENSORES_STATUS_SERVICE_PATH,
             HTTP_GET,
             securityManager->wrapRequest(std::bind(&SensoresStatus::sensoresStatus, this, std::placeholders::_1),
                                          AuthenticationPredicates::NONE_REQUIRED));
}

static String toISOString(tm* time, bool incOffset) {
  char time_string[25];
  strftime(time_string, 25, incOffset ? "%FT%T%z" : "%FT%TZ", time);
  return String(time_string);
}

void SensoresStatus::sensoresStatus(AsyncWebServerRequest* request) {
  AsyncJsonResponse* response = new AsyncJsonResponse(false, 1024);
  JsonObject root = response->getRoot();

  // grab the current instant in unix seconds
  time_t ahora = now();

  root["time_utc"] = toISOString(gmtime(&ahora), false);
  root["latitud"] = ((float)gbl_sensores.latitud - 90000000.0) / 1000000.0;
  root["longitud"] = ((float)gbl_sensores.longitud - 180000000.0) / 1000000.0;
  root["temperatura"] = ((float)(gbl_sensores.temperatura) - 27315) / 100.0;
  root["humedad"] = (float)(gbl_sensores.humedad) / 100.0;
  root["presion"] = (float)(gbl_sensores.presion) / 10.0;
  root["velocidad"] = gbl_sensores.velocidad / 27.777777777 ; // a km/h
  root["direccion"] = gbl_sensores.direccion;
  root["pulsos"] = gbl_sensores.pulsos;
  root["tension"] = ((float)gbl_sensores.tension) / 100.0;
  


  

  // the current time in UTC
  //root["time_utc"] = toISOString(gmtime(&now), false);

  // local time as ISO String with TZ
  //root["time_local"] = toISOString(localtime(&now), true);

  // the sntp server name
  //root["server"] = sntp_getservername(0);

  // device uptime in seconds
  //root["uptime"] = millis() / 1000;

  response->setLength();
  request->send(response);
}
