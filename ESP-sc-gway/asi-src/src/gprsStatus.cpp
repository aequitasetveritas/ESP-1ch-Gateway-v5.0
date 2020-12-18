#include "gprsStatus.h"
#include "../../estado.h"

GprsStatus::GprsStatus(AsyncWebServer* server, SecurityManager* securityManager) {
  server->on(GPRS_STATUS_SERVICE_PATH,
             HTTP_GET,
             securityManager->wrapRequest(std::bind(&GprsStatus::gprsStatus, this, std::placeholders::_1),
                                          AuthenticationPredicates::IS_AUTHENTICATED));
}

void GprsStatus::gprsStatus(AsyncWebServerRequest* request) {
  AsyncJsonResponse* response = new AsyncJsonResponse(false, 1024);
  JsonObject root = response->getRoot();

  // grab the current instant in unix seconds
  //time_t now = time(nullptr);

  // only provide enabled/disabled status for now
  root["connGprs"] = getEConnGprs();
  root["connRed"] = getEConnRed();
  root["up"] = getUpPackets();
  root["dw"] = getDownPackets();
  root["connBroker"] = getBrokerConn();
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
