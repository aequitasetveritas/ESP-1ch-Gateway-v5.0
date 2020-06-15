// Endpoint lora settings
#include "gprsSettings.h"

//extern void gprs_settings_reconfig(int sf, int bw, uint32_t frec);

GprsSettings::GprsSettings(AsyncWebServer* server, FS* fs, SecurityManager* securityManager) :
    AdminSettingsService(server, fs, securityManager, GPRS_SETTINGS_PATH, GPRS_SETTINGS_FILE) {
}

GprsSettings::~GprsSettings() {
}

void GprsSettings::loop() {
  
}

void GprsSettings::onConfigUpdated() {
  //lora_settings_reconfig(_settings._sf, _settings._bw, _settings._frecuencia);
}

void GprsSettings::readFromJsonObject(JsonObject& root) {
      _settings._apn = root["apn"] | "";
      _settings._user = root["user"] | "";
      _settings._pass = root["pass"] | "";
}

void GprsSettings::writeToJsonObject(JsonObject& root) {
  // connection settings
      root["apn"] = _settings._apn;
      root["user"] = _settings._user;
      root["pass"] = _settings._pass;
}
