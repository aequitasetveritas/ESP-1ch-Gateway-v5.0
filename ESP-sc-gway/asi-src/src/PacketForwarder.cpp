// Endpoint lora settings
#include "PacketForwarder.h"

extern void pf_settings_callback(PFListaSettings settings);

PFSettings::PFSettings(AsyncWebServer* server, FS* fs, SecurityManager* securityManager) :
    AdminSettingsService(server, fs, securityManager, PF_SETTINGS_PATH, PF_SETTINGS_FILE) {
}

PFSettings::~PFSettings() {

}

void PFSettings::loop() {
  
}

void PFSettings::onConfigUpdated() {
  pf_settings_callback(_settings);
}

void PFSettings::readFromJsonObject(JsonObject& root) {
      _settings._protocolo = root["protocolo"];
      _settings._stHost = root["stHost"] | "";
      _settings._stPort = root["stPort"];
      _settings._statInterval = root["statInterval"];
      _settings._keepAlive = root["keepAlive"];
      _settings._mqttHost = root["mqttHost"] | "";
      _settings._mqttPass = root["mqttPass"] | "";
      _settings._mqttPort = root["mqttPort"];
      _settings._mqttUser = root["mqttUser"] | "";
}

void PFSettings::writeToJsonObject(JsonObject& root) {
  // connection settings
      root["protocolo"] = _settings._protocolo;
      root["stHost"] = _settings._stHost;
      root["stPort"] = _settings._stPort;
      root["statInterval"] = _settings._statInterval;
      root["keepAlive"] = _settings._keepAlive;
      root["mqttHost"] = _settings._mqttHost;
      root["mqttPass"] = _settings._mqttPass;
      root["mqttPort"] = _settings._mqttPort;
      root["mqttUser"] = _settings._mqttUser;
}
