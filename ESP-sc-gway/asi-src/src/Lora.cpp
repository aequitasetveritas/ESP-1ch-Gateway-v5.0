#include "Lora.h"

extern void lora_settings_reconfig(int sf, int bw, uint32_t frec);

LoraSettings::LoraSettings(AsyncWebServer* server, FS* fs, SecurityManager* securityManager) :
    AdminSettingsService(server, fs, securityManager, LORA_SETTINGS_PATH, LORA_SETTINGS_FILE) {
}

LoraSettings::~LoraSettings() {
}

void LoraSettings::loop() {
  
}

void LoraSettings::onConfigUpdated() {
  lora_settings_reconfig(_settings._sf, _settings._bw, _settings._frecuencia);
}

void LoraSettings::readFromJsonObject(JsonObject& root) {
      _settings._modo_lorawan = root["mode_lorawan"];
      _settings._frecuencia = root["frecuencia"];
      _settings._sf = root["sf"];
      _settings._bw = root["bw"];
      _settings._cad = root["cad"];
      _settings._backbone = root["backbone"];
}

void LoraSettings::writeToJsonObject(JsonObject& root) {
  // connection settings
      root["mode_lorawan"] = _settings._modo_lorawan;
      root["frecuencia"] = _settings._frecuencia;
      root["sf"] = _settings._sf;
      root["bw"] = _settings._bw;
      root["cad"] = _settings._cad;
      root["backbone"] = _settings._backbone;
}
