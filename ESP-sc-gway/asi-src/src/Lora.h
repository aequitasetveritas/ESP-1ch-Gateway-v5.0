#ifndef LoraSettings_h
#define LoraSettings_h

#include <AdminSettingsService.h>
#include <ESP8266React.h>

#define LORA_SETTINGS_FILE "/config/loraSettings.json"
#define LORA_SETTINGS_PATH "/rest/loraSettings"


// Clase que contiene las settings
class LoraListadeSettings {
 public:
   bool _modo_lorawan;
   uint32_t _frecuencia;
   int _sf;
   int _bw;
   bool _cad;
   uint8_t _backbone;
};

/*----------------------------------------------<____Settings>----*/
class LoraSettings : public AdminSettingsService<LoraListadeSettings> {
 public:
  LoraSettings(AsyncWebServer* server, FS* fs, SecurityManager* securityManager);
  ~LoraSettings();

  void loop();
  void onConfigUpdated();

 private:
  

 protected:
  void readFromJsonObject(JsonObject& root);
  void writeToJsonObject(JsonObject& root);
};

#endif
