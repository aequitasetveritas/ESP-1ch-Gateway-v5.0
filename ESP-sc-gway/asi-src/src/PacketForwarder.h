#ifndef PFSettings_h
#define PFSettings_h

#include <AdminSettingsService.h>
#include <ESP8266React.h>

#define PF_SETTINGS_FILE "/config/packetforwarder.json"
#define PF_SETTINGS_PATH "/rest/packetforwarderSettings"


// Clase que contiene las settings
/* 
"protocolo":1,
    "stHost":"192.168.88.16",
    "stPort":1700,
    "statInterval":60,
    "keepAlive":15,
    "mqttHost":"mqtt.flespi.io",
    "mqttPort":1883,
    "mqttUser":"",
    "mqttPass":""
    */

class PFListaSettings {
 public:
   uint8_t _protocolo;
   String _stHost;
   uint16_t _stPort;
   uint16_t _statInterval;
   uint16_t _keepAlive;
   String _mqttHost;
   uint16_t _mqttPort;
   String _mqttUser;
   String _mqttPass;
};

/*----------------------------------------------<____Settings>----*/
class PFSettings : public AdminSettingsService<PFListaSettings> {
 public:
  PFSettings(AsyncWebServer* server, FS* fs, SecurityManager* securityManager);
  ~PFSettings();

  void loop();
  void onConfigUpdated();

 private:
  

 protected:
  void readFromJsonObject(JsonObject& root);
  void writeToJsonObject(JsonObject& root);
};

#endif
