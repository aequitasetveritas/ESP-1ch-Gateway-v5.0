#ifndef GprsSettings_h
#define GprsSettings_h

#include <AdminSettingsService.h>
#include <ESP8266React.h>

#define GPRS_SETTINGS_FILE "/config/gprs.json"
#define GPRS_SETTINGS_PATH "/rest/gprsSettings"


// Clase que contiene las settings
class GprsListadeSettings {
 public:
   String _apn;
   String _user;
   String _pass;
};

/*----------------------------------------------<____Settings>----*/
class GprsSettings : public AdminSettingsService<GprsListadeSettings> {
 public:
  GprsSettings(AsyncWebServer* server, FS* fs, SecurityManager* securityManager);
  ~GprsSettings();

  void loop();
  void onConfigUpdated();

 private:
  

 protected:
  void readFromJsonObject(JsonObject& root);
  void writeToJsonObject(JsonObject& root);
};

#endif
