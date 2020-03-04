// 1-channel LoRa Gateway for ESP8266
// Copyright (c) 2016, 2017, 2018 Maarten Westenberg version for ESP8266
// Version 5.3.3
// Date: 2018-08-25
//
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the MIT License
// which accompanies this distribution, and is available at
// https://opensource.org/licenses/mit-license.php
//
// NO WARRANTY OF ANY KIND IS PROVIDED
//
// Author: Maarten Westenberg (mw12554@hotmail.com)
//
// This file contains the ota code for the ESP Single Channel Gateway.

// Provide OTA server funcionality so the 1ch gateway can be updated 
// over the air.
// This code uses the ESPhttpServer functions to update the gateway.

#if A_OTA==1

//extern ArduinoOTAClass ArduinoOTA;

// Make sure that webserver is running before continuing

// ----------------------------------------------------------------------------
// setupOta
// Function to run in the setup() function to initialise the update function
// ----------------------------------------------------------------------------
void setupOta(char *hostname) {

	ArduinoOTA.begin();
#if DUSB>=1
	dbgpl(F("setupOta:: Started"));
#endif	
	// Hostname defaults to esp8266-[ChipID]
	ArduinoOTA.setHostname(hostname);
	
	ArduinoOTA.onStart([]() {
		String type;
		// XXX version mismatch of platform.io and ArduinoOtaa
		// see https://github.com/esp8266/Arduino/issues/3020
		//if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		//else // U_SPIFFS
		//	type = "filesystem";

		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		dbgpl("Start updating " + type);
	});
	
	ArduinoOTA.onEnd([]() {
		dbgpl("\nEnd");
	});
	
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	});
	
	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) dbgpl("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) dbgpl("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) dbgpl("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) dbgpl("Receive Failed");
		else if (error == OTA_END_ERROR) dbgpl("End Failed");
	});
	
#if DUSB>=1
	dbgpl("Ready");
	dbgp("IP address: ");
	dbgpl(WiFi.localIP());
#endif
	
	// Only if the Webserver is active also
#if A_SERVER==2										// Displayed for the moment
	ESPhttpUpdate.rebootOnUpdate(false);
   
	server.on("/esp", HTTP_POST, [&](){
   
      HTTPUpdateResult ret = ESPhttpUpdate.update(server.arg("firmware"), "1.0.0");
	  
      switch(ret) {
        case HTTP_UPDATE_FAILED:
            //PREi::sendJSON(500, "Update failed.");
			dbgpl(F("Update failed"));
            break;
        case HTTP_UPDATE_NO_UPDATES:
            //PREi::sendJSON(304, "Update not necessary.");
			dbgpl(F("Update not necessary"));
            break;
        case HTTP_UPDATE_OK:
            //PREi::sendJSON(200, "Update started.");
			dbgpl(F("Update started"));
            ESP.restart();
            break;
		default:
			dbgpl(F("setupOta:: Unknown ret="));
      }
	});
#endif
}


// ----------------------------------------------------------------------------
//
//
// ----------------------------------------------------------------------------
void updateOtaa() {

	String response="";
	printIP((IPAddress)WiFi.localIP(),'.',response);
	
	ESPhttpUpdate.update(response, 80, "/arduino.bin");

}


#endif