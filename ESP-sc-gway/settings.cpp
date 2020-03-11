#include "settings.h"
#include "asi-src/src/asi.h"
#include "asi-src/src/PacketForwarder.h"

// ############################## MQTT #########################################

const char * settings_mqtt_server(){
	PFListaSettings s = asi_pfSettings();
	return s._mqttHost.c_str();
}

uint16_t settings_mqtt_port(){
	PFListaSettings s = asi_pfSettings();
	return s._mqttPort;
}

const char * settings_mqtt_user(){
	PFListaSettings s = asi_pfSettings();
	return s._mqttUser.c_str();
}

const char * settings_mqtt_pass(){
	PFListaSettings s = asi_pfSettings();
	return s._mqttPass.c_str();
}





S_PROTOCOL settings_protocol(){
	PFListaSettings s = asi_pfSettings();
	return (s._protocolo) == 1 ? SEMTECH_PF_UDP : MQTTBRIDGE_TCP;
}







S_BACKBONE settings_backbone(){
	return BACKBONE_GPRS;
}

const char * settings_apn(){
	return "igprs.claro.com.ar";
}

const char * settings_gprs_user(){
	return "";
}

const char * settings_gprs_pass(){
	return "";
}