#include "settings.h"

const char * settings_mqtt_server(){
	return "mqtt.flespi.io";
	//return "192.168.88.16";
}

S_PROTOCOL settings_protocol(){
	return MQTTBRIDGE_TCP;
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