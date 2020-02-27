#include "settings.h"

const char * settings_mqtt_server(){
	return "192.168.88.16";
}

S_PROTOCOL settings_protocol(){
	return MQTTBRIDGE_TCP;
}