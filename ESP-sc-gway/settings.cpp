#include "settings.h"
#include "asi-src/src/asi.h"
#include "asi-src/src/PacketForwarder.h"

// ############################## MQTT #########################################ar

static char host[100];
static char user[100];
static char pass[100];


const char * settings_mqtt_server(){
	PFListaSettings s = asi_pfSettings();
	memcpy(host,s._mqttHost.c_str(),s._mqttHost.length());
	return host;
}

uint16_t settings_mqtt_port(){
	PFListaSettings s = asi_pfSettings();
	return s._mqttPort;
}

const char * settings_mqtt_user(){
	PFListaSettings s = asi_pfSettings();
	memcpy(user,s._mqttUser.c_str(),s._mqttUser.length());
	return user;
}

const char * settings_mqtt_pass(){
	PFListaSettings s = asi_pfSettings();
	memcpy(pass,s._mqttPass.c_str(),s._mqttPass.length());
	return pass;
}

uint16_t settings_stats_interval(){
	PFListaSettings s = asi_pfSettings();
	return s._statInterval;
}



S_PROTOCOL settings_protocol(){
	PFListaSettings s = asi_pfSettings();
	if(s._protocolo == 1){
		return SEMTECH_PF_UDP;
	}else if(s._protocolo == 2){
		return MQTTBRIDGE_TCP;
	}else if(s._protocolo == 3){
		return MODO_AGROTOOLS;
	}
}







S_BACKBONE settings_backbone(){
	return BACKBONE_WIFI;
}

const char * settings_apn(){
	return "internet.personal.com";
}

const char * settings_gprs_user(){
	return "";
}

const char * settings_gprs_pass(){
	return "";
}


S_MODO settings_modo(){
	return MODO_CUSTOM;
}

uint16_t settings_tb_mqtt_port(){
	return 1883;
}

const char * settings_tb_mqtt_server(){
	return "agrotools.qts-ar.com.ar";
}

const char * settings_tb_mqtt_user(){
	return "eT9STX5PGgNS7Nvggc3F";
} //AccessToken

