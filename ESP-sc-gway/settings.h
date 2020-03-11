#ifndef SETTINGS_H
#define SETTINGS_H
#include <stdint.h>

typedef enum {SEMTECH_PF_UDP, MQTTBRIDGE_TCP} S_PROTOCOL;
typedef enum {BACKBONE_WIFI, BACKBONE_GPRS} S_BACKBONE;

const char * settings_mqtt_server();
uint16_t settings_mqtt_port();
const char * settings_mqtt_user();
const char * settings_mqtt_pass();


S_PROTOCOL settings_protocol();
S_BACKBONE settings_backbone();

const char * settings_apn(); 
const char * settings_gprs_user(); 
const char * settings_gprs_pass();

#endif