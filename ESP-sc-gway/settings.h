#ifndef SETTINGS_H
#define SETTINGS_H

typedef enum {SEMTECH_PF_UDP, MQTTBRIDGE_TCP} S_PROTOCOL;
typedef enum {BACKBONE_WIFI, BACKBONE_GPRS} S_BACKBONE;

const char * settings_mqtt_server();
S_PROTOCOL settings_protocol();
S_BACKBONE settings_backbone();

const char * settings_apn(); 
const char * settings_gprs_user(); 
const char * settings_gprs_pass();

#endif