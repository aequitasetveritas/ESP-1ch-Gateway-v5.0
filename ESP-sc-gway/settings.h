#ifndef SETTINGS_H
#define SETTINGS_H

typedef enum {SEMTECH_PF_UDP, MQTTBRIDGE_TCP} S_PROTOCOL;

const char * settings_mqtt_server();
S_PROTOCOL settings_protocol();

#endif