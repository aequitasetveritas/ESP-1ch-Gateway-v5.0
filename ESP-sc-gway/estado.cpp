#include "estado.h"
#include <stdint.h>

extern uint32_t cp_dwnb;
extern uint32_t cp_nb_rx_rcv;

struct estado{
    bool connGprs;
    bool connRed;
    bool connMqttHost;
};

struct sensores gbl_sensores;

static struct estado E;

void estadoInit(){
    E.connGprs = false;
    E.connRed = false;
    E.connMqttHost = false;
}

void setEConnGprs(bool s){
    E.connGprs = s;
}

void setEConnRed(bool s){
    E.connRed = s;
}

void setEConnMqttHost(bool s){
    E.connMqttHost = s;
}

bool getEConnGprs(){
    return E.connGprs;
}

bool getEConnRed(){
    return E.connRed;
}

bool getEConnMqttHost(){
    return E.connMqttHost;
}

uint32_t getUpPackets(){
    return cp_nb_rx_rcv;
}

uint32_t getDownPackets(){
    return cp_dwnb;
}

bool getBrokerConn(){
    return getEConnMqttHost();
}