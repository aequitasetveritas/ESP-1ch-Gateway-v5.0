#include "estado.h"

struct estado{
    bool connGprs;
    bool connRed;
    bool connMqttHost;
};

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