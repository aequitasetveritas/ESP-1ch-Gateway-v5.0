#ifndef ESTADO_H
#define ESTADO_H

void estadoInit();
void setEConnGprs(bool s);
void setEConnRed(bool s);
void setEConnMqttHost(bool s);
bool getEConnGprs();
bool getEConnRed();
bool getEConnMqttHost();


#endif