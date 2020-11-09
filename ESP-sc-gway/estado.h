#ifndef ESTADO_H
#define ESTADO_H
#include <stdint.h>

struct sensores{
    uint32_t latitud;
	uint32_t longitud;
	uint16_t temperatura;
	uint16_t humedad;
	uint16_t velocidad;
	uint16_t direccion;
	uint16_t pulsos;
	uint16_t tension;
	uint16_t presion;
};

extern struct sensores gbl_sensores;

void estadoInit();
void setEConnGprs(bool s);
void setEConnRed(bool s);
void setEConnMqttHost(bool s);
bool getEConnGprs();
bool getEConnRed();
bool getEConnMqttHost();


#endif