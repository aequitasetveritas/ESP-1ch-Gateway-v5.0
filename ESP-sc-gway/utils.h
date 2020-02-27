#ifndef UTILS_H
#define UTILS_H
#include <stdint.h>
#include <Arduino.h>

void gway_failed(const char *file, uint16_t line);
void die(const char *s);
int SerialName(char * a, String& response);
void SerialStat(uint8_t intr);
void SerialTime();

#endif