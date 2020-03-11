#ifndef ASI_H
#define ASI_H
#include "asi-src/src/PacketForwarder.h"

void asi_begin();
void asi_loop();


bool cadGet();
void asi_forceGetAllRadioSettings();
PFListaSettings asi_pfSettings();

#endif