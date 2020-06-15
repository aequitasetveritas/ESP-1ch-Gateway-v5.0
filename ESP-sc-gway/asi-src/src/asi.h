#ifndef ASI_H
#define ASI_H
#include "asi-src/src/PacketForwarder.h"
#include "asi-src/src/Lora.h"
#include "asi-src/src/gprsSettings.h"

void asi_begin();
void asi_loop();


bool cadGet();
void asi_forceGetAllRadioSettings();
PFListaSettings asi_pfSettings();
LoraListadeSettings asi_loraSettings();
GprsListadeSettings asi_gprsSettings();


#endif