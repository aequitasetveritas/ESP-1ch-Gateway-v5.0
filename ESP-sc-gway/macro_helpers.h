#ifndef MACRO_HELPERS
#define MACRO_HELPERS

#define MAC 0x6884


#if MAC == 0x00fc
#define TENSION_CAL (5.206)
#define VELOCIDAD_MIN (186)
#define VELOCIDAD_MAX (941)
#define DIRECCION_MIN (186)
#define DIRECCION_MAX (941)
#elif MAC == 0x6884
#define TENSION_CAL (4.277)
#define VELOCIDAD_MIN (186)
#define VELOCIDAD_MAX (941)
#define DIRECCION_MIN (186)
#define DIRECCION_MAX (941)
#endif

//#define SerialOut 0

#ifdef SerialOut
#define dbgp(...) Serial.print(__VA_ARGS__)
#define dbgpl(...) Serial.println(__VA_ARGS__)
#else
#define dbgp(...)
#define dbgpl(...)
#endif

#endif