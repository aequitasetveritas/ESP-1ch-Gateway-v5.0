#ifndef MACRO_HELPERS
#define MACRO_HELPERS

#define SerialOut 0

#ifdef SerialOut
#define dbgp(...) Serial.print(__VA_ARGS__)
#define dbgpl(...) Serial.println(__VA_ARGS__)
#else
#define dbgp(...)
#define dbgpl(...)
#endif

#endif