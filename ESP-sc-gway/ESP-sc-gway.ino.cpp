# 1 "/tmp/tmpu7vdsk15"
#include <Arduino.h>
# 1 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
# 29 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
#include "ESP-sc-gway.h"

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#define ESP32_ARCH 1
#endif

#include <Esp.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>
#include <string>

#include <SPI.h>
#include <TimeLib.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <WiFiUdp.h>
#include <pins_arduino.h>


#include "_loraModem.h"
#include "loraFiles.h"




#include "asi-src/src/asi.h"
#include "utils.h"
#include <pb_common.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include "protobuf/gw.pb.h"

#include <PubSubClient.h>


#include "settings.h"

extern "C"
{
#include "lwip/err.h"
#include "lwip/dns.h"
}

#if WIFIMANAGER == 1
#include <WiFiManager.h>
#endif

#if (GATEWAYNODE == 1) || (_LOCALSERVER == 1)
#include "AES-128_V10.h"
#endif


#if ESP32_ARCH == 1

#include "WiFi.h"
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#if A_SERVER == 1
#include <ESP32WebServer.h>
#include <Streaming.h>
#endif
#if A_OTA == 1
#include <ESP32httpUpdate.h>
#include <ArduinoOTA.h>
#endif


#else

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
extern "C"
{
#include "user_interface.h"
#include "c_types.h"
}
#if A_SERVER == 1
#include <ESP8266WebServer.h>
#include <Streaming.h>
#endif
#if A_OTA == 1
#include <ESP8266httpUpdate.h>
#include <ArduinoOTA.h>
#endif

#endif


uint8_t debug = 1;
uint8_t pdebug = 0xFF;

#if GATEWAYNODE == 1
#if _GPS == 1
#include <TinyGPS++.h>
TinyGPSPlus gps;
HardwareSerial Serial1(1);
#endif
#endif


#if A_SERVER == 1
#if ESP32_ARCH == 1
ESP32WebServer server(A_SERVERPORT);
#else
ESP8266WebServer server(A_SERVERPORT);
#endif
#endif
using namespace std;

wpas wpa[] = {
 {"", ""},
 {"que_miras_mi_wifi", "1008572055"},
 {"ape", "beer"}};

byte currentMode = 0x81;

bool sx1272 = true;

uint32_t cp_nb_rx_rcv;
uint32_t cp_nb_rx_ok;
uint32_t cp_nb_rx_bad;
uint32_t cp_nb_rx_nocrc;
uint32_t cp_up_pkt_fwd;

uint8_t MAC_array[6];
# 171 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
sf_t global_sf = _SPREADING;
sf_t sfi = _SPREADING;




float lat = _LAT;
float lon = _LON;
int alt = _ALT;
char platform[24] = _PLATFORM;
char email[40] = _EMAIL;
char description[64] = _DESCRIPTION;



IPAddress ntpServer;
IPAddress ttnServer;
IPAddress thingServer;

WiFiUDP Udp;

time_t startTime = 0;


uint32_t eventTime = 0;
uint32_t sendTime = 0;
uint32_t doneTime = 0;
uint32_t statTime = 0;
uint32_t pulltime = 0;


#if A_SERVER == 1
uint32_t wwwtime = 0;
#endif
#if NTP_INTR == 0
uint32_t ntptimer = 0;
#endif

#define TX_BUFF_SIZE 1024
#define RX_BUFF_SIZE 1024
#define STATUS_SIZE 512

#if GATEWAYNODE == 1
uint16_t frameCount = 0;
#endif




int mutexSPI = 1;

WiFiClient espClient;
PubSubClient mqtt_client(espClient);
# 234 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
void ICACHE_RAM_ATTR Interrupt_0();
void ICACHE_RAM_ATTR Interrupt_1();

void mqtt_reconnect();
void mqtt_callback(char *topic, byte *payload, unsigned int length);
int sendPacket(uint8_t *buf, uint8_t length);
void setupWWW();
void SerialTime();
static void printIP(IPAddress ipa, const char sep, String &response);

void init_oLED();
void acti_oLED();
void addr_oLED();

void setupOta(char *hostname);
void initLoraModem();
void cadScanner();
void rxLoraModem();
void writeRegister(uint8_t addr, uint8_t value);

void stateMachine();

void SerialStat(uint8_t intr);

#if MUTEX == 1

void ICACHE_FLASH_ATTR CreateMutux(int *mutex);
bool ICACHE_FLASH_ATTR GetMutex(int *mutex);
void ICACHE_FLASH_ATTR ReleaseMutex(int *mutex);
#endif
void printDigits(unsigned long digits);
void printHexDigit(uint8_t digit);
static void printTime();
void ftoa(float f, char *val, int p);
int sendNtpRequest(IPAddress timeServerIP);
time_t getNtpTime();
void setupTime();
int readUdp(int packetSize);
int sendUdp(IPAddress server, int port, uint8_t *msg, int length);
bool UDPconnect();
void pullData();
void sendstat();
void setup();
void loop();
void lora_settings_reconfig(int sf, int bw, uint32_t frec);
bool decode_string(pb_istream_t *stream, const pb_field_t *field, void **arg);
bool pb_phy_payload_decode(pb_istream_t *stream, const pb_field_t *field, void **arg);
bool pb_timing_context_decode(pb_istream_t *stream, const pb_field_t *field, void **arg);
bool pb_downlink_id_decode(pb_istream_t *stream, const pb_field_t *field, void **arg);
bool pb_set_gateway_id(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);
bool pb_set_ip(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);
bool pb_set_config_version(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);
bool pb_set_code_rate(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);
bool pb_set_payload(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);
bool pb_set_timing_context(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);
bool pb_set_downlink_id(pb_ostream_t *stream, const pb_field_t *field, void *const *arg);
void mqtt_sendStat();
void mqtt_sendUplink(struct LoraUp up_packet);
void mqtt_send_ack(uint8_t * downId);
int WlanStatus();
int WlanReadWpa();
int WlanWriteWpa( char* ssid, char *pass);
int WlanConnect(int maxTry);
void gateway_mgt(uint8_t size, uint8_t *buff);
void id_print (String id, String val);
int initConfig(struct espGwayConfig *c);
int readConfig(const char *fn, struct espGwayConfig *c);
int writeGwayCfg(const char *fn);
int writeConfig(const char *fn, struct espGwayConfig *c);
void addLog(const unsigned char * line, int cnt);
void printLog();
void listDir(char * dir);
void msg_oLED(String tim, String sf);
void updateOtaa();
int sendLora(char *msg, int len);
static void smartDelay(unsigned long ms);
static int LoRaSensors(uint8_t *buf);
static void mXor(uint8_t *buf, uint8_t *key);
static void shift_left(uint8_t * buf, uint8_t len);
static void generate_subkey(uint8_t *key, uint8_t *k1, uint8_t *k2);
uint8_t micPacket(uint8_t *data, uint8_t len, uint16_t FrameCount, uint8_t * NwkSKey, uint8_t dir);
static void checkMic(uint8_t *buf, uint8_t len, uint8_t *key);
int sensorPacket();
uint8_t encodePacket(uint8_t *Data, uint8_t DataLength, uint16_t FrameCount, uint8_t *DevAddr, uint8_t *AppSKey, uint8_t Direction);
void fakeLoraWanWrap(uint8_t *message, uint8_t *messageLen);
int buildPacket(uint32_t tmst, uint8_t *buff_up, struct LoraUp LoraUp, bool internal);
int receivePacket();
uint8_t micPacket(uint8_t *data, uint8_t len, uint16_t FrameCount, uint8_t * DevAddr, uint8_t * NwkSKey, uint8_t dir);
uint8_t encodePacket(uint8_t *Data, uint8_t DataLength, uint16_t FrameCount, uint8_t *DevAddr, uint8_t *AppSKey, uint8_t Direction);
static void generate_subkey(uint8_t *key, uint8_t *k1, uint8_t *k2);
static void mXor(uint8_t *buf, uint8_t *key);
static void shift_left(uint8_t * buf, uint8_t len);
static void printIP(IPAddress ipa, const char sep, String& response);
boolean YesNo();
void wwwFile(String fn);
void buttonDocu();
void buttonLog();
static void wwwButtons();
static void setVariables(const char *cmd, const char *arg);
static void openWebPage();
static void settingsData();
static void statisticsData();
static void sensorData();
void sendWebPage(const char *cmd, const char *arg);
static void wifiData();
static void systemData();
static void interruptData();
#line 269 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
void printDigits(unsigned long digits)
{

 if (digits < 10)
  Serial.print(F("0"));
 Serial.print(digits);
}




void printHexDigit(uint8_t digit)
{

 if (digit < 0x10)
  Serial.print('0');
 Serial.print(digit, HEX);
}




static void printTime()
{
 switch (weekday())
 {
 case 1:
  Serial.print(F("Sunday"));
  break;
 case 2:
  Serial.print(F("Monday"));
  break;
 case 3:
  Serial.print(F("Tuesday"));
  break;
 case 4:
  Serial.print(F("Wednesday"));
  break;
 case 5:
  Serial.print(F("Thursday"));
  break;
 case 6:
  Serial.print(F("Friday"));
  break;
 case 7:
  Serial.print(F("Saturday"));
  break;
 default:
  Serial.print(F("ERROR"));
  break;
 }
 Serial.print(F(" "));
 printDigits(hour());
 Serial.print(F(":"));
 printDigits(minute());
 Serial.print(F(":"));
 printDigits(second());
 return;
}
# 336 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
void ftoa(float f, char *val, int p)
{

 int int_part = (int)f;
 float dec_part = f - (float)int_part;

 int int_dec_part = dec_part * (10);
 sprintf(val, "%d.%d00000", int_part, int_dec_part);
# 373 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
}
# 382 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
int sendNtpRequest(IPAddress timeServerIP)
{
 const int NTP_PACKET_SIZE = 48;
 byte packetBuffer[NTP_PACKET_SIZE];

 memset(packetBuffer, 0, NTP_PACKET_SIZE);

 packetBuffer[0] = 0b11100011;
 packetBuffer[1] = 0;
 packetBuffer[2] = 6;
 packetBuffer[3] = 0xEC;

 packetBuffer[12] = 49;
 packetBuffer[13] = 0x4E;
 packetBuffer[14] = 49;
 packetBuffer[15] = 52;

 if (!sendUdp((IPAddress)timeServerIP, (int)123, packetBuffer, NTP_PACKET_SIZE))
 {
  gwayConfig.ntpErr++;
  gwayConfig.ntpErrTime = now();
  return (0);
 }
 return (1);
}






time_t getNtpTime()
{
 gwayConfig.ntps++;

 if (!sendNtpRequest(ntpServer))
 {
  if ((debug >= 0) && (pdebug & P_MAIN))
   Serial.println(F("M sendNtpRequest failed"));
  return (0);
 }

 const int NTP_PACKET_SIZE = 48;
 byte packetBuffer[NTP_PACKET_SIZE];
 memset(packetBuffer, 0, NTP_PACKET_SIZE);

 uint32_t beginWait = millis();
 delay(10);
 while (millis() - beginWait < 1500)
 {
  int size = Udp.parsePacket();
  if (size >= NTP_PACKET_SIZE)
  {

   if (Udp.read(packetBuffer, NTP_PACKET_SIZE) < NTP_PACKET_SIZE)
   {
    break;
   }
   else
   {

    unsigned long secs;
    secs = packetBuffer[40] << 24;
    secs |= packetBuffer[41] << 16;
    secs |= packetBuffer[42] << 8;
    secs |= packetBuffer[43];

    return (secs - 2208988800UL + NTP_TIMEZONES * SECS_IN_HOUR);
   }
   Udp.flush();
  }
  delay(100);
 }

 Udp.flush();



 gwayConfig.ntpErr++;
 gwayConfig.ntpErrTime = now();
#if DUSB >= 1
 if ((debug >= 0) && (pdebug & P_MAIN))
 {
  Serial.println(F("M getNtpTime:: read failed"));
 }
#endif
 return (0);
}




#if NTP_INTR == 1
void setupTime()
{
 setSyncProvider(getNtpTime);
 setSyncInterval(_NTP_INTERVAL);
}
#endif
# 499 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
int readUdp(int packetSize)
{
 uint8_t protocol;
 uint16_t token;
 uint8_t ident;
 uint8_t buff[32];
 uint8_t buff_down[RX_BUFF_SIZE];


 if (WlanConnect(10) < 0)
 {
#if DUSB >= 1
  Serial.print(F("readdUdp: ERROR connecting to WLAN"));
  if (debug >= 2)
   Serial.flush();
#endif
  Udp.flush();
  yield();
  return (-1);
 }

 yield();

 if (packetSize > RX_BUFF_SIZE)
 {
#if DUSB >= 1
  Serial.print(F("readUDP:: ERROR package of size: "));
  Serial.println(packetSize);
#endif
  Udp.flush();
  return (-1);
 }



 if (Udp.read(buff_down, packetSize) < packetSize)
 {
#if DUSB >= 1
  Serial.println(F("A readUsb:: Reading less chars"));
  return (-1);
#endif
 }


 IPAddress remoteIpNo = Udp.remoteIP();


 unsigned int remotePortNo = Udp.remotePort();

 if (remotePortNo == 123)
 {

#if DUSB >= 1
  if (debug >= 0)
  {
   Serial.println(F("A readUdp:: NTP msg rcvd"));
  }
#endif
  gwayConfig.ntpErr++;
  gwayConfig.ntpErrTime = now();
  return (0);
 }


 else
 {
  uint8_t *data = (uint8_t *)((uint8_t *)buff_down + 4);
  protocol = buff_down[0];
  token = buff_down[2] * 256 + buff_down[1];
  ident = buff_down[3];

#if DUSB >= 1
  if ((debug > 1) && (pdebug & P_MAIN))
  {
   Serial.print(F("M readUdp:: message waiting="));
   Serial.print(ident);
   Serial.println();
  }
#endif

  switch (ident)
  {




  case PKT_PUSH_DATA:
#if DUSB >= 1
   if (debug >= 1)
   {
    Serial.print(F("PKT_PUSH_DATA:: size "));
    Serial.print(packetSize);
    Serial.print(F(" From "));
    Serial.print(remoteIpNo);
    Serial.print(F(", port "));
    Serial.print(remotePortNo);
    Serial.print(F(", data: "));
    for (int i = 0; i < packetSize; i++)
    {
     Serial.print(buff_down[i], HEX);
     Serial.print(':');
    }
    Serial.println();
    if (debug >= 2)
     Serial.flush();
   }
#endif
   break;



  case PKT_PUSH_ACK:
#if DUSB >= 1
   if ((debug >= 2) && (pdebug & P_MAIN))
   {
    Serial.print(F("M PKT_PUSH_ACK:: size "));
    Serial.print(packetSize);
    Serial.print(F(" From "));
    Serial.print(remoteIpNo);
    Serial.print(F(", port "));
    Serial.print(remotePortNo);
    Serial.print(F(", token: "));
    Serial.println(token, HEX);
    Serial.println();
   }
#endif
   break;

  case PKT_PULL_DATA:
#if DUSB >= 1
   Serial.print(F(" Pull Data"));
   Serial.println();
#endif
   break;



  case PKT_PULL_RESP:
#if DUSB >= 1
   if ((debug >= 0) && (pdebug & P_MAIN))
   {
    Serial.println(F("M readUdp:: PKT_PULL_RESP received"));
   }
#endif



   _state = S_TX;
   sendTime = micros();


   if (sendPacket(data, packetSize - 4) < 0)
   {
#if DUSB >= 1
    if (debug >= 0)
    {
     Serial.println(F("A readUdp:: Error: PKT_PULL_RESP sendPacket failed"));
    }
#endif
    return (-1);
   }


   buff[0] = buff_down[0];
   buff[1] = buff_down[1];
   buff[2] = buff_down[2];

   buff[3] = PKT_TX_ACK;
   buff[4] = MAC_array[0];
   buff[5] = MAC_array[1];
   buff[6] = MAC_array[2];
   buff[7] = 0xFF;
   buff[8] = 0xFF;
   buff[9] = MAC_array[3];
   buff[10] = MAC_array[4];
   buff[11] = MAC_array[5];
   buff[12] = 0;
#if DUSB >= 1
   if ((debug >= 2) && (pdebug & P_MAIN))
   {
    Serial.println(F("M readUdp:: TX buff filled"));
   }
#endif

   Udp.beginPacket(remoteIpNo, remotePortNo);
   if (Udp.write((unsigned char *)buff, 12) != 12)
   {
#if DUSB >= 1
    if (debug >= 0)
     Serial.println("A readUdp:: Error: PKT_PULL_ACK UDP write");
#endif
   }
   else
   {
#if DUSB >= 1
    if ((debug >= 0) && (pdebug & P_TX))
    {
     Serial.print(F("M PKT_TX_ACK:: micros="));
     Serial.println(micros());
    }
#endif
   }

   if (!Udp.endPacket())
   {
#if DUSB >= 1
    if ((debug >= 0) && (pdebug & P_MAIN))
    {
     Serial.println(F("M PKT_PULL_DATALL Error Udp.endpaket"));
    }
#endif
   }

   yield();
#if DUSB >= 1
   if ((debug >= 1) && (pdebug & P_MAIN))
   {
    Serial.print(F("M PKT_PULL_RESP:: size "));
    Serial.print(packetSize);
    Serial.print(F(" From "));
    Serial.print(remoteIpNo);
    Serial.print(F(", port "));
    Serial.print(remotePortNo);
    Serial.print(F(", data: "));
    data = buff_down + 4;
    data[packetSize] = 0;
    Serial.print((char *)data);
    Serial.println(F("..."));
   }
#endif
   break;

  case PKT_PULL_ACK:
#if DUSB >= 1
   if ((debug >= 2) && (pdebug & P_MAIN))
   {
    Serial.print(F("M PKT_PULL_ACK:: size "));
    Serial.print(packetSize);
    Serial.print(F(" From "));
    Serial.print(remoteIpNo);
    Serial.print(F(", port "));
    Serial.print(remotePortNo);
    Serial.print(F(", data: "));
    for (int i = 0; i < packetSize; i++)
    {
     Serial.print(buff_down[i], HEX);
     Serial.print(':');
    }
    Serial.println();
   }
#endif
   break;

  default:
#if GATEWAYMGT == 1

   gateway_mgt(packetSize, buff_down);
#else

#endif
#if DUSB >= 1
   Serial.print(F(", ERROR ident not recognized="));
   Serial.println(ident);
#endif
   break;
  }
#if DUSB >= 2
  if (debug >= 1)
  {
   Serial.print(F("readUdp:: returning="));
   Serial.println(packetSize);
  }
#endif

  return packetSize;
 }
}
# 790 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
int sendUdp(IPAddress server, int port, uint8_t *msg, int length)
{


 if (WlanConnect(3) < 0)
 {
#if DUSB >= 1
  if ((debug >= 0) && (pdebug & P_MAIN))
  {
   Serial.print(F("M sendUdp: ERROR connecting to WiFi"));
   Serial.flush();
  }
#endif
  Udp.flush();
  yield();
  return (0);
 }

 yield();


#if DUSB >= 1
 if ((debug >= 3) && (pdebug & P_MAIN))
 {
  Serial.println(F("M WiFi connected"));
 }
#endif
 if (!Udp.beginPacket(server, (int)port))
 {
#if DUSB >= 1
  if ((debug >= 1) && (pdebug & P_MAIN))
  {
   Serial.println(F("M sendUdp:: Error Udp.beginPacket"));
  }
#endif
  return (0);
 }

 yield();

 if (Udp.write((unsigned char *)msg, length) != length)
 {
#if DUSB >= 1
  if ((debug <= 1) && (pdebug & P_MAIN))
  {
   Serial.println(F("M sendUdp:: Error write"));
  }
#endif
  Udp.endPacket();
  return (0);
 }

 yield();

 if (!Udp.endPacket())
 {
#if DUSB >= 1
  if (debug >= 1)
  {
   Serial.println(F("sendUdp:: Error Udp.endPacket"));
   Serial.flush();
  }
#endif
  return (0);
 }
 return (1);
}
# 866 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
bool UDPconnect()
{

 bool ret = false;
 unsigned int localPort = _LOCUDPPORT;
#if DUSB >= 1
 if (debug >= 1)
 {
  Serial.print(F("Local UDP port="));
  Serial.println(localPort);
 }
#endif
 if (Udp.begin(localPort) == 1)
 {
#if DUSB >= 1
  if (debug >= 1)
   Serial.println(F("Connection successful"));
#endif
  ret = true;
 }
 else
 {
#if DUSB >= 1
  if (debug >= 1)
   Serial.println("Connection failed");
#endif
 }
 return (ret);
}
# 905 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
void pullData()
{

 uint8_t pullDataReq[12];
 int pullIndex = 0;
 int i;

 uint8_t token_h = (uint8_t)rand();
 uint8_t token_l = (uint8_t)rand();


 pullDataReq[0] = PROTOCOL_VERSION;
 pullDataReq[1] = token_h;
 pullDataReq[2] = token_l;
 pullDataReq[3] = PKT_PULL_DATA;

 pullDataReq[4] = MAC_array[0];
 pullDataReq[5] = MAC_array[1];
 pullDataReq[6] = MAC_array[2];
 pullDataReq[7] = 0xFF;
 pullDataReq[8] = 0xFF;
 pullDataReq[9] = MAC_array[3];
 pullDataReq[10] = MAC_array[4];
 pullDataReq[11] = MAC_array[5];


 pullIndex = 12;



 uint8_t *pullPtr;
 pullPtr = pullDataReq,
#ifdef _TTNSERVER
 sendUdp(ttnServer, _TTNPORT, pullDataReq, pullIndex);
 yield();
#endif

#if DUSB >= 1
 if (pullPtr != pullDataReq)
 {
  Serial.println(F("pullPtr != pullDatReq"));
  Serial.flush();
 }

#endif
#ifdef _THINGSERVER
 sendUdp(thingServer, _THINGPORT, pullDataReq, pullIndex);
#endif

#if DUSB >= 1
 if ((debug >= 2) && (pdebug & P_MAIN))
 {
  yield();
  Serial.print(F("M PKT_PULL_DATA request, len=<"));
  Serial.print(pullIndex);
  Serial.print(F("> "));
  for (i = 0; i < pullIndex; i++)
  {
   Serial.print(pullDataReq[i], HEX);
   Serial.print(':');
  }
  Serial.println();
  if (debug >= 2)
   Serial.flush();
 }
#endif
 return;
}







void sendstat()
{

 uint8_t status_report[STATUS_SIZE];
 char stat_timestamp[32];
 time_t t;
 char clat[10] = {0};
 char clon[10] = {0};

 int stat_index = 0;
 uint8_t token_h = (uint8_t)rand();
 uint8_t token_l = (uint8_t)rand();


 status_report[0] = PROTOCOL_VERSION;
 status_report[1] = token_h;
 status_report[2] = token_l;
 status_report[3] = PKT_PUSH_DATA;


 status_report[4] = MAC_array[0];
 status_report[5] = MAC_array[1];
 status_report[6] = MAC_array[2];
 status_report[7] = 0xFF;
 status_report[8] = 0xFF;
 status_report[9] = MAC_array[3];
 status_report[10] = MAC_array[4];
 status_report[11] = MAC_array[5];

 stat_index = 12;

 t = now();


 sprintf(stat_timestamp, "%04d-%02d-%02d %02d:%02d:%02d CET", year(), month(), day(), hour(), minute(), second());
 yield();

 ftoa(lat, clat, 5);
 ftoa(lon, clon, 5);


 delay(1);

 int j = snprintf((char *)(status_report + stat_index), STATUS_SIZE - stat_index,
      "{\"stat\":{\"time\":\"%s\",\"lati\":%s,\"long\":%s,\"alti\":%i,\"rxnb\":%u,\"rxok\":%u,\"rxfw\":%u,\"ackr\":%u.0,\"dwnb\":%u,\"txnb\":%u,\"pfrm\":\"%s\",\"mail\":\"%s\",\"desc\":\"%s\"}}",
      stat_timestamp, clat, clon, (int)alt, cp_nb_rx_rcv, cp_nb_rx_ok, cp_up_pkt_fwd, 0, 0, 0, platform, email, description);

 yield();

 stat_index += j;
 status_report[stat_index] = 0;

#if DUSB >= 1
 if ((debug >= 2) && (pdebug & P_MAIN))
 {
  Serial.print(F("M stat update: <"));
  Serial.print(stat_index);
  Serial.print(F("> "));
  Serial.println((char *)(status_report + 12));
 }
#endif
 if (stat_index > STATUS_SIZE)
 {
#if DUSB >= 1
  Serial.println(F("A sendstat:: ERROR buffer too big"));
#endif
  return;
 }


#ifdef _TTNSERVER
 sendUdp(ttnServer, _TTNPORT, status_report, stat_index);
 yield();
#endif

#ifdef _THINGSERVER
 sendUdp(thingServer, _THINGPORT, status_report, stat_index);
#endif
 return;
}
# 1068 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
void setup()
{

 char MAC_char[19];
 MAC_char[18] = 0;

 Serial.begin(_BAUDRATE);
 delay(100);

#if _GPS == 1

 Serial1.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
#endif

#ifdef ESP32
#if DUSB >= 1
 Serial.print(F("ESP32 defined, freq="));
#if _LFREQ == 433
 Serial.print(freqs[0]);
 Serial.print(F(" EU433"));
#elif _LFREQ == 868
 Serial.print(freqs[0]);
 Serial.print(F(" EU868"));
#endif
 Serial.println();
#endif
#endif
#ifdef ARDUINO_ARCH_ESP32
#if DUSB >= 1
 Serial.println(F("ARDUINO_ARCH_ESP32 defined"));
#endif
#endif

#if DUSB >= 1
 Serial.flush();

 delay(500);

 if (SPIFFS.begin())
 {
  Serial.println(F("SPIFFS init success"));
 }
 else
 {
 }
#endif
#if _SPIFF_FORMAT >= 1
#if DUSB >= 1
 if ((debug >= 0) && (pdebug & P_MAIN))
 {
  Serial.println(F("M Format Filesystem ... "));
 }
#endif
 SPIFFS.format();
#if DUSB >= 1
 if ((debug >= 0) && (pdebug & P_MAIN))
 {
  Serial.println(F("Done"));
 }
#endif
#endif

 Serial.print(F("Assert="));
#if defined CFG_noassert
 Serial.println(F("No Asserts"));
#else
 Serial.println(F("Do Asserts"));
#endif

#if OLED >= 1
 init_oLED();
#endif

 delay(500);
 yield();
#if DUSB >= 1
 if (debug >= 1)
 {
  Serial.print(F("debug="));
  Serial.println(debug);
  yield();
 }
#endif







 WiFi.macAddress(MAC_array);

 sprintf(MAC_char, "%02x:%02x:%02x:%02x:%02x:%02x",
   MAC_array[0], MAC_array[1], MAC_array[2], MAC_array[3], MAC_array[4], MAC_array[5]);
 Serial.print("MAC: ");
 Serial.print(MAC_char);
 Serial.print(F(", len="));
 Serial.println(strlen(MAC_char));


 char hostname[12];
# 1199 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
 asi_begin();
 asi_loop();
 asi_forceGetAllRadioSettings();

 int8_t res = WiFi.waitForConnectResult();
 if (res != WL_CONNECTED)
 {
  Serial.println("No conectado");
 }


 if (!UDPconnect())
 {
  Serial.println(F("Error UDPconnect"));
 }
 delay(200);


 pinMode(pins.ss, OUTPUT);
 digitalWrite(pins.ss, HIGH);
 pinMode(pins.rst, OUTPUT);
 pinMode(pins.dio0, INPUT);
 pinMode(pins.dio1, INPUT);



#if ESP32_ARCH == 1
 SPI.begin(SCK, MISO, MOSI, SS);
#else
 SPI.begin();
#endif

 delay(500);




 Serial.print("Gateway ID: ");
 printHexDigit(MAC_array[0]);
 printHexDigit(MAC_array[1]);
 printHexDigit(MAC_array[2]);
 printHexDigit(0xFF);
 printHexDigit(0xFF);
 printHexDigit(MAC_array[3]);
 printHexDigit(MAC_array[4]);
 printHexDigit(MAC_array[5]);

 Serial.print(", Listening at SF");
 Serial.print(global_sf);
 Serial.print(" on ");
 Serial.print((double)freq / 1000000);
 Serial.println(" Mhz.");

 if (!WiFi.hostByName(NTP_TIMESERVER, ntpServer))
 {
  die("Setup:: ERROR hostByName NTP");
 };
 delay(100);
#ifdef _TTNSERVER
 if (!WiFi.hostByName(_TTNSERVER, ttnServer))
 {
  die("Setup:: ERROR hostByName TTN");
 };
 delay(100);
#endif
#ifdef _THINGSERVER
 if (!WiFi.hostByName(_THINGSERVER, thingServer))
 {
  die("Setup:: ERROR hostByName THING");
 }
 delay(100);
#endif



#if A_OTA == 1
 setupOta(hostname);
#endif



#if NTP_INTR == 1
 setupTime();
#else





 while (timeStatus() == timeNotSet)
 {
#if DUSB >= 1
  if ((debug >= 0) && (pdebug & P_MAIN))
   Serial.println(F("M setupTime:: Time not set (yet)"));
#endif
  delay(500);
  time_t newTime;
  newTime = (time_t)getNtpTime();
  if (newTime != 0)
   setTime(newTime);
 }

 startTime = now();
#if DUSB >= 1
 Serial.print("Time: ");
 printTime();
 Serial.println();
#endif
 writeGwayCfg(CONFIGFILE);
#if DUSB >= 1
 Serial.println(F("Gateway configuration saved"));
#endif
#endif

#if A_SERVER == 1

 setupWWW();
#endif

 delay(100);


 _state = S_INIT;
 initLoraModem();

 if (cadGet())
 {
  _state = S_SCAN;
  global_sf = SF7;
  cadScanner();
 }
 else
 {
  _state = S_RX;
  rxLoraModem();
 }
 LoraUp.payLoad[0] = 0;
 LoraUp.payLength = 0;



 if (pins.dio0 == pins.dio1)
 {

  attachInterrupt(pins.dio0, Interrupt_0, RISING);
 }

 else
 {


  attachInterrupt(pins.dio0, Interrupt_0, RISING);
  attachInterrupt(pins.dio1, Interrupt_1, RISING);
 }

 writeConfig(CONFIGFILE, &gwayConfig);


#if OLED >= 1
 acti_oLED();
 addr_oLED();
#endif

 mqtt_client.setServer(settings_mqtt_server(), 1883);
 mqtt_client.setCallback(mqtt_callback);
 Serial.println(F("--------------------------------------"));
}
# 1383 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
volatile uint8_t mqtt_down_flag = 0;

void loop()
{
 uint32_t uSeconds;
 int packetSize;
 uint32_t nowSeconds = now();
 S_PROTOCOL protocolo = settings_protocol();

 asi_loop();




 if (protocolo == MQTTBRIDGE_TCP)
 {

  if (mqtt_down_flag == 1)
  {
   Serial.print("mqtt_down ");
   Serial.println(_state);
   mqtt_down_flag = 0;
   _state = S_TX;
  }

 }

 stateMachine();







 if (((nowSeconds - statr[0].tmst) > _MSG_INTERVAL) &&
  (msgTime <= statr[0].tmst))
 {
#if DUSB >= 1
  if ((debug >= 1) && (pdebug & P_MAIN))
  {
   Serial.print("M REINIT:: ");
   Serial.print(_MSG_INTERVAL);
   Serial.print(F(" "));
   SerialStat(0);
  }
#endif


  if ((cadGet()) || (_hop))
  {
   _state = S_SCAN;
   global_sf = SF7;
   cadScanner();
  }
  else
  {
   _state = S_RX;
   rxLoraModem();
  }
  writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t)0x00);
  writeRegister(REG_IRQ_FLAGS, (uint8_t)0xFF);
  msgTime = nowSeconds;
 }

#if A_SERVER == 1



 yield();
 server.handleClient();
#endif






 if (_event == 1)
 {
  return;
 }
 else
  yield();





 if (WlanConnect(1) < 0)
 {
#if DUSB >= 1
  if ((debug >= 0) && (pdebug & P_MAIN))
   Serial.println(F("M ERROR reconnect WLAN"));
#endif
  yield();
  return;
 }
 else
 {







  if (protocolo == SEMTECH_PF_UDP)
  {
   while ((packetSize = Udp.parsePacket()) > 0)
   {
#if DUSB >= 2
    Serial.println(F("loop:: readUdp calling"));
#endif



    if (readUdp(packetSize) <= 0)
    {
#if DUSB >= 1
     if ((debug > 0) && (pdebug & P_MAIN))
      Serial.println(F("M readUDP error"));
#endif
     break;
    }

    else
    {

    }
   }
  }
 }



 yield();




 if ((nowSeconds - statTime) >= _STAT_INTERVAL)
 {
#if DUSB >= 1
  if ((debug >= 1) && (pdebug & P_MAIN))
  {
   Serial.print(F("M STAT:: ..."));
   Serial.flush();
  }
#endif
  if (protocolo == SEMTECH_PF_UDP)
  {
   sendstat();
  }
  else if (protocolo == MQTTBRIDGE_TCP)
  {
   mqtt_sendStat();
  }

#if DUSB >= 1
  if ((debug >= 1) && (pdebug & P_MAIN))
  {
   Serial.println(F(" done"));
   if (debug >= 2)
    Serial.flush();
  }
#endif






#if GATEWAYNODE == 1
  if (gwayConfig.isNode)
  {

   yield();




   if (sensorPacket() < 0)
   {
#if DUSB >= 1
    Serial.println(F("sensorPacket: Error"));
#endif
   }
  }
#endif
  statTime = nowSeconds;
 }

 yield();

 if (protocolo == SEMTECH_PF_UDP)
 {



  nowSeconds = now();
  if ((nowSeconds - pulltime) >= _PULL_INTERVAL)
  {
#if DUSB >= 1
   if ((debug >= 2) && (pdebug & P_MAIN))
   {
    Serial.println(F("M PULL"));
    if (debug >= 1)
     Serial.flush();
   }
#endif
   pullData();
   startReceiver();
   pulltime = nowSeconds;
  }
 }
 else
 {

 }




#if NTP_INTR == 0


 yield();
 nowSeconds = now();
 if (nowSeconds - ntptimer >= _NTP_INTERVAL)
 {
  yield();
  time_t newTime;
  newTime = (time_t)getNtpTime();
  if (newTime != 0)
   setTime(newTime);
  ntptimer = nowSeconds;
 }
#endif

 if (protocolo == MQTTBRIDGE_TCP)
 {
  if (!mqtt_client.connected())
  {
   mqtt_reconnect();
  }
  mqtt_client.loop();
 }

}

void lora_settings_reconfig(int sf, int bw, uint32_t frec)
{
 global_sf = (sf_t)sf;
 freq = frec;
 uint8_t i;
 for (i = 0; i < 10; i++)
 {
  if (freqs[i] == freq)
  {
   break;
  }
 }
 ifreq = i;
 rxLoraModem();

 Serial.println("Reconfig LORA");
}

bool decode_string(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
 uint8_t buffer[1024] = {0};
 char *destino = (char *)*arg;


 if (stream->bytes_left > sizeof(buffer) - 1)
  return false;

 if (!pb_read(stream, (pb_byte_t *)destino, stream->bytes_left))
  return false;

 return true;
}

bool pb_phy_payload_decode(pb_istream_t *stream, const pb_field_t *field, void **arg)
{



 if (stream->bytes_left > sizeof(payLoad) - 1)
  return false;

 LoraDown.payLength = stream->bytes_left;

 if (!pb_read(stream, (pb_byte_t *)payLoad, stream->bytes_left))
  return false;

 return true;
}

bool pb_timing_context_decode(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
 uint8_t buffer[1024] = {0};
 uint32_t *destino = (uint32_t *)*arg;


 if (stream->bytes_left > sizeof(buffer) - 1)
  return false;

 if (!pb_read(stream, (pb_byte_t *)destino, stream->bytes_left))
  return false;

 return true;
}



uint8_t downlink_size;

bool pb_downlink_id_decode(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
 uint8_t buffer[1024] = {0};
 uint32_t *destino = (uint32_t *)*arg;


 if (stream->bytes_left > 50 - 1)
  return false;

 downlink_size = stream->bytes_left;

 if (!pb_read(stream, (pb_byte_t *)destino, stream->bytes_left))
  return false;

 return true;
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{

 Serial.print("Message arrived [");
 Serial.print(topic);
 Serial.print("] Length ");
 Serial.print(length);
 Serial.println();

 gw_DownlinkFrame dm = gw_DownlinkFrame_init_zero;

 uint32_t uplink_timing_context;
 dm.tx_info.context.arg = &uplink_timing_context;
 dm.tx_info.context.funcs.decode = pb_timing_context_decode;

 dm.phy_payload.funcs.decode = pb_phy_payload_decode;

 uint8_t downlink_id[50];
 dm.downlink_id.arg = downlink_id;
 dm.downlink_id.funcs.decode = pb_downlink_id_decode;

 pb_istream_t stream = pb_istream_from_buffer(payload, length);
 bool status = pb_decode(&stream, gw_DownlinkFrame_fields, &dm);
 if (!status)
 {
  Serial.print("Decoding failed: ");
  Serial.println(PB_GET_ERROR(&stream));
  return;
 }

 _state = S_TX;
 mqtt_down_flag = 1;
 sendTime = micros();

 LoraDown.bw = dm.tx_info.modulation_info.lora_modulation_info.bandwidth;
 LoraDown.sfTx = dm.tx_info.modulation_info.lora_modulation_info.spreading_factor;
 LoraDown.powe = dm.tx_info.power;

 LoraDown.tmst = dm.tx_info.timing_info.delay_timing_info.delay.seconds * 1000000 + uplink_timing_context;
 LoraDown.iiq = (dm.tx_info.modulation_info.lora_modulation_info.polarization_inversion ? 0x40 : 0x27);
 LoraDown.fff = dm.tx_info.frequency;
 LoraDown.payLoad = payLoad;

 Serial.print("bw ");
 Serial.println(LoraDown.bw);
 Serial.print("sfTx ");
 Serial.println(LoraDown.sfTx);
 Serial.print("tmst ");
 Serial.println(LoraDown.tmst);
 Serial.print("DelayS ");
 Serial.println((long)dm.tx_info.timing_info.delay_timing_info.delay.seconds);
 Serial.print("DelayN ");
 Serial.println((long)dm.tx_info.timing_info.delay_timing_info.delay.nanos);

 Serial.print("UplinkTimingContext ");
 Serial.println((unsigned long)uplink_timing_context);

 Serial.print("fff ");
 Serial.println(LoraDown.fff);
 Serial.print("length ");
 Serial.println(LoraDown.payLength);

 mqtt_send_ack(downlink_id);
# 1798 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
}

bool pb_set_gateway_id(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
 uint8_t str[8] = {MAC_array[0], MAC_array[1], MAC_array[2], 0xff, 0xff, MAC_array[3], MAC_array[4], MAC_array[5]};

 if (!pb_encode_tag_for_field(stream, field))
  return false;

 return pb_encode_string(stream, (uint8_t *)str, sizeof(str));
}

bool pb_set_ip(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
 char *str = "192.168.88.4";

 if (!pb_encode_tag_for_field(stream, field))
  return false;

 return pb_encode_string(stream, (uint8_t *)str, strlen(str));
}

bool pb_set_config_version(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
 char *str = "1.2.3";

 if (!pb_encode_tag_for_field(stream, field))
  return false;

 return pb_encode_string(stream, (uint8_t *)str, strlen(str));
}

bool pb_set_code_rate(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
 char *str = "4/5";

 if (!pb_encode_tag_for_field(stream, field))
  return false;

 return pb_encode_string(stream, (uint8_t *)str, strlen(str));
}

bool pb_set_payload(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{


 if (!pb_encode_tag_for_field(stream, field))
  return false;

 return pb_encode_string(stream, LoraUp.payLoad, LoraUp.payLength);
}

bool pb_set_timing_context(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
 uint32_t tmst = LoraUp.rx_local_timestamp;

 if (!pb_encode_tag_for_field(stream, field))
  return false;

 return pb_encode_string(stream, (uint8_t *)&tmst, sizeof(uint32_t));
}

bool pb_set_downlink_id(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{

 char *str = (char *)*arg;

 Serial.println("ID enc");
 Serial.println((uint32_t )*arg);
 for(uint8_t i=0; i<downlink_size; i++){
  if (str[i]<=0xF) Serial.print('0');
  Serial.print(str[i], HEX);
  Serial.print(' ');
 }

 if (!pb_encode_tag_for_field(stream, field))
  return false;

 return pb_encode_string(stream, (uint8_t *)str, downlink_size);
}


void mqtt_sendStat()
{
 gw_GatewayStats statsMsg = gw_GatewayStats_init_zero;

 statsMsg.gateway_id.funcs.encode = pb_set_gateway_id;
 statsMsg.ip.funcs.encode = pb_set_ip;
 statsMsg.has_time = true;
 statsMsg.time.nanos = 0;
 statsMsg.time.seconds = now();
 statsMsg.has_location = true;
 statsMsg.location.source = common_LocationSource_CONFIG;
 statsMsg.location.altitude = 0;
 statsMsg.location.latitude = -35.1684698;
 statsMsg.location.longitude = -59.0927072;

 statsMsg.config_version.funcs.encode = pb_set_config_version;
 statsMsg.rx_packets_received = 30;
 statsMsg.rx_packets_received_ok = 100;
 statsMsg.tx_packets_received = cp_up_pkt_fwd;
 statsMsg.tx_packets_emitted = cp_up_pkt_fwd;

 uint8_t buffer[120];
 pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
 if (!pb_encode(&stream, gw_GatewayStats_fields, &statsMsg))
 {
  Serial.print("Encoding failed: ");
  Serial.println(PB_GET_ERROR(&stream));
  return;
 }

 mqtt_client.publish("gateway/4c11aeffff045b23/event/stats", buffer, stream.bytes_written);
}

void mqtt_sendUplink(struct LoraUp up_packet)
{
 gw_UplinkFrame upMsg = gw_UplinkFrame_init_zero;

 upMsg.phy_payload.arg = &up_packet;
 upMsg.phy_payload.funcs.encode = pb_set_payload;
 upMsg.has_tx_info = true;
 upMsg.has_rx_info = true;
 upMsg.tx_info.frequency = freq;
 upMsg.tx_info.modulation = common_Modulation_LORA;

 upMsg.tx_info.which_modulation_info = gw_UplinkTXInfo_lora_modulation_info_tag;

 upMsg.tx_info.modulation_info.lora_modulation_info.bandwidth = 125;
 upMsg.tx_info.modulation_info.lora_modulation_info.code_rate.funcs.encode = pb_set_code_rate;
 upMsg.tx_info.modulation_info.lora_modulation_info.polarization_inversion = true;
 upMsg.tx_info.modulation_info.lora_modulation_info.spreading_factor = up_packet.sf;

 upMsg.rx_info.gateway_id.funcs.encode = pb_set_gateway_id;
 upMsg.rx_info.has_time = true;
 upMsg.rx_info.time.nanos = 0;
 upMsg.rx_info.time.seconds = now();
 upMsg.rx_info.has_time_since_gps_epoch = false;


 upMsg.rx_info.channel = 1;

 upMsg.rx_info.location.source = common_LocationSource_CONFIG;
 upMsg.rx_info.location.latitude = -35.1684698;
 upMsg.rx_info.location.longitude = -59.0927072;
 upMsg.rx_info.context.funcs.encode = pb_set_timing_context;
 upMsg.rx_info.fine_timestamp_type = gw_FineTimestampType_NONE;
 upMsg.rx_info.which_fine_timestamp = gw_UplinkRXInfo_plain_fine_timestamp_tag;

 uint8_t buffer[200];

 Serial.println("Encoding");
 pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
 if (!pb_encode(&stream, gw_UplinkFrame_fields, &upMsg))
 {
  Serial.print("Encoding failed: ");
  Serial.println(PB_GET_ERROR(&stream));
  return;
 }
 Serial.println("Publishings");
 mqtt_client.publish("gateway/4c11aeffff045b23/event/up", buffer, stream.bytes_written);
}

void mqtt_send_ack(uint8_t * downId){
 gw_DownlinkTXAck dwack = gw_DownlinkTXAck_init_zero;

 dwack.gateway_id.funcs.encode = pb_set_gateway_id;

 dwack.downlink_id.arg = downId;
 dwack.downlink_id.funcs.encode = pb_set_downlink_id;

 uint8_t buffer[100];
# 1978 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/ESP-sc-gway.ino"
 Serial.println("Encoding");
 Serial.println((uint32_t )*downId);
 pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
 if (!pb_encode(&stream, gw_DownlinkTXAck_fields, &dwack))
 {
  Serial.print("Encoding failed: ");
  Serial.println(PB_GET_ERROR(&stream));
  return;
 }
 Serial.println("PublishingsACK");
 mqtt_client.publish("gateway/4c11aeffff045b23/event/ack", buffer, stream.bytes_written);
}

void mqtt_reconnect()
{

 if (!mqtt_client.connected())
 {
  Serial.print("Attempting MQTT connection..");

  String clientId = "ESP8266Client-";
  clientId += String(random(0xffff), HEX);

  if (mqtt_client.connect(clientId.c_str(), "chirpstack_gw", ""))
  {
   Serial.println("connected ");



   Serial.print(mqtt_client.subscribe("gateway/4c11aeffff045b23/command/down"));

  }
  else
  {
   Serial.print("failed, rc=");
   Serial.print(mqtt_client.state());
   Serial.println(" try again in 5 seconds");


  }
 }
}
# 1 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_WiFi.ino"
# 33 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_WiFi.ino"
int WlanStatus() {

 switch (WiFi.status()) {
  case WL_CONNECTED:
#if DUSB>=1
   if ( debug>=0 ) {
    Serial.print(F("A WlanStatus:: CONNECTED to "));
    Serial.println(WiFi.SSID());
   }
#endif
   WiFi.setAutoReconnect(true);
   return(1);
   break;



  case WL_DISCONNECTED:
#if DUSB>=1
   if ( debug>=0 ) {
    Serial.print(F("A WlanStatus:: DISCONNECTED, IP="));
    Serial.println(WiFi.localIP());
   }
#endif


    delay(1);

   return(0);
   break;


  case WL_IDLE_STATUS:
#if DUSB>=1
   if ( debug>=0 ) {
    Serial.println(F("A WlanStatus:: IDLE"));
   }
#endif
   break;



  case WL_NO_SSID_AVAIL:
#if DUSB>=1
   if ( debug>=0 )
    Serial.println(F("WlanStatus:: NO SSID"));
#endif
   break;

  case WL_CONNECT_FAILED:
#if DUSB>=1
   if ( debug>=0 )
    Serial.println(F("A WlanStatus:: FAILED"));
#endif
   break;


  case WL_SCAN_COMPLETED:
#if DUSB>=1
   if ( debug>=0 )
    Serial.println(F("A WlanStatus:: SCAN COMPLETE"));
#endif
   break;


  case WL_CONNECTION_LOST:
#if DUSB>=1
   if ( debug>=0 )
    Serial.println(F("A WlanStatus:: LOST"));
#endif
   break;



  case WL_NO_SHIELD:
#if DUSB>=1
   if ( debug>=0 )
    Serial.println(F("A WlanStatus:: WL_NO_SHIELD"));
#endif
   break;

  default:
#if DUSB>=1
   if ( debug>=0 ) {
    Serial.print(F("A WlanStatus Error:: code="));
    Serial.println(WiFi.status());
   }
#endif
   break;
 }
 return(-1);

}






int WlanReadWpa() {

 readConfig( CONFIGFILE, &gwayConfig);

 if (gwayConfig.sf != (uint8_t) 0) global_sf = (sf_t) gwayConfig.sf;
 ifreq = gwayConfig.ch;
 debug = gwayConfig.debug;
 pdebug = gwayConfig.pdebug;
 _cad = gwayConfig.cad;
 _hop = gwayConfig.hop;
 gwayConfig.boots++;

#if GATEWAYNODE==1
 if (gwayConfig.fcnt != (uint8_t) 0) frameCount = gwayConfig.fcnt+10;
#endif

#if WIFIMANAGER==1
 String ssid=gwayConfig.ssid;
 String pass=gwayConfig.pass;

 char ssidBuf[ssid.length()+1];
 ssid.toCharArray(ssidBuf,ssid.length()+1);
 char passBuf[pass.length()+1];
 pass.toCharArray(passBuf,pass.length()+1);
 Serial.print(F("WlanReadWpa: ")); Serial.print(ssidBuf); Serial.print(F(", ")); Serial.println(passBuf);

 strcpy(wpa[0].login, ssidBuf);
 strcpy(wpa[0].passw, passBuf);

 Serial.print(F("WlanReadWpa: <"));
 Serial.print(wpa[0].login);
 Serial.print(F(">, <"));
 Serial.print(wpa[0].passw);
 Serial.println(F(">"));
#endif

}





#if WIFIMANAGER==1
int WlanWriteWpa( char* ssid, char *pass) {

#if DUSB>=1
 if (( debug >=0 ) && ( pdebug & P_MAIN )) {
  Serial.print(F("M WlanWriteWpa:: ssid="));
  Serial.print(ssid);
  Serial.print(F(", pass="));
  Serial.print(pass);
  Serial.println();
 }
#endif

 String s((char *) ssid);
 gwayConfig.ssid = s;

 String p((char *) pass);
 gwayConfig.pass = p;

#if GATEWAYNODE==1
 gwayConfig.fcnt = frameCount;
#endif
 gwayConfig.ch = ifreq;
 gwayConfig.sf = sf;
 gwayConfig.cad = _cad;
 gwayConfig.hop = _hop;

 writeConfig( CONFIGFILE, &gwayConfig);
 return 1;
}
#endif
# 230 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_WiFi.ino"
int WlanConnect(int maxTry) {

#if WIFIMANAGER==1
 WiFiManager wifiManager;
#endif

 unsigned char agains = 0;
 unsigned char wpa_index = (WIFIMANAGER >0 ? 0 : 1);




 if (maxTry==0) {
  Serial.println(F("WlanConnect:: Init para 0"));
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  if (gwayConfig.ssid.length() >0) {
   WiFi.begin(gwayConfig.ssid.c_str(), gwayConfig.pass.c_str());
   delay(100);
  }
 }




 int i=0;

 while ( (WiFi.status() != WL_CONNECTED) && ( i<= maxTry ) )
 {


  for (int j=wpa_index; (j< (sizeof(wpa)/sizeof(wpa[0]))) && (WiFi.status() != WL_CONNECTED ); j++)
  {

   char *ssid = wpa[j].login;
   char *password = wpa[j].passw;
#if DUSB>=1
   if (debug>=0) {
    Serial.print(i);
    Serial.print(':');
    Serial.print(j);
    Serial.print(':');
    Serial.print(sizeof(wpa)/sizeof(wpa[0]));
    Serial.print(F(". WiFi connect SSID="));
    Serial.print(ssid);
    if ( debug>=1 ) {
     Serial.print(F(", pass="));
     Serial.print(password);
    }
    Serial.println();
   }
#endif

   gwayConfig.wifis++;



   WiFi.mode(WIFI_STA);
   delay(1000);
   WiFi.begin(ssid, password);
   delay(8000);





   int stat = WlanStatus();
   if ( stat == 1) {
    writeGwayCfg(CONFIGFILE);
    return(1);
   }



   agains=1;
   while (((WiFi.status()) != WL_CONNECTED) && (agains < 10)) {
    agains++;
    delay(agains*500);
#if DUSB>=1
    if ( debug>=0 ) {
     Serial.print(".");
    }
#endif
   }
#if DUSB>=1
   Serial.println();
#endif





   WiFi.persistent(false);
   WiFi.mode(WIFI_OFF);

  }

  i++;
 }




 if (WiFi.status() != WL_CONNECTED) {
#if WIFIMANAGER==1
#if DUSB>=1
  Serial.println(F("Starting Access Point Mode"));
  Serial.print(F("Connect Wifi to accesspoint: "));
  Serial.print(AP_NAME);
  Serial.print(F(" and connect to IP: 192.168.4.1"));
  Serial.println();
#endif
  wifiManager.autoConnect(AP_NAME, AP_PASSWD );







  struct station_config sta_conf;
  wifi_station_get_config(&sta_conf);


  WlanWriteWpa((char *)sta_conf.ssid, (char *)sta_conf.password);
#else
#if DUSB>=1
  if (debug>=0) {
   Serial.println(F("WlanConnect:: Not connected after all"));
   Serial.print(F("WLAN retry="));
   Serial.print(i);
   Serial.print(F(" , stat="));
   Serial.print(WiFi.status() );
   Serial.println();
  }
#endif
  return(-1);
#endif
 }

 yield();
 return(1);
}
# 1 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_gatewayMgt.ino"
# 31 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_gatewayMgt.ino"
#if GATEWAYMGT==1

#if !defined _THINGPORT
#error "The management functions needs _THINGPORT defined (and not over _TTNPORT)"
#endif
# 56 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_gatewayMgt.ino"
void gateway_mgt(uint8_t size, uint8_t *buff) {

 uint8_t opcode = buff[3];

 switch (opcode) {
  case MGT_RESET:
   Serial.println(F("gateway_mgt:: RESET"));

   setup();


  break;
  case MGT_SET_SF:
   Serial.println(F("gateway_mgt:: SET SF"));

  break;
  case MGT_SET_FREQ:
   Serial.println(F("gateway_mgt:: SET FREQ"));

  break;
  default:
   Serial.print(F("gateway_mgt:: Unknown UDP code="));
   Serial.println(opcode);
   return;
  break;
 }
}

#endif
# 1 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_loraFiles.ino"
# 20 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_loraFiles.ino"
#include "asi-src/src/asi.h"

#define LOGFILEREC 10
#define LOGFILEMAX 10
# 32 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_loraFiles.ino"
void id_print (String id, String val) {
#if DUSB>=1
 if (( debug>=0 ) && ( pdebug & P_MAIN )) {
  Serial.print(id);
  Serial.print(F("=\t"));
  Serial.println(val);
 }
#endif
}







int initConfig(struct espGwayConfig *c) {
 (*c).ch = 0;
 (*c).sf = _SPREADING;
 (*c).debug = 1;
 (*c).pdebug = P_GUI;
 (*c).cad = _CAD;
 (*c).hop = false;
 (*c).expert = false;
}





int readConfig(const char *fn, struct espGwayConfig *c) {

 int tries = 0;
#if DUSB>=1
 Serial.println(F("readConfig:: Starting "));
#endif
 if (!SPIFFS.exists(fn)) {
#if DUSB>=1
  if (( debug>=0 ) && ( pdebug & P_MAIN ))
  Serial.print(F("M ERR:: readConfig, file="));
  Serial.print(fn);
  Serial.println(F(" does not exist .. Formatting"));
#endif

  initConfig(c);
  return(-1);
 }

 File f = SPIFFS.open(fn, "r");
 if (!f) {
  Serial.println(F("ERROR:: SPIFFS open failed"));
  return(-1);
 }

 while (f.available()) {

#if DUSB>=1
  if (( debug>=0 ) && ( pdebug & P_MAIN )) {
   Serial.print('.');
  }
#endif



  if (tries >= 10) {
   f.close();
#if DUSB>=1
   if (( debug>=0 ) && ( pdebug & P_MAIN ))
    Serial.println(F("Formatting"));
#endif
   SPIFFS.format();
   initConfig(c);
   f = SPIFFS.open(fn, "r");
   tries = 0;
  }

  String id =f.readStringUntil('=');
  String val=f.readStringUntil('\n');

  if (id == "SSID") {
   id_print(id, val);
   (*c).ssid = val;
  }
  else if (id == "PASS") {
   id_print(id, val);
   (*c).pass = val;
  }
  else if (id == "CH") {
   id_print(id,val);
   (*c).ch = (uint32_t) val.toInt();
  }
  else if (id == "SF") {
   id_print(id, val);
   (*c).sf = (uint32_t) val.toInt();
  }
  else if (id == "FCNT") {
   id_print(id, val);
   (*c).fcnt = (uint32_t) val.toInt();
  }
  else if (id == "DEBUG") {
   id_print(id, val);
   (*c).debug = (uint8_t) val.toInt();
  }
  else if (id == "PDEBUG") {
   Serial.print(F("PDEBUG=")); Serial.println(val);
   (*c).pdebug = (uint8_t) val.toInt();
  }
  else if (id == "CAD") {
   Serial.print(F("CAD=")); Serial.println(val);
   (*c).cad = (uint8_t) val.toInt();
  }
  else if (id == "HOP") {
   Serial.print(F("HOP=")); Serial.println(val);
   (*c).hop = (uint8_t) val.toInt();
  }
  else if (id == "BOOTS") {
   id_print(id, val);
   (*c).boots = (uint8_t) val.toInt();
  }
  else if (id == "RESETS") {
   id_print(id, val);
   (*c).resets = (uint8_t) val.toInt();
  }
  else if (id == "WIFIS") {
   id_print(id, val);
   (*c).wifis = (uint8_t) val.toInt();
  }
  else if (id == "VIEWS") {
   id_print(id, val);
   (*c).views = (uint8_t) val.toInt();
  }
  else if (id == "NODE") {
   id_print(id, val);
   (*c).isNode = (uint8_t) val.toInt();
  }
  else if (id == "REFR") {
   id_print(id, val);
   (*c).refresh = (uint8_t) val.toInt();
  }
  else if (id == "REENTS") {
   id_print(id, val);
   (*c).reents = (uint8_t) val.toInt();
  }
  else if (id == "NTPERR") {
   id_print(id, val);
   (*c).ntpErr = (uint8_t) val.toInt();
  }
  else if (id == "NTPETIM") {
   id_print(id, val);
   (*c).ntpErrTime = (uint32_t) val.toInt();
  }
  else if (id == "NTPS") {
   id_print(id, val);
   (*c).ntps = (uint8_t) val.toInt();
  }
  else if (id == "FILENO") {
   id_print(id, val);
   (*c).logFileNo = (uint8_t) val.toInt();
  }
  else if (id == "FILEREC") {
   id_print(id, val);
   (*c).logFileRec = (uint16_t) val.toInt();
  }
  else if (id == "FILENUM") {
   id_print(id, val);
   (*c).logFileNum = (uint16_t) val.toInt();
  }
  else if (id == "EXPERT") {
   id_print(id, val);
   (*c).expert = (uint8_t) val.toInt();
  }
  else {
   tries++;
  }
 }
 f.close();
#if DUSB>=1
 if (debug>=0) {
  Serial.println('#');
 }
#endif
 Serial.println();
 return(1);
}






int writeGwayCfg(const char *fn) {

 gwayConfig.ssid = WiFi.SSID();
 gwayConfig.pass = WiFi.psk();
 gwayConfig.ch = ifreq;
 gwayConfig.sf = (uint8_t) global_sf;
 gwayConfig.debug = debug;
 gwayConfig.pdebug = pdebug;
 gwayConfig.cad = cadGet();
 gwayConfig.hop = _hop;
#if GATEWAYNODE==1
 gwayConfig.fcnt = frameCount;
#endif
 return(writeConfig(fn, &gwayConfig));
}
# 247 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_loraFiles.ino"
int writeConfig(const char *fn, struct espGwayConfig *c) {

 if (!SPIFFS.exists(fn)) {
  Serial.print("WARNING:: writeConfig, file not exists, formatting ");

  initConfig(c);
  Serial.println(fn);
 }
 File f = SPIFFS.open(fn, "w");
 if (!f) {
  Serial.print("ERROR:: writeConfig, open file=");
  Serial.print(fn);
  Serial.println();
  return(-1);
 }

 f.print("SSID"); f.print('='); f.print((*c).ssid); f.print('\n');
 f.print("PASS"); f.print('='); f.print((*c).pass); f.print('\n');
 f.print("CH"); f.print('='); f.print((*c).ch); f.print('\n');
 f.print("SF"); f.print('='); f.print((*c).sf); f.print('\n');
 f.print("FCNT"); f.print('='); f.print((*c).fcnt); f.print('\n');
 f.print("DEBUG"); f.print('='); f.print((*c).debug); f.print('\n');
 f.print("PDEBUG"); f.print('='); f.print((*c).pdebug); f.print('\n');
 f.print("CAD"); f.print('='); f.print((*c).cad); f.print('\n');
 f.print("HOP"); f.print('='); f.print((*c).hop); f.print('\n');
 f.print("NODE"); f.print('='); f.print((*c).isNode); f.print('\n');
 f.print("BOOTS"); f.print('='); f.print((*c).boots); f.print('\n');
 f.print("RESETS"); f.print('='); f.print((*c).resets); f.print('\n');
 f.print("WIFIS"); f.print('='); f.print((*c).wifis); f.print('\n');
 f.print("VIEWS"); f.print('='); f.print((*c).views); f.print('\n');
 f.print("REFR"); f.print('='); f.print((*c).refresh); f.print('\n');
 f.print("REENTS"); f.print('='); f.print((*c).reents); f.print('\n');
 f.print("NTPETIM"); f.print('='); f.print((*c).ntpErrTime); f.print('\n');
 f.print("NTPERR"); f.print('='); f.print((*c).ntpErr); f.print('\n');
 f.print("NTPS"); f.print('='); f.print((*c).ntps); f.print('\n');
 f.print("FILEREC"); f.print('='); f.print((*c).logFileRec); f.print('\n');
 f.print("FILENO"); f.print('='); f.print((*c).logFileNo); f.print('\n');
 f.print("FILENUM"); f.print('='); f.print((*c).logFileNum); f.print('\n');
 f.print("EXPERT"); f.print('='); f.print((*c).expert); f.print('\n');

 f.close();
 return(1);
}
# 304 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_loraFiles.ino"
void addLog(const unsigned char * line, int cnt)
{
#if STAT_LOG==1
 char fn[16];

 if (gwayConfig.logFileRec > LOGFILEREC) {
  gwayConfig.logFileRec = 0;
  gwayConfig.logFileNo++;
  gwayConfig.logFileNum++;
 }
 gwayConfig.logFileRec++;



 if (gwayConfig.logFileNum > LOGFILEMAX){
  sprintf(fn,"/log-%d", gwayConfig.logFileNo - LOGFILEMAX);
#if DUSB>=1
  if (( debug>=0 ) && ( pdebug & P_GUI )) {
   Serial.print(F("G addLog:: Too many logfile, deleting="));
   Serial.println(fn);
  }
#endif
  SPIFFS.remove(fn);
  gwayConfig.logFileNum--;
 }


 sprintf(fn,"/log-%d", gwayConfig.logFileNo);



 if (!SPIFFS.exists(fn)) {
#if DUSB>=1
  if (( debug >= 1 ) && ( pdebug & P_GUI )) {
   Serial.print(F("G ERROR:: addLog:: file="));
   Serial.print(fn);
   Serial.print(F(" does not exist .. rec="));
   Serial.print(gwayConfig.logFileRec);
   Serial.println();
  }
#endif
 }

 File f = SPIFFS.open(fn, "a");
 if (!f) {
#if DUSB>=1
  if (( debug>=1 ) && ( pdebug & P_GUI )) {
   Serial.println("G file open failed=");
   Serial.println(fn);
  }
#endif
  return;
 }

 int i;
#if DUSB>=1
 if (( debug>=1 ) && ( pdebug & P_GUI )) {
  Serial.print(F("G addLog:: fileno="));
  Serial.print(gwayConfig.logFileNo);
  Serial.print(F(", rec="));
  Serial.print(gwayConfig.logFileRec);

  Serial.print(F(": "));

  for (i=0; i< 12; i++) {
   Serial.print(line[i],HEX);
   Serial.print(' ');
  }
  Serial.print((char *) &line[i]);

  Serial.println();
 }
#endif


 for (i=0; i< 12; i++) {

  f.print('*');
 }
 f.write(&(line[i]), cnt-12);
 f.print('\n');
 f.close();

#endif
}





void printLog()
{
 char fn[16];
 int i=0;
#if DUSB>=1
 while (i< LOGFILEMAX ) {
  sprintf(fn,"/log-%d", gwayConfig.logFileNo - i);
  if (!SPIFFS.exists(fn)) break;


  File f = SPIFFS.open(fn, "r");

  int j;
  for (j=0; j<LOGFILEREC; j++) {

   String s=f.readStringUntil('\n');
   if (s.length() == 0) break;

   Serial.println(s.substring(12));
   yield();
  }
  i++;
 }
#endif
}






void listDir(char * dir)
{
#if DUSB>=1

#endif
}
# 1 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_oLED.ino"
# 23 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_oLED.ino"
#if OLED>=1






void init_oLED()
{
#if defined OLED_RST
 pinMode(OLED_RST,OUTPUT);
 digitalWrite(OLED_RST, LOW);
 delay(50);
 digitalWrite(OLED_RST, HIGH);
 delay(50);
#else
#endif

 display.init();
 display.flipScreenVertically();
 display.setFont(ArialMT_Plain_24);
 display.setTextAlignment(TEXT_ALIGN_LEFT);
 display.drawString(0, 24, "STARTING");
 display.display();
}





void acti_oLED()
{

 display.clear();

#if OLED==1
 display.setFont(ArialMT_Plain_16);
 display.drawString(0, 16, "READY,  SSID=");
 display.drawString(0, 32, WiFi.SSID());
#elif OLED==2
 display.setFont(ArialMT_Plain_16);
 display.drawString(0, 16, "READY,  SSID=");
 display.drawString(0, 32, WiFi.SSID());
#endif

 display.display();
}






void msg_oLED(String tim, String sf) {
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);

 display.drawString(0, 48, "LEN: " );

    display.display();
 yield();
}





void addr_oLED()
{
 Serial.print(F("OLED_ADDR=0x"));
 Serial.println(OLED_ADDR, HEX);
}



#endif
# 1 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_otaServer.ino"
# 22 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_otaServer.ino"
#if A_OTA==1
# 32 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_otaServer.ino"
void setupOta(char *hostname) {

 ArduinoOTA.begin();
#if DUSB>=1
 Serial.println(F("setupOta:: Started"));
#endif

 ArduinoOTA.setHostname(hostname);

 ArduinoOTA.onStart([]() {
  String type;



   type = "sketch";




  Serial.println("Start updating " + type);
 });

 ArduinoOTA.onEnd([]() {
  Serial.println("\nEnd");
 });

 ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
 });

 ArduinoOTA.onError([](ota_error_t error) {
  Serial.printf("Error[%u]: ", error);
  if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
  else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
  else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
  else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
  else if (error == OTA_END_ERROR) Serial.println("End Failed");
 });

#if DUSB>=1
 Serial.println("Ready");
 Serial.print("IP address: ");
 Serial.println(WiFi.localIP());
#endif


#if A_SERVER==2
 ESPhttpUpdate.rebootOnUpdate(false);

 server.on("/esp", HTTP_POST, [&](){

      HTTPUpdateResult ret = ESPhttpUpdate.update(server.arg("firmware"), "1.0.0");

      switch(ret) {
        case HTTP_UPDATE_FAILED:

   Serial.println(F("Update failed"));
            break;
        case HTTP_UPDATE_NO_UPDATES:

   Serial.println(F("Update not necessary"));
            break;
        case HTTP_UPDATE_OK:

   Serial.println(F("Update started"));
            ESP.restart();
            break;
  default:
   Serial.println(F("setupOta:: Unknown ret="));
      }
 });
#endif
}






void updateOtaa() {

 String response="";
 printIP((IPAddress)WiFi.localIP(),'.',response);

 ESPhttpUpdate.update(response, 80, "/arduino.bin");

}


#endif
# 1 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_repeater.ino"
# 21 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_repeater.ino"
#if REPEATER==1

#define _ICHAN 0
#define _OCHAN 1

#ifdef _TTNSERVER
#error "Please undefined _THINGSERVER, for REAPETR shutdown WiFi"
#endif




int sendLora(char *msg, int len) {

 Serial.print("sendLora:: ");

 for (int i=0; i< len; i++) {
  Serial.print(msg[1],HEX);
  Serial.print('.');
 }

 if (debug>=2) Serial.flush();
 return(1);
}

#endif
# 1 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_sensor.ino"
# 24 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_sensor.ino"
#if GATEWAYNODE==1

#include "LoRaCode.h"

unsigned char DevAddr[4] = _DEVADDR ;



#if _GPS==1




static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}
#endif
# 73 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_sensor.ino"
static int LoRaSensors(uint8_t *buf) {

 uint8_t tchars = 1;
 buf[0] = 0x86;

#if DUSB>=1
 if (debug>=0)
  Serial.print(F("LoRaSensors:: "));
#endif

#if _BATTERY==1
#if DUSB>=1
 if (debug>=0)
  Serial.print(F("Battery "));
#endif
#if defined(ARDUINO_ARCH_ESP8266) || defined(ESP32)


 pinMode(35, INPUT);
#if defined(ESP32)
 int devider=4095;
#else
 int devider=1023;
#endif
 float volts=3.3 * analogRead(35) / 4095 * 2;
#else

 float volts=0;
#endif
 tchars += lcode.eBattery(volts, buf + tchars);
#endif

#if _GPS==1
#if DUSB>=1
 if (debug>=0)
  Serial.print(F("M GPS "));

 if (( debug>=1 ) && ( pdebug & P_MAIN )) {
  Serial.print("\tLatitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("\tLongitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("\tSatellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("\tAltitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  Serial.println("M");
  Serial.print("\tTime      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
 }
#endif

 smartDelay(1000);

 if (millis() > 5000 && gps.charsProcessed() < 10) {
#if DUSB>=1
  Serial.println(F("No GPS data received: check wiring"));
#endif
  return(0);
 }




 tchars += lcode.eGpsL(gps.location.lat(), gps.location.lng(), gps.altitude.value(),
                       gps.satellites.value(), buf + tchars);

#endif

#if DUSB>=1
 if (debug>=0)
  Serial.println();
#endif


 lcode.eMsg(buf, tchars);

 return(tchars);
}
# 164 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_sensor.ino"
static void mXor(uint8_t *buf, uint8_t *key) {
 for (uint8_t i = 0; i < 16; ++i) buf[i] ^= key[i];
}
# 176 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_sensor.ino"
static void shift_left(uint8_t * buf, uint8_t len) {
    while (len--) {
        uint8_t next = len ? buf[1] : 0;

        uint8_t val = (*buf << 1);
        if (next & 0x80) val |= 0x01;
        *buf++ = val;
    }
}






static void generate_subkey(uint8_t *key, uint8_t *k1, uint8_t *k2) {

 memset(k1, 0, 16);


 AES_Encrypt(k1,key);


 if (k1[0] & 0x80) {
  shift_left(k1,16);
  k1[15] ^= 0x87;
 }
 else {
  shift_left(k1,16);
 }


 for (uint8_t i=0; i<16; i++) k2[i]=k1[i];
 if (k1[0] & 0x80) {
  shift_left(k2,16);
  k2[15] ^= 0x87;
 }
 else {
  shift_left(k2,16);
 }


 return;
}
# 244 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_sensor.ino"
uint8_t micPacket(uint8_t *data, uint8_t len, uint16_t FrameCount, uint8_t * NwkSKey, uint8_t dir) {



 uint8_t Block_B[16];
 uint8_t X[16];
 uint8_t Y[16];



 Block_B[0]= 0x49;

 Block_B[1]= 0x00;
 Block_B[2]= 0x00;
 Block_B[3]= 0x00;
 Block_B[4]= 0x00;

 Block_B[5]= dir;

 Block_B[6]= DevAddr[3];
 Block_B[7]= DevAddr[2];
 Block_B[8]= DevAddr[1];
 Block_B[9]= DevAddr[0];

 Block_B[10]= (FrameCount & 0x00FF);
 Block_B[11]= ((FrameCount >> 8) & 0x00FF);
 Block_B[12]= 0x00;
 Block_B[13]= 0x00;

 Block_B[14]= 0x00;

 Block_B[15]= len;




 uint8_t k1[16];
 uint8_t k2[16];
 generate_subkey(NwkSKey, k1, k2);




 uint8_t micBuf[len+16];
 for (uint8_t i=0; i<16; i++) micBuf[i]=Block_B[i];
 for (uint8_t i=0; i<len; i++) micBuf[i+16]=data[i];




 uint8_t numBlocks = len/16 + 1;
 if ((len % 16)!=0) numBlocks++;




 uint8_t restBits = len%16;





 memset(X, 0, 16);




 for(uint8_t i= 0x0; i < (numBlocks - 1); i++) {
  for (uint8_t j=0; j<16; j++) Y[j] = micBuf[(i*16)+j];
  mXor(Y, X);
  AES_Encrypt(Y, NwkSKey);
  for (uint8_t j=0; j<16; j++) X[j] = Y[j];
 }







 if (restBits) {
  for (uint8_t i=0; i<16; i++) {
   if (i< restBits) Y[i] = micBuf[((numBlocks-1)*16)+i];
   if (i==restBits) Y[i] = 0x80;
   if (i> restBits) Y[i] = 0x00;
  }
  mXor(Y, k2);
 }
 else {
  for (uint8_t i=0; i<16; i++) {
   Y[i] = micBuf[((numBlocks-1)*16)+i];
  }
  mXor(Y, k1);
 }
 mXor(Y, X);
 AES_Encrypt(Y,NwkSKey);






 data[len+0]=Y[0];
 data[len+1]=Y[1];
 data[len+2]=Y[2];
 data[len+3]=Y[3];
 return 4;
}


#if _CHECK_MIC==1
# 364 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_sensor.ino"
static void checkMic(uint8_t *buf, uint8_t len, uint8_t *key) {
 uint8_t cBuf[len+1];
 uint8_t NwkSKey[16] = _NWKSKEY;

 if (debug>=2) {
  Serial.print(F("old="));
  for (uint8_t i=0; i<len; i++) {
   printHexDigit(buf[i]);
   Serial.print(' ');
  }
  Serial.println();
 }
 for (uint8_t i=0; i<len-4; i++) cBuf[i] = buf[i];
 len -=4;

 uint16_t FrameCount = ( cBuf[7] * 256 ) + cBuf[6];
 len += micPacket(cBuf, len, FrameCount, NwkSKey, 0);

 if (debug>=2) {
  Serial.print(F("new="));
  for (uint8_t i=0; i<len; i++) {
   printHexDigit(cBuf[i]);
   Serial.print(' ');
  }
  Serial.println();
 }

}
#endif
# 420 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_sensor.ino"
int sensorPacket() {

 uint8_t buff_up[512];
 uint8_t message[64]={ 0 };
 uint8_t mlength = 0;
 uint32_t tmst = micros();
 struct LoraUp LUP;
 uint8_t NwkSKey[16] = _NWKSKEY;
 uint8_t AppSKey[16] = _APPSKEY;
 uint8_t DevAddr[4] = _DEVADDR;


 LUP.sf = 8;
 LUP.prssi = -50;
 LUP.rssicorr = 139;
 LUP.snr = 0;
# 444 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_sensor.ino"
 LUP.payLoad[0] = 0x40;





 LUP.payLoad[1] = DevAddr[3];
 LUP.payLoad[2] = DevAddr[2];
 LUP.payLoad[3] = DevAddr[1];
 LUP.payLoad[4] = DevAddr[0];

 LUP.payLoad[5] = 0x00;
 LUP.payLoad[6] = frameCount % 0x100;
 LUP.payLoad[7] = frameCount / 0x100;




 LUP.payLoad[8] = 0x01;
 LUP.payLength = 9;







 uint8_t PayLength = LoRaSensors((uint8_t *)(LUP.payLoad + LUP.payLength));

#if DUSB>=1
 if ((debug>=2) && (pdebug & P_RADIO )) {
  Serial.print(F("old: "));
  for (int i=0; i<PayLength; i++) {
   Serial.print(LUP.payLoad[i],HEX);
   Serial.print(' ');
  }
  Serial.println();
 }
#endif


 uint8_t CodeLength = encodePacket((uint8_t *)(LUP.payLoad + LUP.payLength), PayLength, (uint16_t)frameCount, DevAddr, AppSKey, 0);

#if DUSB>=1
 if ((debug>=2) && (pdebug & P_RADIO )) {
  Serial.print(F("new: "));
  for (int i=0; i<CodeLength; i++) {
   Serial.print(LUP.payLoad[i],HEX);
   Serial.print(' ');
  }
  Serial.println();
 }
#endif

 LUP.payLength += CodeLength;







 LUP.payLength += micPacket((uint8_t *)(LUP.payLoad), LUP.payLength, (uint16_t)frameCount, NwkSKey, 0);

#if DUSB>=1
 if ((debug>=2) && (pdebug & P_RADIO )) {
  Serial.print(F("mic: "));
  for (int i=0; i<LUP.payLength; i++) {
   Serial.print(LUP.payLoad[i],HEX);
   Serial.print(' ');
  }
  Serial.println();
 }
#endif





 int buff_index = buildPacket(tmst, buff_up, LUP, true);

 frameCount++;
    cp_nb_rx_rcv++;





 if (( frameCount % 10)==0) writeGwayCfg(CONFIGFILE);

 if (buff_index > 512) {
  if (debug>0) Serial.println(F("sensorPacket:: ERROR buffer size too large"));
  return(-1);
 }

#ifdef _TTNSERVER
 if (!sendUdp(ttnServer, _TTNPORT, buff_up, buff_index)) {
  return(-1);
 }
#endif
#ifdef _THINGSERVER
 if (!sendUdp(thingServer, _THINGPORT, buff_up, buff_index)) {
  return(-1);
 }
#endif

#if DUSB>=1


 if ((debug>=2) && (pdebug & P_RADIO )) {
  CodeLength = encodePacket((uint8_t *)(LUP.payLoad + 9), PayLength, (uint16_t)frameCount-1, DevAddr, AppSKey, 0);
  Serial.print(F("rev: "));
  for (int i=0; i<CodeLength; i++) {
   Serial.print(LUP.payLoad[i],HEX);
   Serial.print(' ');
  }
  Serial.print(F(", addr="));
  for (int i=0; i<4; i++) {
   Serial.print(DevAddr[i],HEX);
   Serial.print(' ');
  }
  Serial.println();
 }
#endif

 if (_cad) {

  _state = S_SCAN;
  sf = SF7;
  cadScanner();
 }
 else {

  _state = S_RX;
  rxLoraModem();
 }

 return(buff_index);
}

#endif

#if (GATEWAYNODE==1) || (_LOCALSERVER==1)
# 611 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_sensor.ino"
uint8_t encodePacket(uint8_t *Data, uint8_t DataLength, uint16_t FrameCount, uint8_t *DevAddr, uint8_t *AppSKey, uint8_t Direction) {

#if DUSB>=1
 if (( debug>=2 ) && ( pdebug & P_GUI )) {
  Serial.print(F("G encodePacket:: DevAddr="));
  for (int i=0; i<4; i++ ) { Serial.print(DevAddr[i],HEX); Serial.print(' '); }
  Serial.print(F("G encodePacket:: AppSKey="));
  for (int i=0; i<16; i++ ) { Serial.print(AppSKey[i],HEX); Serial.print(' '); }
  Serial.println();
 }
#endif


 uint8_t i, j;
 uint8_t Block_A[16];
 uint8_t bLen=16;

 uint8_t restLength = DataLength % 16;
 uint8_t numBlocks = DataLength / 16;
 if (restLength>0) numBlocks++;

 for(i = 1; i <= numBlocks; i++) {
  Block_A[0] = 0x01;

  Block_A[1] = 0x00;
  Block_A[2] = 0x00;
  Block_A[3] = 0x00;
  Block_A[4] = 0x00;

  Block_A[5] = Direction;

  Block_A[6] = DevAddr[3];
  Block_A[7] = DevAddr[2];
  Block_A[8] = DevAddr[1];
  Block_A[9] = DevAddr[0];

  Block_A[10] = (FrameCount & 0x00FF);
  Block_A[11] = ((FrameCount >> 8) & 0x00FF);
  Block_A[12] = 0x00;
  Block_A[13] = 0x00;

  Block_A[14] = 0x00;

  Block_A[15] = i;


  AES_Encrypt(Block_A, AppSKey);


  if ((i == numBlocks) && (restLength>0)) bLen = restLength;

  for(j = 0; j < bLen; j++) {
   *Data = *Data ^ Block_A[j];
   Data++;
  }
 }

 return(DataLength);
}

#endif
# 1 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_stateMachine.ino"
# 23 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_stateMachine.ino"
#include "_loraModem.h"
# 74 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_stateMachine.ino"
void stateMachine()
{


 uint8_t flags = readRegister(REG_IRQ_FLAGS);
 uint8_t mask = readRegister(REG_IRQ_FLAGS_MASK);
 uint8_t intr = flags & ( ~ mask );
 uint8_t rssi;
 _event=0;

#if DUSB>=1
 if (intr != flags) {
  Serial.print(F("FLAG  ::"));
  SerialStat(intr);
 }
#endif





 if ((_hop) && (intr == 0x00) )
 {




  if ((_state == S_SCAN) || (_state == S_CAD)) {

   _event=0;

   uint32_t eventWait = EVENT_WAIT;
   switch (_state) {
    case S_INIT: eventWait = 0; break;

    case S_SCAN: eventWait = EVENT_WAIT * 1; break;
    case S_CAD: eventWait = EVENT_WAIT * 1; break;

    case S_RX: eventWait = EVENT_WAIT * 8; break;
    case S_TX: eventWait = EVENT_WAIT * 1; break;
    case S_TXDONE: eventWait = EVENT_WAIT * 4; break;
    default:
     eventWait=0;
#if DUSB>=1
     Serial.print(F("DEFAULT :: "));
     SerialStat(intr);
#endif
   }






   uint32_t doneWait = DONE_WAIT;
   switch (global_sf) {
    case SF7: break;
    case SF8: doneWait *= 2; break;
    case SF9: doneWait *= 4; break;
    case SF10: doneWait *= 8; break;
    case SF11: doneWait *= 16; break;
    case SF12: doneWait *= 32; break;
    default:
     doneWait *= 1;
#if DUSB>=1
     if (( debug>=0 ) && ( pdebug & P_PRE )) {
      Serial.print(F("PRE:: DEF set"));
      Serial.println();
     }
#endif
     break;
   }




   if (eventTime > micros()) eventTime=micros();
   if (doneTime > micros()) doneTime=micros();

   if (((micros() - doneTime) > doneWait ) &&
    (( _state == S_SCAN ) || ( _state == S_CAD )))
   {
    _state = S_SCAN;
    hop();
    cadScanner();
#if DUSB>=1
    if (( debug >= 1 ) && ( pdebug & P_PRE )) {
     Serial.print(F("DONE  :: "));
     SerialStat(intr);
    }
#endif
    eventTime=micros();
    doneTime=micros();
    return;
   }



   if ((micros() - eventTime) > eventWait )
   {
    _state = S_SCAN;
    hop();
    cadScanner();
#if DUSB>=1
    if (( debug >= 2 ) && ( pdebug & P_PRE )) {
     Serial.print(F("HOP ::  "));
     SerialStat(intr);
    }
#endif
    eventTime=micros();
    doneTime=micros();
    return;
   }





#if DUSB>=1
   if (( debug>=3 ) && ( pdebug & P_PRE )) {
    Serial.print(F("PRE:: eventTime="));
    Serial.print(eventTime);
    Serial.print(F(", micros="));
    Serial.print(micros());
    Serial.print(F(": "));
    SerialStat(intr);
   }
#endif
  }


  else {

  }

  yield();

 }
# 221 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_stateMachine.ino"
 switch (_state)
 {




   case S_INIT:
#if DUSB>=1
  if (( debug>=1 ) && ( pdebug & P_PRE )) {
   Serial.println(F("S_INIT"));
  }
#endif

  writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF );
  writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00 );
  _event=0;
   break;







   case S_SCAN:





  if (intr & IRQ_LORA_CDDETD_MASK) {

   _state = S_RX;


   writeRegister(REG_DIO_MAPPING_1, (
    MAP_DIO0_LORA_RXDONE |
    MAP_DIO1_LORA_RXTOUT |
    MAP_DIO2_LORA_NOP |
    MAP_DIO3_LORA_CRC));



   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
    IRQ_LORA_RXDONE_MASK |
    IRQ_LORA_RXTOUT_MASK |
    IRQ_LORA_HEADER_MASK |
    IRQ_LORA_CRCERR_MASK));







   rssi = readRegister(REG_RSSI);
   _rssi = rssi;

   _event = 0;
   detTime = micros();

#if DUSB>=1
   if (( debug>=1 ) && ( pdebug & P_SCAN )) {
    Serial.print(F("SCAN:: "));
    SerialStat(intr);
   }
#endif
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF );
   opmode(OPMODE_RX_SINGLE);

  }







  else if (intr & IRQ_LORA_CDDONE_MASK) {

   opmode(OPMODE_CAD);
   rssi = readRegister(REG_RSSI);

#if DUSB>=1
   if (( debug>=2 ) && ( pdebug & P_SCAN )) {
    Serial.print(F("SCAN:: CDDONE: "));
    SerialStat(intr);
   }
#endif







   if ( rssi > (RSSI_LIMIT - (_hop * 7)))
   {
#if DUSB>=1
    if (( debug>=2 ) && ( pdebug & P_SCAN )) {
     Serial.print(F("SCAN:: -> CAD: "));
     SerialStat(intr);
    }
#endif
    _state = S_CAD;
    _event=0;
   }



   else {
#if DUSB>=1
    if (( debug>=2 ) && ( pdebug & P_SCAN )) {
     Serial.print("SCAN:: rssi=");
     Serial.print(rssi);
     Serial.print(F(": "));
     SerialStat(intr);
    }
#endif
    _state = S_SCAN;

   }


   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
   doneTime = micros();

  }
# 366 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_stateMachine.ino"
  else if (intr == 0x00)
  {
   _event=0;
  }



  else {
#if DUSB>=1
   if (( debug>=0 ) && ( pdebug & P_SCAN )) {
    Serial.print(F("SCAN unknown:: "));
    SerialStat(intr);
   }
#endif
   _state=S_SCAN;

   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
  }

   break;
# 400 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_stateMachine.ino"
   case S_CAD:





  if (intr & IRQ_LORA_CDDETD_MASK) {


   writeRegister(REG_DIO_MAPPING_1, (
    MAP_DIO0_LORA_RXDONE |
    MAP_DIO1_LORA_RXTOUT |
    MAP_DIO2_LORA_NOP |
    MAP_DIO3_LORA_CRC ));


   _event=0;



   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
    IRQ_LORA_RXDONE_MASK |
    IRQ_LORA_RXTOUT_MASK |
    IRQ_LORA_HEADER_MASK |
    IRQ_LORA_CRCERR_MASK ));





   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF );


   opmode(OPMODE_RX_SINGLE);

   delayMicroseconds( RSSI_WAIT );

   rssi = readRegister(REG_RSSI);
   _rssi = rssi;

   detTime = micros();
#if DUSB>=1
   if (( debug>=1 ) && ( pdebug & P_CAD )) {
    Serial.print(F("CAD:: "));
    SerialStat(intr);
   }
#endif
   _state = S_RX;

  }




  else if (intr & IRQ_LORA_CDDONE_MASK) {



   if (((uint8_t)global_sf) < SF12) {

    global_sf = (sf_t)((uint8_t)global_sf+1);
    setRate(global_sf, 0x04);


    _event=0;
    writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);

    writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF );

    opmode(OPMODE_CAD);

    delayMicroseconds(RSSI_WAIT);
    rssi = readRegister(REG_RSSI);

#if DUSB>=1
    if (( debug>=2 ) && ( pdebug & P_CAD )) {
     Serial.print(F("S_CAD:: CDONE, SF="));
     Serial.println(global_sf);
    }
#endif
   }



   else {


    _event=1;
    writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
    writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF );

    _state = S_SCAN;
    global_sf = SF7;
    cadScanner();

#if DUSB>=1
    if (( debug>=2 ) && ( pdebug & P_CAD )) {
     Serial.print(F("CAD->SCAN:: "));
     SerialStat(intr);
    }
#endif
   }
   doneTime = micros();

  }







  else if (intr == 0x00) {
#if DUSB>=0
   if (( debug>=3 ) && ( pdebug & P_CAD )) {
    Serial.println("Err CAD:: intr is 0x00");
   }
#endif
   _event=1;
  }




  else {
#if DUSB>=1
   if (( debug>=0) && ( pdebug & P_CAD )) {
    Serial.print(F("Err CAD: Unknown::"));
    SerialStat(intr);
   }
#endif
   _state = S_SCAN;
   global_sf = SF7;
   cadScanner();


   _event=1;
   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);

  }
   break;
# 551 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_stateMachine.ino"
   case S_RX:

  if (intr & IRQ_LORA_RXDONE_MASK) {

#if CRCCHECK==1





   if (intr & IRQ_LORA_CRCERR_MASK) {
#if DUSB>=1
    if (( debug>=0 ) && ( pdebug & P_RX )) {
     Serial.print(F("Rx CRC err: "));
     SerialStat(intr);
    }
#endif
    if (cadGet()) {
     global_sf = SF7;
     _state = S_SCAN;
     cadScanner();
    }
    else {
     _state = S_RX;
     rxLoraModem();
    }


    _event=0;
    writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
    writeRegister(REG_IRQ_FLAGS, (uint8_t)(
     IRQ_LORA_RXDONE_MASK |
     IRQ_LORA_RXTOUT_MASK |
     IRQ_LORA_HEADER_MASK |
     IRQ_LORA_CRCERR_MASK ));

    break;
   }
#endif


#if DUSB>=1
   unsigned long ffTime = micros();
#endif

   LoraUp.payLoad[0]= 0x00;







   if((LoraUp.payLength = receivePkt(LoraUp.payLoad)) <= 0) {
#if DUSB>=1
    if (( debug>=1 ) && ( pdebug & P_RX )) {
     Serial.print(F("sMachine:: Error S-RX: "));
     Serial.print(F("payLength="));
     Serial.print(LoraUp.payLength);
     Serial.println();
    }
#endif
    _event=1;
    writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);





    writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);

    _state = S_SCAN;
    break;
   }
#if DUSB>=1
   if (( debug>=1 ) && ( pdebug & P_RX )) {
    Serial.print(F("RXDONE in dT="));
    Serial.print(ffTime - detTime);
    Serial.print(F(": "));
    SerialStat(intr);
   }
#endif


   uint8_t value = readRegister(REG_PKT_SNR_VALUE);
   if ( value & 0x80 ) {

    value = ( ( ~value + 1 ) & 0xFF ) >> 2;
    LoraUp.snr = -value;
   }
   else {

    LoraUp.snr = ( value & 0xFF ) >> 2;
   }


   LoraUp.prssi = readRegister(REG_PKT_RSSI);


   if (sx1272) {
    LoraUp.rssicorr = 139;
   } else {
    LoraUp.rssicorr = 157;
   }

   LoraUp.sf = readRegister(REG_MODEM_CONFIG2) >> 4;



   if (receivePacket() <= 0) {
#if DUSB>=1
    if (( debug>=0 ) && ( pdebug & P_RX )) {
     Serial.println(F("sMach:: Error receivePacket"));
    }
#endif
   }



   if ((cadGet()) || (_hop)) {
    _state = S_SCAN;
    global_sf = SF7;
    cadScanner();
   }
   else {
    _state = S_RX;
    rxLoraModem();
   }

   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
   eventTime=micros();
   _event=0;
  }
# 693 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_stateMachine.ino"
  else if (intr & IRQ_LORA_RXTOUT_MASK) {



   _event=0;
   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00 );
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);




   if ((cadGet()) || (_hop)) {

#if DUSB>=1
    if (( debug>=2 ) && ( pdebug & P_RX )) {
     Serial.print(F("RXTOUT:: "));
     SerialStat(intr);
    }
#endif
    global_sf = SF7;
    cadScanner();
    _state = S_SCAN;

   }



   else {
    _state = S_RX;
    rxLoraModem();
   }

   eventTime=micros();
   doneTime = micros();

  }

  else if (intr & IRQ_LORA_HEADER_MASK) {



#if DUSB>=1
   if (( debug>=3 ) && ( pdebug & P_RX )) {
    Serial.print(F("RX HEADER:: "));
    SerialStat(intr);
   }
#endif

  }





  else if (intr == 0x00) {
#if DUSB>=1
   if (( debug>=3) && ( pdebug & P_RX )) {
    Serial.print(F("S_RX no INTR:: "));
    SerialStat(intr);
   }
#endif
  }




  else {
#if DUSB>=1
   if (( debug>=0 ) && ( pdebug & P_RX )) {
    Serial.print(F("S_RX:: no RXDONE, RXTOUT, HEADER:: "));
    SerialStat(intr);
   }
#endif


  }

   break;
# 780 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_stateMachine.ino"
   case S_TX:




  if (intr == 0x00) {
#if DUSB>=1
   if (( debug>=2 ) && ( pdebug & P_TX )) {
    Serial.println(F("TX:: 0x00"));
   }
#endif
   _event=1;
   _state=S_TXDONE;
  }


  _state = S_TXDONE;


  writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
  writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);



  txLoraModem(
   LoraDown.payLoad,
   LoraDown.payLength,
   LoraDown.tmst,
   LoraDown.sfTx,
   LoraDown.powe,
   LoraDown.fff,
   LoraDown.crc,
   LoraDown.iiq,
   LoraDown.bw
  );


#if DUSB>=1
  if (( debug>=1 ) && ( pdebug & P_TX )) {
   Serial.print(F("T TX done:: "));
   SerialStat(intr);
  }
#endif

  _state=S_TXDONE;
  _event=1;

   break;
# 838 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_stateMachine.ino"
   case S_TXDONE:
  if (intr & IRQ_LORA_TXDONE_MASK) {

#if DUSB>=1
   if (( debug>=0 ) && ( pdebug & P_TX )) {
    Serial.print(F("T TXDONE:: rcvd="));
    Serial.print(micros());
    Serial.print(F(", diff="));
    Serial.println(micros()-LoraDown.tmst);
    if (debug>=2) Serial.flush();
   }
#endif

   if ((cadGet()) || (_hop)) {

    _state = S_SCAN;
    global_sf = SF7;
    cadScanner();
   }
   else {
    _state = S_RX;
    rxLoraModem();
   }

   _event=0;
   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
#if DUSB>=1
   if (( debug>=1 ) && ( pdebug & P_TX )) {
    Serial.println(F("T TXDONE:: done OK"));
   }
#endif
  }


  else if ( intr != 0 ) {
#if DUSB>=1
   if (( debug>=0 ) && ( pdebug & P_TX )) {
    Serial.print(F("T TXDONE:: unknown int:"));
    SerialStat(intr);
   }
#endif
   writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
   writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
   _event=0;
   _state=S_SCAN;
  }


  else {




   if ( sendTime > micros() ) sendTime = 0;
   if (( _state == S_TXDONE ) && (( micros() - sendTime) > 7000000 )) {
#if DUSB>=1
    if (( debug>=1 ) && ( pdebug & P_TX )) {
     Serial.println(F("T TXDONE:: reset TX"));
     Serial.flush();
    }
#endif
    startReceiver();
   }
#if DUSB>=1
   if (( debug>=3 ) && ( pdebug & P_TX )) {
    Serial.println(F("T TXDONE:: No Interrupt"));
   }
#endif
  }


   break;






   default:
#if DUSB>=1
  if (( debug>=0) && ( pdebug & P_PRE )) {
   Serial.print("ERR state=");
   Serial.println(_state);
  }
#endif
  if ((cadGet()) || (_hop)) {
#if DUSB>=1
   if (debug>=0) {
    Serial.println(F("default:: Unknown _state "));
    SerialStat(intr);
   }
#endif
   _state = S_SCAN;
   global_sf = SF7;
   cadScanner();
   _event=0;
  }
  else
  {
   _state = S_RX;
   rxLoraModem();
   _event=0;
  }
  writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) 0x00);
  writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF);
  eventTime=micros();

   break;
 }

 return;
}
# 1 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
# 22 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
#include <Base64.h>
#include <AES-128_V10.h>
#include "_loraModem.h"
#include "settings.h"

void fakeLoraWanWrap(uint8_t *message, uint8_t *messageLen)
{
 uint8_t mhdr = 0x40;
 uint8_t fhdr[7];

 fhdr[0] = message[13];
 fhdr[1] = message[12];
 fhdr[2] = message[11];
 fhdr[3] = message[2];

 fhdr[4] = 0x00;

 fhdr[5] = message[14];
 fhdr[6] = message[15];

 uint16_t FCount = (((uint16_t)fhdr[6]) << 8) | ((uint16_t)fhdr[5]);
 Serial.print("FCount ");
 Serial.println(FCount);
 uint8_t fport = 0x10;
 uint8_t mic[4] = {0x00, 0x00, 0x00, 0x00};

 uint8_t temp_buffer[100];

 uint8_t AppSKey[] = _APPSKEY;
 uint8_t NwkSKey[] = _NWKSKEY;
 uint8_t DevAddr[4];
 DevAddr[0] = message[2];
 DevAddr[1] = message[11];
 DevAddr[2] = message[12];
 DevAddr[3] = message[13];

 uint8_t CodeLength = encodePacket((uint8_t *)(message), *messageLen, FCount, DevAddr, AppSKey, 0);

 temp_buffer[0] = mhdr;
 memcpy(&temp_buffer[1], fhdr, 7);
 temp_buffer[8] = fport;
 memcpy(&temp_buffer[9], message, CodeLength);

 (*messageLen) = CodeLength + 9;


 (*messageLen) += micPacket((uint8_t *)(temp_buffer), *messageLen, FCount, DevAddr, NwkSKey, 0);


 memcpy(message, temp_buffer, *messageLen);
}
# 86 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
int sendPacket(uint8_t *buf, uint8_t length)
{
# 112 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
 int i = 0;
 StaticJsonDocument<312> jsonBuffer;
 char *bufPtr = (char *)(buf);
 buf[length] = 0;

#if DUSB >= 1
 if (debug >= 2)
 {
  Serial.println((char *)buf);
  Serial.print(F("<"));
  Serial.flush();
 }
#endif


 auto error = deserializeJson(jsonBuffer, bufPtr);

 if (error)
 {
#if DUSB >= 1
  if ((debug >= 1) && (pdebug & P_TX))
  {
   Serial.print(F("T sendPacket:: ERROR Json Decode"));
   if (debug >= 2)
   {
    Serial.print(':');
    Serial.println(bufPtr);
   }
   Serial.flush();
  }
#endif
  return (-1);
 }
 yield();





 JsonObject root = jsonBuffer.as<JsonObject>();
 const char *data = root["txpk"]["data"];
 uint8_t psize = root["txpk"]["size"];
 bool ipol = root["txpk"]["ipol"];
 uint8_t powe = root["txpk"]["powe"];
 LoraDown.tmst = (uint32_t)root["txpk"]["tmst"].as<unsigned long>();
 const float ff = root["txpk"]["freq"];


 const char *datr = root["txpk"]["datr"];
 const char *modu = root["txpk"]["modu"];
 const char *codr = root["txpk"]["codr"];




 if (data != NULL)
 {
#if DUSB >= 1
  if ((debug >= 2) && (pdebug & P_TX))
  {
   Serial.print(F("T data: "));
   Serial.println((char *)data);
   if (debug >= 2)
    Serial.flush();
  }
#endif
 }
 else
 {
#if DUSB >= 1
  if ((debug > 0) && (pdebug & P_TX))
  {
   Serial.println(F("T sendPacket:: ERROR: data is NULL"));
   if (debug >= 2)
    Serial.flush();
  }
#endif
  return (-1);
 }

 LoraDown.sfTx = atoi(datr + 2);
 if (datr[3] == 'B')
 {
  LoraDown.bw = atoi(datr + 5);
 }
 else if (datr[4] == 'B')
 {
  LoraDown.bw = atoi(datr + 6);
 }
 else
 {
  LoraDown.bw = 125;
 }

 LoraDown.iiq = (ipol ? 0x40 : 0x27);
 LoraDown.crc = 0x00;
 LoraDown.payLength = Base64.decodedLength((char *)data, strlen(data));
 Base64.decode((char *)payLoad, (char *)data, strlen(data));


 uint32_t w = (uint32_t)(LoraDown.tmst - micros());







#if _STRICT_1CH == 1



 if ((w > 1000000) && (w < 3000000))
 {
  LoraDown.tmst -= 1000000;
 }
 else if ((w > 6000000) && (w < 7000000))
 {
  LoraDown.tmst -= 500000;
 }
 LoraDown.powe = 14;

 LoraDown.fff = freq;
#else
 LoraDown.powe = powe;


 LoraDown.fff = (uint32_t)((uint32_t)((ff + 0.000035) * 1000)) * 1000;
#endif

 LoraDown.payLoad = payLoad;

#if DUSB >= 1
 if ((debug >= 1) && (pdebug & P_TX))
 {

  Serial.print(F("T LoraDown tmst="));
  Serial.print(LoraDown.tmst);



  if (debug >= 2)
  {
   Serial.print(F(" Request:: "));
   Serial.print(F(" tmst="));
   Serial.print(LoraDown.tmst);
   Serial.print(F(" wait="));
   Serial.println(w);

   Serial.print(F(" strict="));
   Serial.print(_STRICT_1CH);
   Serial.print(F(" datr="));
   Serial.println(datr);
   Serial.print(F(" Rfreq="));
   Serial.print(freq);
   Serial.print(F(", Request="));
   Serial.print(freq);
   Serial.print(F(" ->"));
   Serial.println(LoraDown.fff);
   Serial.print(F(" sf  ="));
   Serial.print(atoi(datr + 2));
   Serial.print(F(" ->"));
   Serial.println(LoraDown.sfTx);

   Serial.print(F(" modu="));
   Serial.println(modu);
   Serial.print(F(" powe="));
   Serial.println(powe);
   Serial.print(F(" codr="));
   Serial.println(codr);

   Serial.print(F(" ipol="));
   Serial.println(ipol);
  }
  Serial.println();
 }
#endif

 if (LoraDown.payLength != psize)
 {
#if DUSB >= 1
  Serial.print(F("sendPacket:: WARNING payLength: "));
  Serial.print(LoraDown.payLength);
  Serial.print(F(", psize="));
  Serial.println(psize);
  if (debug >= 2)
   Serial.flush();
#endif
 }
#if DUSB >= 1
 else if ((debug >= 2) && (pdebug & P_TX))
 {
  Serial.print(F("T Payload="));
  for (i = 0; i < LoraDown.payLength; i++)
  {
   Serial.print(payLoad[i], HEX);
   Serial.print(':');
  }
  Serial.println();
  if (debug >= 2)
   Serial.flush();
 }
#endif
 cp_up_pkt_fwd++;

#if DUSB >= 1
 if ((debug >= 2) && (pdebug & P_TX))
 {
  Serial.println(F("T sendPacket:: fini OK"));
 }
#endif



 _state = S_TX;

 return 1;
}
# 346 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
int buildPacket(uint32_t tmst, uint8_t *buff_up, struct LoraUp LoraUp, bool internal)
{
 long SNR;
 int rssicorr;
 int prssi;

 char cfreq[12] = {0};

 int buff_index = 0;
 char b64[256];

 uint8_t *message = LoraUp.payLoad;
 char messageLength = LoraUp.payLength;


 Serial.println("RX lora");
 for (uint8_t i = 0; i < LoraUp.payLength; i++)
 {
  if (message[i] < 0x10)
  {
   Serial.print(0, HEX);
  }
  Serial.print(message[i], HEX);
  Serial.print(" ");
 }

 Serial.println("");

 Serial.print("LoraUp Length ");
 Serial.println((int)messageLength);
 Serial.print("Calculed Length ");
 Serial.println(message[19] + 20);
 if ((messageLength == (message[19] + 20)))
 {

  Serial.println("Custom agrotools");
  fakeLoraWanWrap(message, (uint8_t *)&messageLength);

  Serial.println("RX fake LoraWan wrap");
  for (uint8_t i = 0; i < messageLength; i++)
  {
   if (message[i] < 0x10)
   {
    Serial.print(0, HEX);
   }
   Serial.print(message[i], HEX);
   Serial.print(" ");
  }

  Serial.println("");
 }
 else
 {

 }

#if _CHECK_MIC == 1
 unsigned char NwkSKey[16] = _NWKSKEY;
 checkMic(message, messageLength, NwkSKey);
#endif



 if (internal)
 {
  SNR = 12;
  prssi = 50;
  rssicorr = 157;
 }
 else
 {
  SNR = LoraUp.snr;
  prssi = LoraUp.prssi;
  rssicorr = LoraUp.rssicorr;
 }

#if STATISTICS >= 1




 for (int m = (MAX_STAT - 1); m > 0; m--)
  statr[m] = statr[m - 1];


#if _LOCALSERVER == 1
 statr[0].datal = 0;
 int index;
 if ((index = inDecodes((char *)(LoraUp.payLoad + 1))) >= 0)
 {

  uint16_t frameCount = LoraUp.payLoad[7] * 256 + LoraUp.payLoad[6];

  for (int k = 0; (k < LoraUp.payLength) && (k < 23); k++)
  {
   statr[0].data[k] = LoraUp.payLoad[k + 9];
  };




  uint8_t DevAddr[4];
  DevAddr[0] = LoraUp.payLoad[4];
  DevAddr[1] = LoraUp.payLoad[3];
  DevAddr[2] = LoraUp.payLoad[2];
  DevAddr[3] = LoraUp.payLoad[1];

  statr[0].datal = encodePacket((uint8_t *)(statr[0].data),
           LoraUp.payLength - 9 - 4,
           (uint16_t)frameCount,
           DevAddr,
           decodes[index].appKey,
           0);
 }
#endif
 statr[0].tmst = now();
 statr[0].ch = ifreq;
 statr[0].prssi = prssi - rssicorr;
#if RSSI == 1
 statr[0].rssi = _rssi - rssicorr;
#endif
 statr[0].sf = LoraUp.sf;
#if DUSB >= 2
 if (debug >= 0)
 {
  if ((message[4] != 0x26) || (message[1] == 0x99))
  {
   Serial.print(F("addr="));
   for (int i = messageLength; i > 0; i--)
   {
    if (message[i] < 0x10)
     Serial.print('0');
    Serial.print(message[i], HEX);
    Serial.print(' ');
   }
   Serial.println();
  }
 }
#endif
 statr[0].node = (message[1] << 24 | message[2] << 16 | message[3] << 8 | message[4]);

#if STATISTICS >= 2


 switch (statr[0].sf)
 {
 case SF7:
  statc.sf7++;
  break;
 case SF8:
  statc.sf8++;
  break;
 case SF9:
  statc.sf9++;
  break;
 case SF10:
  statc.sf10++;
  break;
 case SF11:
  statc.sf11++;
  break;
 case SF12:
  statc.sf12++;
  break;
 }
#endif

#if STATISTICS >= 3
 if (statr[0].ch == 0)
  switch (statr[0].sf)
  {
  case SF7:
   statc.sf7_0++;
   break;
  case SF8:
   statc.sf8_0++;
   break;
  case SF9:
   statc.sf9_0++;
   break;
  case SF10:
   statc.sf10_0++;
   break;
  case SF11:
   statc.sf11_0++;
   break;
  case SF12:
   statc.sf12_0++;
   break;
  }
 else if (statr[0].ch == 1)
  switch (statr[0].sf)
  {
  case SF7:
   statc.sf7_1++;
   break;
  case SF8:
   statc.sf8_1++;
   break;
  case SF9:
   statc.sf9_1++;
   break;
  case SF10:
   statc.sf10_1++;
   break;
  case SF11:
   statc.sf11_1++;
   break;
  case SF12:
   statc.sf12_1++;
   break;
  }
 else if (statr[0].ch == 2)
  switch (statr[0].sf)
  {
  case SF7:
   statc.sf7_2++;
   break;
  case SF8:
   statc.sf8_2++;
   break;
  case SF9:
   statc.sf9_2++;
   break;
  case SF10:
   statc.sf10_2++;
   break;
  case SF11:
   statc.sf11_2++;
   break;
  case SF12:
   statc.sf12_2++;
   break;
  }
#endif

#endif

#if DUSB >= 1
 if ((debug >= 2) && (pdebug & P_RADIO))
 {
  Serial.print(F("R buildPacket:: pRSSI="));
  Serial.print(prssi - rssicorr);
  Serial.print(F(" RSSI: "));
  Serial.print(_rssi - rssicorr);
  Serial.print(F(" SNR: "));
  Serial.print(SNR);
  Serial.print(F(" Length: "));
  Serial.print((int)messageLength);
  Serial.print(F(" -> "));
  int i;
  for (i = 0; i < messageLength; i++)
  {
   Serial.print(message[i], HEX);
   Serial.print(' ');
  }
  Serial.println();
  yield();
 }
#endif


#if OLED >= 1
 char timBuff[20];
 sprintf(timBuff, "%02i:%02i:%02i", hour(), minute(), second());

 display.clear();
 display.setFont(ArialMT_Plain_16);
 display.setTextAlignment(TEXT_ALIGN_LEFT);



 display.drawString(0, 0, "Time: ");
 display.drawString(40, 0, timBuff);

 display.drawString(0, 16, "RSSI: ");
 display.drawString(40, 16, String(prssi - rssicorr));

 display.drawString(70, 16, ",SNR: ");
 display.drawString(110, 16, String(SNR));

 display.drawString(0, 32, "Addr: ");

 if (message[4] < 0x10)
  display.drawString(40, 32, "0" + String(message[4], HEX));
 else
  display.drawString(40, 32, String(message[4], HEX));
 if (message[3] < 0x10)
  display.drawString(61, 32, "0" + String(message[3], HEX));
 else
  display.drawString(61, 32, String(message[3], HEX));
 if (message[2] < 0x10)
  display.drawString(82, 32, "0" + String(message[2], HEX));
 else
  display.drawString(82, 32, String(message[2], HEX));
 if (message[1] < 0x10)
  display.drawString(103, 32, "0" + String(message[1], HEX));
 else
  display.drawString(103, 32, String(message[1], HEX));

 display.drawString(0, 48, "LEN: ");
 display.drawString(40, 48, String((int)messageLength));
 display.display();


#endif

 int j;




 int encodedLen = Base64.encodedLength(messageLength);
#if DUSB >= 1
 if ((debug >= 1) && (encodedLen > 255) && (pdebug & P_RADIO))
 {
  Serial.print(F("R buildPacket:: b64 err, len="));
  Serial.println(encodedLen);
  if (debug >= 2)
   Serial.flush();
  return (-1);
 }
#endif
 Base64.encode(b64, (char *)message, messageLength);

 uint8_t token_h = (uint8_t)rand();
 uint8_t token_l = (uint8_t)rand();


 buff_up[0] = PROTOCOL_VERSION;

 buff_up[1] = token_h;
 buff_up[2] = token_l;

 buff_up[3] = PKT_PUSH_DATA;


 buff_up[4] = MAC_array[0];
 buff_up[5] = MAC_array[1];
 buff_up[6] = MAC_array[2];
 buff_up[7] = 0xFF;
 buff_up[8] = 0xFF;
 buff_up[9] = MAC_array[3];
 buff_up[10] = MAC_array[4];
 buff_up[11] = MAC_array[5];

 buff_index = 12;


 memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[", 9);
 buff_index += 9;
 buff_up[buff_index] = '{';
 ++buff_index;
 j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index, "\"tmst\":%u", tmst);
#if DUSB >= 1
 if ((j < 0) && (debug >= 1) && (pdebug & P_RADIO))
 {
  Serial.println(F("buildPacket:: Error "));
 }
#endif
 buff_index += j;
 ftoa(((double)freq) / 1000000.0, cfreq, 6);
 j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index, ",\"chan\":%1u,\"rfch\":%1u,\"freq\":%s", 0, 0, cfreq);
 buff_index += j;
 memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
 buff_index += 9;
 memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
 buff_index += 14;


 switch (LoraUp.sf)
 {
 case SF6:
  memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF6", 12);
  buff_index += 12;
  break;
 case SF7:
  memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF7", 12);
  buff_index += 12;
  break;
 case SF8:
  memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF8", 12);
  buff_index += 12;
  break;
 case SF9:
  memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF9", 12);
  buff_index += 12;
  break;
 case SF10:
  memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF10", 13);
  buff_index += 13;
  break;
 case SF11:
  memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF11", 13);
  buff_index += 13;
  break;
 case SF12:
  memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF12", 13);
  buff_index += 13;
  break;
 default:
  memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF?", 12);
  buff_index += 12;
 }
 memcpy((void *)(buff_up + buff_index), (void *)"BW125\"", 6);
 buff_index += 6;
 memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/5\"", 13);
 buff_index += 13;
 j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index, ",\"lsnr\":%li", SNR);
 buff_index += j;
 j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index, ",\"rssi\":%d,\"size\":%u", prssi - rssicorr, messageLength);
 buff_index += j;
 memcpy((void *)(buff_up + buff_index), (void *)",\"data\":\"", 9);
 buff_index += 9;


 encodedLen = Base64.encodedLength(messageLength);
 j = Base64.encode((char *)(buff_up + buff_index), (char *)message, messageLength);

 buff_index += j;
 buff_up[buff_index] = '"';
 ++buff_index;


 buff_up[buff_index] = '}';
 ++buff_index;
 buff_up[buff_index] = ']';
 ++buff_index;


 buff_up[buff_index] = '}';
 ++buff_index;
 buff_up[buff_index] = 0;

#if STAT_LOG == 1



 addLog((unsigned char *)(buff_up), buff_index);
#endif

#if DUSB >= 1
 if ((debug >= 2) && (pdebug & P_RX))
 {
  Serial.print(F("R RXPK:: "));
  Serial.println((char *)(buff_up + 12));
  Serial.print(F("R RXPK:: package length="));
  Serial.println(buff_index);
 }
#endif
 return (buff_index);
}
# 810 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
int receivePacket()
{
 uint8_t buff_up[TX_BUFF_SIZE];
 long SNR;
 uint8_t message[128] = {0x00};
 uint8_t messageLength = 0;
# 824 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
 uint32_t tmst = (uint32_t)micros();


 LoraUp.rx_local_timestamp = tmst;

 if (LoraUp.payLength > 0)
 {


  int build_index = buildPacket(tmst, buff_up, LoraUp, false);




#if REPEATER == 1
  if (!sendLora(LoraUp.payLoad, LoraUp.payLength))
  {
   return (-3);
  }
#endif




#ifdef _TTNSERVER


  S_PROTOCOL proto = settings_protocol();
  if (proto == MQTTBRIDGE_TCP)
  {
   mqtt_sendUplink(LoraUp);
  }
  else if (proto == SEMTECH_PF_UDP)
  {
   if (!sendUdp(ttnServer, _TTNPORT, buff_up, build_index))
   {
    return (-1);
   }
  }

  yield();
#endif

#ifdef _THINGSERVER
  if (!sendUdp(thingServer, _THINGPORT, buff_up, build_index))
  {
   return (-2);
  }
#endif

#if _LOCALSERVER == 1
# 897 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
  int index = 0;
  if ((index = inDecodes((char *)(LoraUp.payLoad + 1))) >= 0)
  {

   uint8_t DevAddr[4];
   DevAddr[0] = LoraUp.payLoad[4];
   DevAddr[1] = LoraUp.payLoad[3];
   DevAddr[2] = LoraUp.payLoad[2];
   DevAddr[3] = LoraUp.payLoad[1];
   uint16_t frameCount = LoraUp.payLoad[7] * 256 + LoraUp.payLoad[6];

#if DUSB >= 1
   if ((debug >= 1) && (pdebug & P_RX))
   {
    Serial.print(F("R receivePacket:: Ind="));
    Serial.print(index);
    Serial.print(F(", Len="));
    Serial.print(LoraUp.payLength);
    Serial.print(F(", A="));
    for (int i = 0; i < 4; i++)
    {
     if (DevAddr[i] < 0x0F)
      Serial.print('0');
     Serial.print(DevAddr[i], HEX);

    }

    Serial.print(F(", Msg="));
    for (int i = 0; (i < statr[0].datal) && (i < 23); i++)
    {
     if (statr[0].data[i] < 0x0F)
      Serial.print('0');
     Serial.print(statr[0].data[i], HEX);
     Serial.print(' ');
    }
    Serial.println();
   }
  }
  else if ((debug >= 2) && (pdebug & P_RX))
  {
   Serial.println(F("receivePacket:: No Index"));
  }
#endif
#endif


  LoraUp.payLength = 0;
  LoraUp.payLoad[0] = 0x00;
  return (build_index);
 }

 return (0);

}
# 976 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
uint8_t micPacket(uint8_t *data, uint8_t len, uint16_t FrameCount, uint8_t * DevAddr, uint8_t * NwkSKey, uint8_t dir) {



 uint8_t Block_B[16];
 uint8_t X[16];
 uint8_t Y[16];



 Block_B[0]= 0x49;

 Block_B[1]= 0x00;
 Block_B[2]= 0x00;
 Block_B[3]= 0x00;
 Block_B[4]= 0x00;

 Block_B[5]= dir;

 Block_B[6]= DevAddr[3];
 Block_B[7]= DevAddr[2];
 Block_B[8]= DevAddr[1];
 Block_B[9]= DevAddr[0];

 Block_B[10]= (FrameCount & 0x00FF);
 Block_B[11]= ((FrameCount >> 8) & 0x00FF);
 Block_B[12]= 0x00;
 Block_B[13]= 0x00;

 Block_B[14]= 0x00;

 Block_B[15]= len;




 uint8_t k1[16];
 uint8_t k2[16];
 generate_subkey(NwkSKey, k1, k2);




 uint8_t micBuf[len+16];
 for (uint8_t i=0; i<16; i++) micBuf[i]=Block_B[i];
 for (uint8_t i=0; i<len; i++) micBuf[i+16]=data[i];




 uint8_t numBlocks = len/16 + 1;
 if ((len % 16)!=0) numBlocks++;




 uint8_t restBits = len%16;





 memset(X, 0, 16);




 for(uint8_t i= 0x0; i < (numBlocks - 1); i++) {
  for (uint8_t j=0; j<16; j++) Y[j] = micBuf[(i*16)+j];
  mXor(Y, X);
  AES_Encrypt(Y, NwkSKey);
  for (uint8_t j=0; j<16; j++) X[j] = Y[j];
 }







 if (restBits) {
  for (uint8_t i=0; i<16; i++) {
   if (i< restBits) Y[i] = micBuf[((numBlocks-1)*16)+i];
   if (i==restBits) Y[i] = 0x80;
   if (i> restBits) Y[i] = 0x00;
  }
  mXor(Y, k2);
 }
 else {
  for (uint8_t i=0; i<16; i++) {
   Y[i] = micBuf[((numBlocks-1)*16)+i];
  }
  mXor(Y, k1);
 }
 mXor(Y, X);
 AES_Encrypt(Y,NwkSKey);






 data[len+0]=Y[0];
 data[len+1]=Y[1];
 data[len+2]=Y[2];
 data[len+3]=Y[3];
 return 4;
}
# 1109 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
uint8_t encodePacket(uint8_t *Data, uint8_t DataLength, uint16_t FrameCount, uint8_t *DevAddr, uint8_t *AppSKey, uint8_t Direction) {

#if DUSB>=1
 if (( debug>=2 ) && ( pdebug & P_GUI )) {
  Serial.print(F("G encodePacket:: DevAddr="));
  for (int i=0; i<4; i++ ) { Serial.print(DevAddr[i],HEX); Serial.print(' '); }
  Serial.print(F("G encodePacket:: AppSKey="));
  for (int i=0; i<16; i++ ) { Serial.print(AppSKey[i],HEX); Serial.print(' '); }
  Serial.println();
 }
#endif


 uint8_t i, j;
 uint8_t Block_A[16];
 uint8_t bLen=16;

 uint8_t restLength = DataLength % 16;
 uint8_t numBlocks = DataLength / 16;
 if (restLength>0) numBlocks++;

 for(i = 1; i <= numBlocks; i++) {
  Block_A[0] = 0x01;

  Block_A[1] = 0x00;
  Block_A[2] = 0x00;
  Block_A[3] = 0x00;
  Block_A[4] = 0x00;

  Block_A[5] = Direction;

  Block_A[6] = DevAddr[3];
  Block_A[7] = DevAddr[2];
  Block_A[8] = DevAddr[1];
  Block_A[9] = DevAddr[0];

  Block_A[10] = (FrameCount & 0x00FF);
  Block_A[11] = ((FrameCount >> 8) & 0x00FF);
  Block_A[12] = 0x00;
  Block_A[13] = 0x00;

  Block_A[14] = 0x00;

  Block_A[15] = i;


  AES_Encrypt(Block_A, AppSKey);


  if ((i == numBlocks) && (restLength>0)) bLen = restLength;

  for(j = 0; j < bLen; j++) {
   *Data = *Data ^ Block_A[j];
   Data++;
  }
 }

 return(DataLength);
}





static void generate_subkey(uint8_t *key, uint8_t *k1, uint8_t *k2) {

 memset(k1, 0, 16);


 AES_Encrypt(k1,key);


 if (k1[0] & 0x80) {
  shift_left(k1,16);
  k1[15] ^= 0x87;
 }
 else {
  shift_left(k1,16);
 }


 for (uint8_t i=0; i<16; i++) k2[i]=k1[i];
 if (k1[0] & 0x80) {
  shift_left(k2,16);
  k2[15] ^= 0x87;
 }
 else {
  shift_left(k2,16);
 }


 return;
}
# 1211 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
static void mXor(uint8_t *buf, uint8_t *key) {
 for (uint8_t i = 0; i < 16; ++i) buf[i] ^= key[i];
}
# 1223 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_txRx.ino"
static void shift_left(uint8_t * buf, uint8_t len) {
    while (len--) {
        uint8_t next = len ? buf[1] : 0;

        uint8_t val = (*buf << 1);
        if (next & 0x80) val |= 0x01;
        *buf++ = val;
    }
}
# 1 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_wwwServer.ino"
# 43 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_wwwServer.ino"
static void printIP(IPAddress ipa, const char sep, String& response)
{
 response+=ipa[0]; response+=sep;
 response+=ipa[1]; response+=sep;
 response+=ipa[2]; response+=sep;
 response+=ipa[3];
}




#if A_SERVER==1
# 79 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_wwwServer.ino"
boolean YesNo()
{
 boolean ret = false;
 String response = "";
 response += "<script>";

 response += "var ch = \"\"; ";
 response += "function ynDialog(s,y) {";
 response += "  try { adddlert(s); }";
 response += "  catch(err) {";
 response += "    ch  = \" \" + s + \".\\n\\n\"; ";
 response += "    ch += \"Click OK to continue,\\n\"; ";
 response += "    ch += \"or Cancel\\n\\n\"; ";
 response += "    if(!confirm(ch)) { ";
 response += "      javascript:window.location.reload(true);";
 response += "    } ";
 response += "    else { ";
 response += "      document.location.href = '/'+y; ";
 response += "    } ";
 response += "  }";
 response += "}";
 response += "</script>";
 server.sendContent(response);




 return(ret);
}
# 120 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_wwwServer.ino"
void wwwFile(String fn) {

 if (!SPIFFS.exists(fn)) {
#if DUSB>=1
  Serial.print(F("wwwFile:: ERROR: file not found="));
  Serial.println(fn);
#endif
  return;
 }
#if DUSB>=2
 else {
  Serial.print(F("wwwFile:: File existist= "));
  Serial.println(fn);
 }
#endif

#if DUSB>=1
  File f = SPIFFS.open(fn, "r");

  int j;
  for (j=0; j<LOGFILEREC; j++) {

   String s=f.readStringUntil('\n');
   if (s.length() == 0) {
    Serial.print(F("wwwFile:: String length 0"));
    break;
   }
   server.sendContent(s.substring(12));
   server.sendContent("\n");
   yield();
  }
#endif

}





void buttonDocu()
{

 String response = "";
 response += "<script>";

 response += "var txt = \"\";";
 response += "function showDocu() {";
 response += "  try { adddlert(\"Welcome,\"); }";
 response += "  catch(err) {";
 response += "    txt  = \"Do you want the documentation page.\\n\\n\"; ";
 response += "    txt += \"Click OK to continue viewing documentation,\\n\"; ";
 response += "    txt += \"or Cancel to return to the home page.\\n\\n\"; ";
 response += "    if(confirm(txt)) { ";
 response += "      document.location.href = \"https://things4u.github.io/UserGuide/One%20Channel%20Gateway/Introduction%205.html\"; ";
 response += "    }";
 response += "  }";
 response += "}";

 response += "</script>";
 server.sendContent(response);
}
# 189 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_wwwServer.ino"
void buttonLog()
{

 String response = "";
 String fn = "";
 int i = 0;

 while (i< LOGFILEMAX ) {
  fn = "/log-" + String(gwayConfig.logFileNo - i);
  wwwFile(fn);
  i++;
 }

 server.sendContent(response);
}






static void wwwButtons()
{
 String response = "";
 String mode = (gwayConfig.expert ? "Basic Mode" : "Expert Mode");

 YesNo();
 buttonDocu();

 response += "<input type=\"button\" value=\"Documentation\" onclick=\"showDocu()\" >";

 response += "<a href=\"EXPERT\" download><button type=\"button\">" + mode + "</button></a>";

 response += "<a href=\"LOG\" download><button type=\"button\">Log Files</button></a>";

 server.sendContent(response);
}
# 242 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_wwwServer.ino"
static void setVariables(const char *cmd, const char *arg) {


 if (strcmp(cmd, "DEBUG")==0) {
  if (atoi(arg) == 1) {
   debug = (debug+1)%4;
  }
  else if (atoi(arg) == -1) {
   debug = (debug+3)%4;
  }
  writeGwayCfg(CONFIGFILE);
 }

 if (strcmp(cmd, "CAD")==0) {
  _cad=(bool)atoi(arg);
  writeGwayCfg(CONFIGFILE);
 }

 if (strcmp(cmd, "HOP")==0) {
  _hop=(bool)atoi(arg);
  if (! _hop) {
   ifreq=0;
   freq=freqs[ifreq];
   rxLoraModem();
   sf = SF7;
   cadScanner();
  }
  writeGwayCfg(CONFIGFILE);
 }

 if (strcmp(cmd, "DELAY")==0) {
  txDelay+=atoi(arg)*1000;
 }


 if (strcmp(cmd, "SF")==0) {
  uint8_t sfi = sf;
  if (atoi(arg) == 1) {
   if (sf>=SF12) sf=SF7; else sf= (sf_t)((int)sf+1);
  }
  else if (atoi(arg) == -1) {
   if (sf<=SF7) sf=SF12; else sf= (sf_t)((int)sf-1);
  }
  rxLoraModem();
  writeGwayCfg(CONFIGFILE);
 }


 if (strcmp(cmd, "FREQ")==0) {
  uint8_t nf = sizeof(freqs)/sizeof(int);


  if (atoi(arg) == 1) {
   if (ifreq==(nf-1)) ifreq=0; else ifreq++;
  }
  else if (atoi(arg) == -1) {
   Serial.println("down");
   if (ifreq==0) ifreq=(nf-1); else ifreq--;
  }

  freq = freqs[ifreq];
  rxLoraModem();
  writeGwayCfg(CONFIGFILE);
 }





 if (strcmp(cmd, "HELP")==0) { Serial.println(F("Display Help Topics")); }

#if GATEWAYNODE==1
 if (strcmp(cmd, "NODE")==0) {
  gwayConfig.isNode =(bool)atoi(arg);
  writeGwayCfg(CONFIGFILE);
 }

 if (strcmp(cmd, "FCNT")==0) {
  frameCount=0;
  rxLoraModem();
  writeGwayCfg(CONFIGFILE);
 }
#endif

#if WIFIMANAGER==1
 if (strcmp(cmd, "NEWSSID")==0) {
  WiFiManager wifiManager;
  strcpy(wpa[0].login,"");
  strcpy(wpa[0].passw,"");
  WiFi.disconnect();
  wifiManager.autoConnect(AP_NAME, AP_PASSWD );
 }
#endif

#if A_OTA==1
 if (strcmp(cmd, "UPDATE")==0) {
  if (atoi(arg) == 1) {
   updateOtaa();
  }
 }
#endif

#if A_REFRESH==1
 if (strcmp(cmd, "REFR")==0) {
  gwayConfig.refresh =(bool)atoi(arg);
  writeGwayCfg(CONFIGFILE);
 }
#endif

}







static void openWebPage()
{
 ++gwayConfig.views;
#if A_REFRESH==1

#endif
 String response="";

 server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
 server.sendHeader("Pragma", "no-cache");
 server.sendHeader("Expires", "-1");




 server.setContentLength(CONTENT_LENGTH_UNKNOWN);
 server.send(200, "text/html", "");
#if A_REFRESH==1
 if (gwayConfig.refresh) {
  response += String() + "<!DOCTYPE HTML><HTML><HEAD><meta http-equiv='refresh' content='"+_WWW_INTERVAL+";http://";
  printIP((IPAddress)WiFi.localIP(),'.',response);
  response += "'><TITLE>ESP8266 1ch Gateway</TITLE>";
 }
 else {
  response += String() + "<!DOCTYPE HTML><HTML><HEAD><TITLE>ESP8266 1ch Gateway</TITLE>";
 }
#else
 response += String() + "<!DOCTYPE HTML><HTML><HEAD><TITLE>ESP8266 1ch Gateway</TITLE>";
#endif
 response += "<META HTTP-EQUIV='CONTENT-TYPE' CONTENT='text/html; charset=UTF-8'>";
 response += "<META NAME='AUTHOR' CONTENT='M. Westenberg (mw1554@hotmail.com)'>";

 response += "<style>.thead {background-color:green; color:white;} ";
 response += ".cell {border: 1px solid black;}";
 response += ".config_table {max_width:100%; min-width:400px; width:98%; border:1px solid black; border-collapse:collapse;}";
 response += "</style></HEAD><BODY>";

 response +="<h1>ESP Gateway Config</h1>";

 response +="<p style='font-size:10px;'>";
 response +="Version: "; response+=VERSION;
 response +="<br>ESP alive since ";
 stringTime(startTime, response);

 response +=", Uptime: ";
 uint32_t secs = millis()/1000;
 uint16_t days = secs / 86400;
 uint8_t _hour = hour(secs);
 uint8_t _minute = minute(secs);
 uint8_t _second = second(secs);
 response += String() + days + "-";
 if (_hour < 10) response += "0";
 response += String() + _hour + ":";
 if (_minute < 10) response += "0";
 response += String() + _minute + ":";
 if (_second < 10) response += "0";
 response += String() + _second;

 response +="<br>Current time    ";
 stringTime(now(), response);
 response +="<br>";
 response +="</p>";

 server.sendContent(response);
}







static void settingsData()
{
 String response="";
 String bg="";

 response +="<h2>Gateway Settings</h2>";

 response +="<table class=\"config_table\">";
 response +="<tr>";
 response +="<th class=\"thead\">Setting</th>";
 response +="<th colspan=\"2\" style=\"background-color: green; color: white; width:120px;\">Value</th>";
 response +="<th colspan=\"4\" style=\"background-color: green; color: white; width:100px;\">Set</th>";
 response +="</tr>";

 bg = " background-color: ";
 bg += ( _cad ? "LightGreen" : "orange" );
 response +="<tr><td class=\"cell\">CAD</td>";
 response +="<td colspan=\"2\" style=\"border: 1px solid black;"; response += bg; response += "\">";
 response += ( _cad ? "ON" : "OFF" );
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"CAD=1\"><button>ON</button></a></td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"CAD=0\"><button>OFF</button></a></td>";
 response +="</tr>";

 bg = " background-color: ";
 bg += ( _hop ? "LightGreen" : "orange" );
 response +="<tr><td class=\"cell\">HOP</td>";
 response +="<td colspan=\"2\" style=\"border: 1px solid black;"; response += bg; response += "\">";
 response += ( _hop ? "ON" : "OFF" );
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"HOP=1\"><button>ON</button></a></td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"HOP=0\"><button>OFF</button></a></td>";
 response +="</tr>";

 response +="<tr><td class=\"cell\">SF Setting</td><td class=\"cell\" colspan=\"2\">";
 if (_cad) {
  response += "AUTO</td>";
 }
 else {
  response += sf;
  response +="<td class=\"cell\"><a href=\"SF=-1\"><button>-</button></a></td>";
  response +="<td class=\"cell\"><a href=\"SF=1\"><button>+</button></a></td>";
 }
 response +="</tr>";


 response +="<tr><td class=\"cell\">Channel</td>";
 response +="<td class=\"cell\" colspan=\"2\">";
 if (_hop) {
  response += "AUTO</td>";
 }
 else {
  response += String() + ifreq;
  response +="</td>";
  response +="<td class=\"cell\"><a href=\"FREQ=-1\"><button>-</button></a></td>";
  response +="<td class=\"cell\"><a href=\"FREQ=1\"><button>+</button></a></td>";
 }
 response +="</tr>";



#if DUSB>=1
 response +="<tr><td class=\"cell\">Debug level</td><td class=\"cell\" colspan=\"2\">";
 response +=debug;
 response +="</td>";
 response +="<td class=\"cell\"><a href=\"DEBUG=-1\"><button>-</button></a></td>";
 response +="<td class=\"cell\"><a href=\"DEBUG=1\"><button>+</button></a></td>";
 response +="</tr>";

 response +="<tr><td class=\"cell\">Debug pattern</td>";

 bg = ( (pdebug & P_SCAN) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=SCAN\">";
 response +="<button>SCN</button></a></td>";

 bg = ( (pdebug & P_CAD) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=CAD\">";
 response +="<button>CAD</button></a></td>";

 bg = ( (pdebug & P_RX) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=RX\">";
 response +="<button>RX</button></a></td>";

 bg = ( (pdebug & P_TX) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=TX\">";
 response +="<button>TX</button></a></td>";
 response += "</tr>";


 response +="<tr><td class=\"cell\"></td>";
 bg = ( (pdebug & P_PRE) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=PRE\">";
 response +="<button>PRE</button></a></td>";

 bg = ( (pdebug & P_MAIN) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=MAIN\">";
 response +="<button>MAI</button></a></td>";

 bg = ( (pdebug & P_GUI) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=GUI\">";
 response +="<button>GUI</button></a></td>";

 bg = ( (pdebug & P_RADIO) ? "LightGreen" : "orange" );
 response +="<td class=\"cell\" style=\"border: 1px solid black; width:20px; background-color: ";
 response += bg; response += "\">";
 response +="<a href=\"PDEBUG=RADIO\">";
 response +="<button>RDIO</button></a></td>";
 response +="</tr>";
#endif

 response +="<tr><td class=\"cell\">Usb Debug</td><td class=\"cell\" colspan=\"2\">";
 response +=DUSB;
 response +="</td>";


 response +="</tr>";

#if GATEWAYNODE==1
 response +="<tr><td class=\"cell\">Framecounter Internal Sensor</td>";
 response +="<td class=\"cell\" colspan=\"2\">";
 response +=frameCount;
 response +="</td><td colspan=\"2\" style=\"border: 1px solid black;\">";
 response +="<button><a href=\"/FCNT\">RESET</a></button></td>";
 response +="</tr>";

 bg = " background-color: ";
 bg += ( (gwayConfig.isNode == 1) ? "LightGreen" : "orange" );
 response +="<tr><td class=\"cell\">Gateway Node</td>";
 response +="<td class=\"cell\" style=\"border: 1px solid black;" + bg + "\">";
 response += ( (gwayConfig.isNode == true) ? "ON" : "OFF" );
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"NODE=1\"><button>ON</button></a></td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"NODE=0\"><button>OFF</button></a></td>";
 response +="</tr>";
#endif

#if A_REFRESH==1
 bg = " background-color: ";
 bg += ( (gwayConfig.refresh == 1) ? "LightGreen" : "orange" );
 response +="<tr><td class=\"cell\">WWW Refresh</td>";
 response +="<td class=\"cell\" colspan=\"2\" style=\"border: 1px solid black; " + bg + "\">";
 response += ( (gwayConfig.refresh == 1) ? "ON" : "OFF" );
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"REFR=1\"><button>ON</button></a></td>";
 response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"REFR=0\"><button>OFF</button></a></td>";
 response +="</tr>";
#endif


#if WIFIMANAGER==1
 response +="<tr><td><tr><td>";
 response +="Click <a href=\"/NEWSSID\">here</a> to reset accesspoint<br>";
 response +="</td><td></td></tr>";
#endif


 response +="<tr><td class=\"cell\">Update Firmware</td><td colspan=\"2\"></td>";
 response +="<td class=\"cell\" colspan=\"2\" class=\"cell\"><a href=\"/UPDATE=1\"><button>UPDATE</button></a></td></tr>";


 response +="<tr><td class=\"cell\">Format SPIFFS</td>";
 response +=String() + "<td class=\"cell\" colspan=\"2\" >"+""+"</td>";
 response +="<td colspan=\"2\" class=\"cell\"><input type=\"button\" value=\"FORMAT\" onclick=\"ynDialog(\'Do you really want to format?\',\'FORMAT\')\" /></td></tr>";


#if STATISTICS >= 1
 response +="<tr><td class=\"cell\">Statistics</td>";
 response +=String() + "<td class=\"cell\" colspan=\"2\" >"+statc.resets+"</td>";
 response +="<td colspan=\"2\" class=\"cell\"><input type=\"button\" value=\"RESET\" onclick=\"ynDialog(\'Do you really want to reset statistics?\',\'RESET\')\" /></td></tr>";


 response +="<tr><td class=\"cell\">Boots and Resets</td>";
 response +=String() + "<td class=\"cell\" colspan=\"2\" >"+gwayConfig.boots+"</td>";
 response +="<td colspan=\"2\" class=\"cell\"><input type=\"button\" value=\"RESET\" onclick=\"ynDialog(\'Do you want to reset boots?\',\'BOOT\')\" /></td></tr>";
#endif

 response +="</table>";

 server.sendContent(response);
}






static void statisticsData()
{
 String response="";


 response +="<h2>Package Statistics</h2>";
 response +="<table class=\"config_table\">";
 response +="<tr><th class=\"thead\">Counter</th>";
#if STATISTICS == 3
 response +="<th class=\"thead\">C 0</th>";
 response +="<th class=\"thead\">C 1</th>";
 response +="<th class=\"thead\">C 2</th>";
#endif
 response +="<th class=\"thead\">Pkgs</th>";
 response +="<th class=\"thead\">Pkgs/hr</th>";
 response +="</tr>";


 response +="<tr><td class=\"cell\">Packages Downlink</td>";
#if STATISTICS == 3
  response +="<td class=\"cell\"></td>";
  response +="<td class=\"cell\"></td>";
  response +="<td class=\"cell\"></td>";
#endif
 response += "<td class=\"cell\">" + String(cp_up_pkt_fwd) + "</td>";
 response +="<td class=\"cell\"></td></tr>";

 response +="<tr><td class=\"cell\">Packages Uplink Total</td>";
#if STATISTICS == 3
  response +="<td class=\"cell\"></td>";
  response +="<td class=\"cell\"></td>";
  response +="<td class=\"cell\"></td>";
#endif
  response +="<td class=\"cell\">" + String(cp_nb_rx_rcv) + "</td>";
  response +="<td class=\"cell\">" + String((cp_nb_rx_rcv*3600)/(now() - startTime)) + "</td></tr>";

 response +="<tr><td class=\"cell\">Packages Uplink OK </td>";
#if STATISTICS == 3
  response +="<td class=\"cell\"></td>";
  response +="<td class=\"cell\"></td>";
  response +="<td class=\"cell\"></td>";
#endif
 response +="<td class=\"cell\">" + String(cp_nb_rx_ok) + "</td>";
 response +="<td class=\"cell\"></td></tr>";



#if STATISTICS == 2
 response +="<tr><td class=\"cell\">SF7 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf7;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf7/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";
 response +="<tr><td class=\"cell\">SF8 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf8;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf8/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";
 response +="<tr><td class=\"cell\">SF9 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf9;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf9/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";
 response +="<tr><td class=\"cell\">SF10 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf10;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf10/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";
 response +="<tr><td class=\"cell\">SF11 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf11;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf11/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";
 response +="<tr><td class=\"cell\">SF12 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf12;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf12/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";
#endif
#if STATISTICS == 3
 response +="<tr><td class=\"cell\">SF7 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf7_0;
  response +="<td class=\"cell\">"; response +=statc.sf7_1;
  response +="<td class=\"cell\">"; response +=statc.sf7_2;
  response +="<td class=\"cell\">"; response +=statc.sf7;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf7/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";

 response +="<tr><td class=\"cell\">SF8 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf8_0;
  response +="<td class=\"cell\">"; response +=statc.sf8_1;
  response +="<td class=\"cell\">"; response +=statc.sf8_2;
  response +="<td class=\"cell\">"; response +=statc.sf8;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf8/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";

 response +="<tr><td class=\"cell\">SF9 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf9_0;
  response +="<td class=\"cell\">"; response +=statc.sf9_1;
  response +="<td class=\"cell\">"; response +=statc.sf9_2;
  response +="<td class=\"cell\">"; response +=statc.sf9;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf9/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";

 response +="<tr><td class=\"cell\">SF10 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf10_0;
  response +="<td class=\"cell\">"; response +=statc.sf10_1;
  response +="<td class=\"cell\">"; response +=statc.sf10_2;
  response +="<td class=\"cell\">"; response +=statc.sf10;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf10/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";

 response +="<tr><td class=\"cell\">SF11 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf11_0;
  response +="<td class=\"cell\">"; response +=statc.sf11_1;
  response +="<td class=\"cell\">"; response +=statc.sf11_2;
  response +="<td class=\"cell\">"; response +=statc.sf11;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf11/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";

 response +="<tr><td class=\"cell\">SF12 rcvd</td>";
  response +="<td class=\"cell\">"; response +=statc.sf12_0;
  response +="<td class=\"cell\">"; response +=statc.sf12_1;
  response +="<td class=\"cell\">"; response +=statc.sf12_1;
  response +="<td class=\"cell\">"; response +=statc.sf12;
  response +="<td class=\"cell\">"; response += String(cp_nb_rx_rcv>0 ? 100*statc.sf12/cp_nb_rx_rcv : 0)+" %";
  response +="</td></tr>";
#endif

 response +="</table>";
 server.sendContent(response);
}
# 775 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_wwwServer.ino"
static void sensorData()
{
#if STATISTICS >= 1
 String response="";

 response += "<h2>Message History</h2>";
 response += "<table class=\"config_table\">";
 response += "<tr>";
 response += "<th class=\"thead\">Time</th>";
 response += "<th class=\"thead\">Node</th>";
#if _LOCALSERVER==1
 response += "<th class=\"thead\">Data</th>";
#endif
 response += "<th class=\"thead\" style=\"width: 20px;\">C</th>";
 response += "<th class=\"thead\">Freq</th>";
 response += "<th class=\"thead\" style=\"width: 40px;\">SF</th>";
 response += "<th class=\"thead\" style=\"width: 50px;\">pRSSI</th>";
#if RSSI==1
 if (debug > 1) {
  response += "<th class=\"thead\" style=\"width: 50px;\">RSSI</th>";
 }
#endif
 response += "</tr>";
 server.sendContent(response);

 for (int i=0; i<MAX_STAT; i++) {
  if (statr[i].sf == 0) break;

  response = "";

  response += String() + "<tr><td class=\"cell\">";
  stringTime((statr[i].tmst), response);
  response += "</td>";

  response += String() + "<td class=\"cell\">";
  if (SerialName((char *)(& (statr[i].node)), response) < 0) {
   printHEX((char *)(& (statr[i].node)),' ',response);
  }
  response += "</td>";

#if _LOCALSERVER==1
  response += String() + "<td class=\"cell\">";
  for (int j=0; j<statr[i].datal; j++) {
   if (statr[i].data[j] <0x10) response+= "0";
   response += String(statr[i].data[j],HEX) + " ";
  }
  response += "</td>";
#endif


  response += String() + "<td class=\"cell\">" + statr[i].ch + "</td>";
  response += String() + "<td class=\"cell\">" + freqs[statr[i].ch] + "</td>";
  response += String() + "<td class=\"cell\">" + statr[i].sf + "</td>";

  response += String() + "<td class=\"cell\">" + statr[i].prssi + "</td>";
#if RSSI==1
  if (debug >= 2) {
   response += String() + "<td class=\"cell\">" + statr[i].rssi + "</td>";
  }
#endif
  response += "</tr>";
  server.sendContent(response);
 }

 server.sendContent("</table>");

#endif
}
# 854 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_wwwServer.ino"
void sendWebPage(const char *cmd, const char *arg)
{
 openWebPage(); yield();

 wwwButtons();

 setVariables(cmd,arg); yield();

 statisticsData(); yield();
 sensorData(); yield();

 settingsData(); yield();
 wifiData(); yield();

 systemData(); yield();
 interruptData(); yield();


 server.sendContent(String() + "<br><br /><p style='font-size:10px'>Click <a href=\"/HELP\">here</a> to explain Help and REST options</p><br>");

 server.sendContent(String() + "</BODY></HTML>");
 server.sendContent(""); yield();

 server.client().stop();
}
# 891 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_wwwServer.ino"
void setupWWW()
{
 server.begin();




 server.on("/", []() {
  sendWebPage("","");
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });


 server.on("/HELP", []() {
  sendWebPage("HELP","");
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });


 server.on("/FORMAT", []() {
  Serial.print(F("FORMAT ..."));

  SPIFFS.format();
  initConfig(&gwayConfig);
  writeConfig( CONFIGFILE, &gwayConfig);
#if DUSB>=1
  Serial.println(F("DONE"));
#endif
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });



 server.on("/RESET", []() {
  Serial.println(F("RESET"));
  startTime= now() - 1;
  cp_nb_rx_rcv = 0;
  cp_nb_rx_ok = 0;
  cp_up_pkt_fwd = 0;
#if STATISTICS >= 1
  for (int i=0; i<MAX_STAT; i++) { statr[i].sf = 0; }
#if STATISTICS >= 2
  statc.sf7 = 0;
  statc.sf8 = 0;
  statc.sf9 = 0;
  statc.sf10= 0;
  statc.sf11= 0;
  statc.sf12= 0;

  statc.resets= 0;
  writeGwayCfg(CONFIGFILE);
#if STATISTICS >= 3
  statc.sf7_0 = 0; statc.sf7_1 = 0; statc.sf7_2 = 0;
  statc.sf8_0 = 0; statc.sf8_1 = 0; statc.sf8_2 = 0;
  statc.sf9_0 = 0; statc.sf9_1 = 0; statc.sf9_2 = 0;
  statc.sf10_0= 0; statc.sf10_1= 0; statc.sf10_2= 0;
  statc.sf11_0= 0; statc.sf11_1= 0; statc.sf11_2= 0;
  statc.sf12_0= 0; statc.sf12_1= 0; statc.sf12_2= 0;
#endif
#endif
#endif
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });


 server.on("/BOOT", []() {
  Serial.println(F("BOOT"));
#if STATISTICS >= 2
  gwayConfig.boots = 0;
  gwayConfig.wifis = 0;
  gwayConfig.views = 0;
  gwayConfig.ntpErr = 0;
  gwayConfig.ntpErrTime = 0;
  gwayConfig.ntps = 0;
#endif
  gwayConfig.reents = 0;

  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });

 server.on("/NEWSSID", []() {
  sendWebPage("NEWSSID","");
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });


 server.on("/DEBUG=-1", []() {
  debug = (debug+3)%4;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/DEBUG=1", []() {
  debug = (debug+1)%4;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });



  server.on("/PDEBUG=SCAN", []() {
  pdebug ^= P_SCAN;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/PDEBUG=CAD", []() {
  pdebug ^= P_CAD;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/PDEBUG=RX", []() {
  pdebug ^= P_RX;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/PDEBUG=TX", []() {
  pdebug ^= P_TX;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/PDEBUG=PRE", []() {
  pdebug ^= P_PRE;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/PDEBUG=MAIN", []() {
  pdebug ^= P_MAIN;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/PDEBUG=GUI", []() {
  pdebug ^= P_GUI;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/PDEBUG=RADIO", []() {
  pdebug ^= P_RADIO;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });



 server.on("/DELAY=1", []() {
  txDelay+=5000;
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/DELAY=-1", []() {
  txDelay-=5000;
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });



 server.on("/SF=1", []() {
  if (sf>=SF12) sf=SF7; else sf= (sf_t)((int)sf+1);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/SF=-1", []() {
  if (sf<=SF7) sf=SF12; else sf= (sf_t)((int)sf-1);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });


 server.on("/FREQ=1", []() {
  uint8_t nf = sizeof(freqs)/sizeof(int);
  if (ifreq==(nf-1)) ifreq=0; else ifreq++;
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/FREQ=-1", []() {
  uint8_t nf = sizeof(freqs)/sizeof(int);
  if (ifreq==0) ifreq=(nf-1); else ifreq--;
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });


 server.on("/CAD=1", []() {
  _cad=(bool)1;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/CAD=0", []() {
  _cad=(bool)0;
  writeGwayCfg(CONFIGFILE);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });


 server.on("/NODE=1", []() {
#if GATEWAYNODE==1
  gwayConfig.isNode =(bool)1;
  writeGwayCfg(CONFIGFILE);
#endif
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/NODE=0", []() {
#if GATEWAYNODE==1
  gwayConfig.isNode =(bool)0;
  writeGwayCfg(CONFIGFILE);
#endif
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });

#if GATEWAYNODE==1

 server.on("/FCNT", []() {

  frameCount=0;
  rxLoraModem();
  writeGwayCfg(CONFIGFILE);


  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
#endif

 server.on("/REFR=1", []() {
#if A_REFRESH==1
  gwayConfig.refresh =1;
  writeGwayCfg(CONFIGFILE);
#endif
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/REFR=0", []() {
#if A_REFRESH==1
  gwayConfig.refresh =0;
  writeGwayCfg(CONFIGFILE);
#endif
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });



 server.on("/HOP=1", []() {
  _hop=true;
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/HOP=0", []() {
  _hop=false;
  ifreq=0;
  freq=freqs[0];
  rxLoraModem();
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });

#if !defined ESP32_ARCH

 server.on("/SPEED=80", []() {
  system_update_cpu_freq(80);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
 server.on("/SPEED=160", []() {
  system_update_cpu_freq(160);
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
#endif

 server.on("/DOCU", []() {

  server.sendHeader("Location", String("/"), true);
  buttonDocu();
  server.send ( 302, "text/plain", "");
 });

 server.on("/LOG", []() {
  server.sendHeader("Location", String("/"), true);
#if DUSB>=1
  Serial.println(F("LOG button"));
#endif
  buttonLog();
  server.send ( 302, "text/plain", "");
 });


 server.on("/EXPERT", []() {
  server.sendHeader("Location", String("/"), true);
  gwayConfig.expert = bool(1 - (int) gwayConfig.expert) ;
  server.send ( 302, "text/plain", "");
 });



 server.on("/UPDATE=1", []() {
#if A_OTA==1
  updateOtaa();
#endif
  server.sendHeader("Location", String("/"), true);
  server.send ( 302, "text/plain", "");
 });
# 1221 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_wwwServer.ino"
 Serial.print(F("WWW Server started on port "));
 Serial.println(A_SERVERPORT);
 return;
}
# 1233 "/home/escualot60/sketchbook/ESP-1ch-Gateway-v5.0/ESP-sc-gway/_wwwServer.ino"
static void wifiData()
{
 if (gwayConfig.expert) {
 String response="";
 response +="<h2>WiFi Config</h2>";

 response +="<table class=\"config_table\">";

 response +="<tr><th class=\"thead\">Parameter</th><th class=\"thead\">Value</th></tr>";

 response +="<tr><td class=\"cell\">WiFi host</td><td class=\"cell\">";
#if ESP32_ARCH==1
 response +=WiFi.getHostname(); response+="</tr>";
#else
 response +=wifi_station_get_hostname(); response+="</tr>";
#endif

 response +="<tr><td class=\"cell\">WiFi SSID</td><td class=\"cell\">";
 response +=WiFi.SSID(); response+="</tr>";

 response +="<tr><td class=\"cell\">IP Address</td><td class=\"cell\">";
 printIP((IPAddress)WiFi.localIP(),'.',response);
 response +="</tr>";
 response +="<tr><td class=\"cell\">IP Gateway</td><td class=\"cell\">";
 printIP((IPAddress)WiFi.gatewayIP(),'.',response);
 response +="</tr>";
 response +="<tr><td class=\"cell\">NTP Server</td><td class=\"cell\">"; response+=NTP_TIMESERVER; response+="</tr>";
 response +="<tr><td class=\"cell\">LoRa Router</td><td class=\"cell\">"; response+=_TTNSERVER; response+="</tr>";
 response +="<tr><td class=\"cell\">LoRa Router IP</td><td class=\"cell\">";
 printIP((IPAddress)ttnServer,'.',response);
 response +="</tr>";
#ifdef _THINGSERVER
 response +="<tr><td class=\"cell\">LoRa Router 2</td><td class=\"cell\">"; response+=_THINGSERVER;
 response += String() + ":" + _THINGPORT + "</tr>";
 response +="<tr><td class=\"cell\">LoRa Router 2 IP</td><td class=\"cell\">";
 printIP((IPAddress)thingServer,'.',response);
 response +="</tr>";
#endif
 response +="</table>";

 server.sendContent(response);
 }
}






static void systemData()
{
 if (gwayConfig.expert) {
  String response="";
  response +="<h2>System Status</h2>";

  response +="<table class=\"config_table\">";
  response +="<tr>";
  response +="<th class=\"thead\">Parameter</th>";
  response +="<th class=\"thead\">Value</th>";
  response +="<th colspan=\"2\" class=\"thead\">Set</th>";
  response +="</tr>";

  response +="<tr><td style=\"border: 1px solid black; width:120px;\">Gateway ID</td>";
  response +="<td class=\"cell\">";
  if (MAC_array[0]< 0x10) response +='0'; response +=String(MAC_array[0],HEX);
  if (MAC_array[1]< 0x10) response +='0'; response +=String(MAC_array[1],HEX);
  if (MAC_array[2]< 0x10) response +='0'; response +=String(MAC_array[2],HEX);
  response +="FFFF";
  if (MAC_array[3]< 0x10) response +='0'; response +=String(MAC_array[3],HEX);
  if (MAC_array[4]< 0x10) response +='0'; response +=String(MAC_array[4],HEX);
  if (MAC_array[5]< 0x10) response +='0'; response +=String(MAC_array[5],HEX);
  response+="</tr>";


  response +="<tr><td class=\"cell\">Free heap</td><td class=\"cell\">"; response+=ESP.getFreeHeap(); response+="</tr>";

#if !defined ESP32_ARCH
  response +="<tr><td class=\"cell\">ESP speed</td><td class=\"cell\">"; response+=ESP.getCpuFreqMHz();
  response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"SPEED=80\"><button>80</button></a></td>";
  response +="<td style=\"border: 1px solid black; width:40px;\"><a href=\"SPEED=160\"><button>160</button></a></td>";
  response+="</tr>";
  response +="<tr><td class=\"cell\">ESP Chip ID</td><td class=\"cell\">"; response+=ESP.getChipId(); response+="</tr>";
#endif
  response +="<tr><td class=\"cell\">OLED</td><td class=\"cell\">"; response+=OLED; response+="</tr>";

#if STATISTICS>=1
  response +="<tr><td class=\"cell\">WiFi Setups</td><td class=\"cell\">"; response+=gwayConfig.wifis; response+="</tr>";
  response +="<tr><td class=\"cell\">WWW Views</td><td class=\"cell\">"; response+=gwayConfig.views; response+="</tr>";
#endif

  response +="</table>";
  server.sendContent(response);
 }
}







static void interruptData()
{
 if (gwayConfig.expert) {
  uint8_t flags = readRegister(REG_IRQ_FLAGS);
  uint8_t mask = readRegister(REG_IRQ_FLAGS_MASK);
  String response="";

  response +="<h2>System State and Interrupt</h2>";

  response +="<table class=\"config_table\">";
  response +="<tr>";
  response +="<th class=\"thead\">Parameter</th>";
  response +="<th class=\"thead\">Value</th>";
  response +="<th colspan=\"2\"  class=\"thead\">Set</th>";
  response +="</tr>";

  response +="<tr><td class=\"cell\">_state</td>";
  response +="<td class=\"cell\">";
  switch (_state) {
   case S_INIT: response +="INIT"; break;
   case S_SCAN: response +="SCAN"; break;
   case S_CAD: response +="CAD"; break;
   case S_RX: response +="RX"; break;
   case S_TX: response +="TX"; break;
   default: response +="unknown"; break;
  }
  response +="</td></tr>";

  response +="<tr><td class=\"cell\">flags (8 bits)</td>";
  response +="<td class=\"cell\">0x";
  if (flags <16) response += "0";
  response +=String(flags,HEX); response+="</td></tr>";


  response +="<tr><td class=\"cell\">mask (8 bits)</td>";
  response +="<td class=\"cell\">0x";
  if (mask <16) response += "0";
  response +=String(mask,HEX); response+="</td></tr>";

  response +="<tr><td class=\"cell\">Re-entrant cntr</td>";
  response +="<td class=\"cell\">";
  response += String() + gwayConfig.reents;
  response +="</td></tr>";

  response +="<tr><td class=\"cell\">ntp call cntr</td>";
  response +="<td class=\"cell\">";
  response += String() + gwayConfig.ntps;
  response+="</td></tr>";

  response +="<tr><td class=\"cell\">ntpErr cntr</td>";
  response +="<td class=\"cell\">";
  response += String() + gwayConfig.ntpErr;
  response +="</td>";
  response +="<td colspan=\"2\" style=\"border: 1px solid black;\">";
  stringTime(gwayConfig.ntpErrTime, response);
  response +="</td>";
  response +="</tr>";

  response +="<tr><td class=\"cell\">Time Correction (uSec)</td><td class=\"cell\">";
  response += txDelay;
  response +="</td>";
  response +="<td class=\"cell\"><a href=\"DELAY=-1\"><button>-</button></a></td>";
  response +="<td class=\"cell\"><a href=\"DELAY=1\"><button>+</button></a></td>";
  response +="</tr>";

  response +="</table>";

  server.sendContent(response);
 }
}

#endif