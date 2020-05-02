// 1-channel LoRa Gateway for ESP8266
// Copyright (c) 2016, 2017, 2018 Maarten Westenberg version for ESP8266
// Version 5.3.3
// Date: 2018-08-25
// Author: Maarten Westenberg (mw12554@hotmail.com)
//
// Based on work done by Thomas Telkamp for Raspberry PI 1-ch gateway and many others.
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the MIT License
// which accompanies this distribution, and is available at
// https://opensource.org/licenses/mit-license.php
//
// NO WARRANTY OF ANY KIND IS PROVIDED
//
// The protocols and specifications used for this 1ch gateway:
// 1. LoRA Specification version V1.0 and V1.1 for Gateway-Node communication
//
// 2. Semtech Basic communication protocol between Lora gateway and server version 3.0.0
//	https://github.com/Lora-net/packet_forwarder/blob/master/PROTOCOL.TXT
//
// Notes:
// - Once call gethostbyname() to get IP for services, after that only use IP
//	 addresses (too many gethost name makes the ESP unstable)
// - Only call yield() in main stream (not for background NTP sync).
//
// ----------------------------------------------------------------------------------------

#include "ESP-sc-gway.h" // This file contains configuration of GWay

#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#define ESP32_ARCH 1
#endif

#include <Esp.h> // ESP8266 specific IDE functions
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>
#include <string> // C++ specific string functions

#include <SPI.h>	   // For the RFM95 bus
#include <TimeLib.h>   // http://playground.arduino.cc/code/time
#include <DNSServer.h> // Local DNSserver
#include <ArduinoJson.h>
#include <FS.h> // ESP8266 Specific
#include <WiFiUdp.h>
#include <pins_arduino.h>

// Local include files
#include "_loraModem.h"
#include "loraFiles.h"
//#include "sensor.h"
//#include "oLED.h"

// Interfaz de administracion
#include "asi-src/src/asi.h"
#include "asi-src/src/PacketForwarder.h"

#include "utils.h"
#include <pb_common.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include "protobuf/gw.pb.h"

#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>

#include "lcd.h"
#include "macro_helpers.h"

// Helpers para acceder a las settings
#include "settings.h"

extern "C"
{
#include "lwip/err.h"
#include "lwip/dns.h"
}

#if WIFIMANAGER == 1
#include <WiFiManager.h> // Library for ESP WiFi config through an AP
#endif

#if (GATEWAYNODE == 1) || (_LOCALSERVER == 1)
#include "AES-128_V10.h"
#endif

// ----------- Specific ESP32 stuff --------------
#if ESP32_ARCH == 1 // IF ESP32

#include "WiFi.h"
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#if A_SERVER == 1
#include <ESP32WebServer.h> // Dedicated Webserver for ESP32
#include <Streaming.h>		// http://arduiniana.org/libraries/streaming/
#endif
#if A_OTA == 1
#include <ESP32httpUpdate.h> // Not yet available
#include <ArduinoOTA.h>
#endif //OTA

// ----------- Generic ESP8266 stuff --------------
#else

#include <ESP8266WiFi.h> // Which is specific for ESP8266
#include <ESP8266mDNS.h>
extern "C"
{
#include "user_interface.h"
#include "c_types.h"
}
#if A_SERVER == 1
#include <ESP8266WebServer.h>
#include <Streaming.h> // http://arduiniana.org/libraries/streaming/
#endif				   //A_SERVER
#if A_OTA == 1
#include <ESP8266httpUpdate.h>
#include <ArduinoOTA.h>
#endif //OTA

#endif //ESP_ARCH

// ----------- Declaration of vars --------------
uint8_t debug = 1;	   // Debug level! 0 is no msgs, 1 normal, 2 extensive
uint8_t pdebug = 0xFF; // Allow all atterns (departments)

#if GATEWAYNODE == 1
#if _GPS == 1
#include <TinyGPS++.h>
TinyGPSPlus gps;
HardwareSerial Serial1(1);
#endif
#endif

// You can switch webserver off if not necessary but probably better to leave it in.
#if A_SERVER == 1
#if ESP32_ARCH == 1
ESP32WebServer server(A_SERVERPORT);
#else
ESP8266WebServer server(A_SERVERPORT);
#endif
#endif
using namespace std;

byte currentMode = 0x81;

bool sx1272 = true; // Actually we use sx1276/RFM95

uint32_t cp_nb_rx_rcv;	 // Number of messages received by gateway
uint32_t cp_nb_rx_ok;	 // Number of messages received OK
uint32_t cp_nb_rx_bad;	 // Number of messages received bad
uint32_t cp_nb_rx_nocrc; // Number of messages without CRC
uint32_t cp_up_pkt_fwd;  // Number of packets forwarded
uint32_t cp_up_pkt_up_fail; // Number of packets received but not forwarded
uint32_t cp_dwnb; // Downlink recibidos
uint32_t cp_txnb; // Paquetes TX

uint8_t MAC_array[6];

// ----------------------------------------------------------------------------
//
// Configure these values only if necessary!
//
// ----------------------------------------------------------------------------

// Set spreading factor (SF7 - SF12)
sf_t global_sf = _SPREADING;
sf_t sfi = _SPREADING; // Initial value of SF

// Set location, description and other configuration parameters
// Defined in ESP-sc_gway.h
//
float lat = _LAT; // Configuration specific info...
float lon = _LON;
int alt = _ALT;
char platform[24] = _PLATFORM;		 // platform definition
char email[40] = _EMAIL;			 // used for contact email
char description[64] = _DESCRIPTION; // used for free form description

// define servers

IPAddress ntpServer; // IP address of NTP_TIMESERVER
IPAddress ttnServer; // IP Address of thethingsnetwork server
IPAddress thingServer;

WiFiUDP Udp;

time_t startTime = 0;	// The time in seconds since 1970 that the server started
						// be aware that UTP time has to succeed for meaningful values.
						// We use this variable since millis() is reset every 50 days...
uint32_t eventTime = 0; // Timing of _event to change value (or not).
uint32_t sendTime = 0;	// Time that the last message transmitted
uint32_t doneTime = 0;	// Time to expire when CDDONE takes too long
uint32_t statTime = 0;	// last time we sent a stat message to server
uint32_t pulltime = 0;	// last time we sent a pull_data request to server
//uint32_t lastTmst = 0;							// Last activity Timer

#if A_SERVER == 1
uint32_t wwwtime = 0;
#endif
#if NTP_INTR == 0
uint32_t ntptimer = 0;
#endif

#define TX_BUFF_SIZE 1024 // Upstream buffer to send to MQTT
#define RX_BUFF_SIZE 1024 // Downstream received from MQTT
#define STATUS_SIZE 512	  // Should(!) be enough based on the static text .. was 1024

#if GATEWAYNODE == 1
uint16_t frameCount = 0; // We write this to SPIFF file
#endif

// volatile bool inSPI This initial value of mutex is to be free,
// which means that its value is 1 (!)
//
int mutexSPI = 1;

// Clientes
WiFiClient espClient;
TinyGsm modem(Serial);
TinyGsmClient gprsClient(modem);
PubSubClient mqtt_client;

// ----------------------------------------------------------------------------
// FORWARD DECARATIONS
// These forward declarations are done since other .ino fils are linked by the
// compiler/linker AFTER the main ESP-sc-gway.ino file.
// And espcesially when calling functions with ICACHE_RAM_ATTR the complier
// does not want this.
// Solution can also be to pecify less STRICT compile options in Makefile
// ----------------------------------------------------------------------------

void ICACHE_RAM_ATTR Interrupt_0();
void ICACHE_RAM_ATTR Interrupt_1();

void mqtt_reconnect();
void mqtt_callback(char *topic, byte *payload, unsigned int length);
int sendPacket(uint8_t *buf, uint8_t length);						  // _txRx.ino
void setupWWW();													  // _wwwServer.ino
void SerialTime();													  // _utils.ino
static void printIP(IPAddress ipa, const char sep, String &response); // _wwwServer.ino

void init_oLED(); // oLED.ino
void acti_oLED();
void addr_oLED();

void setupOta(char *hostname);
void initLoraModem(); // _loraModem.ino
void cadScanner();
void rxLoraModem();								 // _loraModem.ino
void writeRegister(uint8_t addr, uint8_t value); // _loraModem.ino

void stateMachine(); // _stateMachine.ino

void SerialStat(uint8_t intr); // _utils.ino

#if MUTEX == 1
// Forward declarations
void ICACHE_FLASH_ATTR CreateMutux(int *mutex);
bool ICACHE_FLASH_ATTR GetMutex(int *mutex);
void ICACHE_FLASH_ATTR ReleaseMutex(int *mutex);
#endif

// ----------------------------------------------------------------------------
// Print leading '0' digits for hours(0) and second(0) when
// printing values less than 10
// ----------------------------------------------------------------------------
void printDigits(unsigned long digits)
{
	// utility function for digital clock display: prints leading 0
	if (digits < 10)
		dbgp(F("0"));
	dbgp(digits);
}

// ----------------------------------------------------------------------------
// Print utin8_t values in HEX with leading 0 when necessary
// ----------------------------------------------------------------------------
void printHexDigit(uint8_t digit)
{
	// utility function for printing Hex Values with leading 0
	if (digit < 0x10)
		dbgp('0');
	dbgp(digit, HEX);
}

// ----------------------------------------------------------------------------
// Print the current time
// ----------------------------------------------------------------------------
static void printTime()
{
	switch (weekday())
	{
	case 1:
		dbgp(F("Sunday"));
		break;
	case 2:
		dbgp(F("Monday"));
		break;
	case 3:
		dbgp(F("Tuesday"));
		break;
	case 4:
		dbgp(F("Wednesday"));
		break;
	case 5:
		dbgp(F("Thursday"));
		break;
	case 6:
		dbgp(F("Friday"));
		break;
	case 7:
		dbgp(F("Saturday"));
		break;
	default:
		dbgp(F("ERROR"));
		break;
	}
	dbgp(F(" "));
	printDigits(hour());
	dbgp(F(":"));
	printDigits(minute());
	dbgp(F(":"));
	printDigits(second());
	return;
}

// ----------------------------------------------------------------------------
// Convert a float to string for printing
// Parameters:
//	f is float value to convert
//	p is precision in decimal digits
//	val is character array for results
// ----------------------------------------------------------------------------
void ftoa(float f, char *val, int p)
{

	int int_part = (int)f;
	float dec_part = f - (float)int_part;

	int int_dec_part = dec_part * (10);
	sprintf(val, "%d.%d00000", int_part, int_dec_part);

	/*
	int j = 1;
	int ival, fval;
	char b[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	for (int i = 0; i < p; i++)
	{
		j = j * 10;
	}

	ival = (int)f;				  // Make integer part
	fval = (int)((f - ival) * j); // Make fraction. Has same sign as integer part
	if (fval < 0)
		fval = -fval; // So if it is negative make fraction positive again.
					  // sprintf does NOT fit in memory
	if ((f < 0) && (ival == 0))
		strcat(val, "-");
	strcat(val, itoa(ival, b, 10)); // Copy integer part first, base 10, null terminated
	strcat(val, ".");				// Copy decimal point

	itoa(fval, b, 10); // Copy fraction part base 10
	for (int i = 0; i < (p - strlen(b)); i++)
	{
		strcat(val, "0"); // first number of 0 of faction?
	}

	// Fraction can be anything from 0 to 10^p , so can have less digits
	strcat(val, b);*/
}

// ============================================================================
// NTP TIME functions

// ----------------------------------------------------------------------------
// Send the request packet to the NTP server.
//
// ----------------------------------------------------------------------------
int sendNtpRequest(IPAddress timeServerIP)
{
	const int NTP_PACKET_SIZE = 48; // Fixed size of NTP record
	byte packetBuffer[NTP_PACKET_SIZE];

	memset(packetBuffer, 0, NTP_PACKET_SIZE); // Zeroise the buffer.

	packetBuffer[0] = 0b11100011; // LI, Version, Mode
	packetBuffer[1] = 0;		  // Stratum, or type of clock
	packetBuffer[2] = 6;		  // Polling Interval
	packetBuffer[3] = 0xEC;		  // Peer Clock Precision
	// 8 bytes of zero for Root Delay & Root Dispersion
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

// ----------------------------------------------------------------------------
// Get the NTP time from one of the time servers
// Note: As this function is called from SyncINterval in the background
//	make sure we have no blocking calls in this function
// ----------------------------------------------------------------------------
time_t getNtpTime()
{
	gwayConfig.ntps++;

	if (!sendNtpRequest(ntpServer)) // Send the request for new time
	{
		if ((debug >= 0) && (pdebug & P_MAIN))
			dbgpl(F("M sendNtpRequest failed"));
		return (0);
	}

	const int NTP_PACKET_SIZE = 48; // Fixed size of NTP record
	byte packetBuffer[NTP_PACKET_SIZE];
	memset(packetBuffer, 0, NTP_PACKET_SIZE); // Set buffer cntents to zero

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
				// Extract seconds portion.
				unsigned long secs;
				secs = packetBuffer[40] << 24;
				secs |= packetBuffer[41] << 16;
				secs |= packetBuffer[42] << 8;
				secs |= packetBuffer[43];
				// UTC is 1 TimeZone correction when no daylight saving time
				return (secs - 2208988800UL + NTP_TIMEZONES * SECS_IN_HOUR);
			}
			Udp.flush();
		}
		delay(100); // Wait 100 millisecs, allow kernel to act when necessary
	}

	Udp.flush();

	// If we are here, we could not read the time from internet
	// So increase the counter
	gwayConfig.ntpErr++;
	gwayConfig.ntpErrTime = now();
#if DUSB >= 1
	if ((debug >= 0) && (pdebug & P_MAIN))
	{
		dbgpl(F("M getNtpTime:: read failed"));
	}
#endif
	return (0); // return 0 if unable to get the time
}

// ----------------------------------------------------------------------------
// Set up regular synchronization of NTP server and the local time.
// ----------------------------------------------------------------------------
#if NTP_INTR == 1
void setupTime()
{
	setSyncProvider(getNtpTime);
	setSyncInterval(_NTP_INTERVAL);
}
#endif

// ============================================================================
// UDP  FUNCTIONS

// ----------------------------------------------------------------------------
// Read DOWN a package from UDP socket, can come from any server
// Messages are received when server responds to gateway requests from LoRa nodes
// (e.g. JOIN requests etc.) or when server has downstream data.
// We respond only to the server that sent us a message!
// Note: So normally we can forget here about codes that do upstream
// Parameters:
//	Packetsize: size of the buffer to read, as read by loop() calling function
//
// Returns:
//	-1 or false if not read
//	Or number of characters read is success
//
// ----------------------------------------------------------------------------
int readUdp(int packetSize)
{
	uint8_t protocol;
	uint16_t token;
	uint8_t ident;
	uint8_t buff[32];				 // General buffer to use for UDP, set to 64
	uint8_t buff_down[RX_BUFF_SIZE]; // Buffer for downstream

	if ((WiFi.status() != WL_CONNECTED))
	//if (WlanConnect(10) < 0)
	{
#if DUSB >= 1
		dbgp(F("readdUdp: ERROR connecting to WLAN"));
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
		dbgp(F("readUDP:: ERROR package of size: "));
		dbgpl(packetSize);
#endif
		Udp.flush();
		return (-1);
	}

	// We assume here that we know the originator of the message
	// In practice however this can be any sender!
	if (Udp.read(buff_down, packetSize) < packetSize)
	{
#if DUSB >= 1
		dbgpl(F("A readUsb:: Reading less chars"));
		return (-1);
#endif
	}

	// Remote Address should be known
	IPAddress remoteIpNo = Udp.remoteIP();

	// Remote port is either of the remote TTN server or from NTP server (=123)
	unsigned int remotePortNo = Udp.remotePort();

	if (remotePortNo == 123)
	{
		// This is an NTP message arriving
#if DUSB >= 1
		if (debug >= 0)
		{
			dbgpl(F("A readUdp:: NTP msg rcvd"));
		}
#endif
		gwayConfig.ntpErr++;
		gwayConfig.ntpErrTime = now();
		return (0);
	}

	// If it is not NTP it must be a LoRa message for gateway or node
	else
	{
		uint8_t *data = (uint8_t *)((uint8_t *)buff_down + 4);
		protocol = buff_down[0];
		token = buff_down[2] * 256 + buff_down[1];
		ident = buff_down[3];

#if DUSB >= 1
		if ((debug > 1) && (pdebug & P_MAIN))
		{
			dbgp(F("M readUdp:: message waiting="));
			dbgp(ident);
			dbgpl();
		}
#endif
		// now parse the message type from the server (if any)
		switch (ident)
		{

		// This message is used by the gateway to send sensor data to the
		// server. As this function is used for downstream only, this option
		// will never be selected but is included as a reference only
		case PKT_PUSH_DATA: // 0x00 UP
#if DUSB >= 1
			if (debug >= 1)
			{
				dbgp(F("PKT_PUSH_DATA:: size "));
				dbgp(packetSize);
				dbgp(F(" From "));
				dbgp(remoteIpNo);
				dbgp(F(", port "));
				dbgp(remotePortNo);
				dbgp(F(", data: "));
				for (int i = 0; i < packetSize; i++)
				{
					dbgp(buff_down[i], HEX);
					dbgp(':');
				}
				dbgpl();
				if (debug >= 2)
					Serial.flush();
			}
#endif
			break;

		// This message is sent by the server to acknoledge receipt of a
		// (sensor) message sent with the code above.
		case PKT_PUSH_ACK: // 0x01 DOWN
#if DUSB >= 1
			if ((debug >= 2) && (pdebug & P_MAIN))
			{
				dbgp(F("M PKT_PUSH_ACK:: size "));
				dbgp(packetSize);
				dbgp(F(" From "));
				dbgp(remoteIpNo);
				dbgp(F(", port "));
				dbgp(remotePortNo);
				dbgp(F(", token: "));
				dbgpl(token, HEX);
				dbgpl();
			}
#endif
			break;

		case PKT_PULL_DATA: // 0x02 UP
#if DUSB >= 1
			dbgp(F(" Pull Data"));
			dbgpl();
#endif
			break;

		// This message type is used to confirm OTAA message to the node
		// XXX This message format may also be used for other downstream communucation
		case PKT_PULL_RESP: // 0x03 DOWN
#if DUSB >= 1
			if ((debug >= 0) && (pdebug & P_MAIN))
			{
				dbgpl(F("M readUdp:: PKT_PULL_RESP received"));
			}
#endif
			//			lastTmst = micros();					// Store the tmst this package was received

			// Send to the LoRa Node first (timing) and then do reporting to Serial
			_state = S_TX;
			sendTime = micros(); // record when we started sending the message
			// Punto de envio DOWN

			if (sendPacket(data, packetSize - 4) < 0) // Forma el paquete que es enviado en statemachine.
			{
#if DUSB >= 1
				if (debug >= 0)
				{
					dbgpl(F("A readUdp:: Error: PKT_PULL_RESP sendPacket failed"));
				}
#endif
				return (-1);
			}

			// Now respond with an PKT_TX_ACK; 0x04 UP
			buff[0] = buff_down[0];
			buff[1] = buff_down[1];
			buff[2] = buff_down[2];
			//buff[3]=PKT_PULL_ACK;					// Pull request/Change of Mogyi
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
				dbgpl(F("M readUdp:: TX buff filled"));
			}
#endif
			// Only send the PKT_PULL_ACK to the UDP socket that just sent the data!!!
			Udp.beginPacket(remoteIpNo, remotePortNo);
			if (Udp.write((unsigned char *)buff, 12) != 12)
			{
#if DUSB >= 1
				if (debug >= 0)
					dbgpl("A readUdp:: Error: PKT_PULL_ACK UDP write");
#endif
			}
			else
			{
#if DUSB >= 1
				if ((debug >= 0) && (pdebug & P_TX))
				{
					dbgp(F("M PKT_TX_ACK:: micros="));
					dbgpl(micros());
				}
#endif
			}

			if (!Udp.endPacket())
			{
#if DUSB >= 1
				if ((debug >= 0) && (pdebug & P_MAIN))
				{
					dbgpl(F("M PKT_PULL_DATALL Error Udp.endpaket"));
				}
#endif
			}

			yield();
#if DUSB >= 1
			if ((debug >= 1) && (pdebug & P_MAIN))
			{
				dbgp(F("M PKT_PULL_RESP:: size "));
				dbgp(packetSize);
				dbgp(F(" From "));
				dbgp(remoteIpNo);
				dbgp(F(", port "));
				dbgp(remotePortNo);
				dbgp(F(", data: "));
				data = buff_down + 4;
				data[packetSize] = 0;
				dbgp((char *)data);
				dbgpl(F("..."));
			}
#endif
			break;

		case PKT_PULL_ACK: // 0x04 DOWN; the server sends a PULL_ACK to confirm PULL_DATA receipt
#if DUSB >= 1
			if ((debug >= 2) && (pdebug & P_MAIN))
			{
				dbgp(F("M PKT_PULL_ACK:: size "));
				dbgp(packetSize);
				dbgp(F(" From "));
				dbgp(remoteIpNo);
				dbgp(F(", port "));
				dbgp(remotePortNo);
				dbgp(F(", data: "));
				for (int i = 0; i < packetSize; i++)
				{
					dbgp(buff_down[i], HEX);
					dbgp(':');
				}
				dbgpl();
			}
#endif
			break;

		default:
#if GATEWAYMGT == 1
			// For simplicity, we send the first 4 bytes too
			gateway_mgt(packetSize, buff_down);
#else

#endif
#if DUSB >= 1
			dbgp(F(", ERROR ident not recognized="));
			dbgpl(ident);
#endif
			break;
		}
#if DUSB >= 2
		if (debug >= 1)
		{
			dbgp(F("readUdp:: returning="));
			dbgpl(packetSize);
		}
#endif
		// For downstream messages
		return packetSize;
	}
} //readUdp

// ----------------------------------------------------------------------------
// Send UP an UDP/DGRAM message to the MQTT server
// If we send to more than one host (not sure why) then we need to set sockaddr
// before sending.
// Parameters:
//	IPAddress
//	port
//	msg *
//	length (of msg)
// return values:
//	0: Error
//	1: Success
// ----------------------------------------------------------------------------
int sendUdp(IPAddress server, int port, uint8_t *msg, int length)
{

	// Check whether we are conected to Wifi and the internet
	if (WiFi.status() != WL_CONNECTED) // (WlanConnect(3) < 0)
	{
#if DUSB >= 1
		if ((debug >= 0) && (pdebug & P_MAIN))
		{
			dbgp(F("M sendUdp: ERROR connecting to WiFi"));
			Serial.flush();
		}
#endif
		Udp.flush();
		yield();
		return (0);
	}

	yield();

	//send the update
#if DUSB >= 1
	if ((debug >= 3) && (pdebug & P_MAIN))
	{
		dbgpl(F("M WiFi connected"));
	}
#endif
	if (!Udp.beginPacket(server, (int)port))
	{
#if DUSB >= 1
		if ((debug >= 1) && (pdebug & P_MAIN))
		{
			dbgpl(F("M sendUdp:: Error Udp.beginPacket"));
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
			dbgpl(F("M sendUdp:: Error write"));
		}
#endif
		Udp.endPacket(); // Close UDP
		return (0);		 // Return error
	}

	yield();

	if (!Udp.endPacket())
	{
#if DUSB >= 1
		if (debug >= 1)
		{
			dbgpl(F("sendUdp:: Error Udp.endPacket"));
			Serial.flush();
		}
#endif
		return (0);
	}
	return (1);
} //sendUDP

// ----------------------------------------------------------------------------
// UDPconnect(): connect to UDP (which is a local thing, after all UDP
// connections do not exist.
// Parameters:
//	<None>
// Returns
//	Boollean indicating success or not
// ----------------------------------------------------------------------------
bool UDPconnect()
{

	bool ret = false;
	unsigned int localPort = _LOCUDPPORT; // To listen to return messages from WiFi
#if DUSB >= 1
	if (debug >= 1)
	{
		dbgp(F("Local UDP port="));
		dbgpl(localPort);
	}
#endif
	if (Udp.begin(localPort) == 1)
	{
#if DUSB >= 1
		if (debug >= 1)
			dbgpl(F("Connection successful"));
#endif
		ret = true;
	}
	else
	{
#if DUSB >= 1
		if (debug >= 1)
			dbgpl("Connection failed");
#endif
	}
	return (ret);
} //udpConnect

// ----------------------------------------------------------------------------
// Send UP periodic Pull_DATA message to server to keepalive the connection
// and to invite the server to send downstream messages when these are available
// *2, par. 5.2
//	- Protocol Version (1 byte)
//	- Random Token (2 bytes)
//	- PULL_DATA identifier (1 byte) = 0x02
//	- Gateway unique identifier (8 bytes) = MAC address
// ----------------------------------------------------------------------------
void pullData()
{

	uint8_t pullDataReq[12]; // status report as a JSON object
	int pullIndex = 0;
	int i;

	uint8_t token_h = (uint8_t)rand(); // random token
	uint8_t token_l = (uint8_t)rand(); // random token

	// pre-fill the data buffer with fixed fields
	pullDataReq[0] = PROTOCOL_VERSION; // 0x01
	pullDataReq[1] = token_h;
	pullDataReq[2] = token_l;
	pullDataReq[3] = PKT_PULL_DATA; // 0x02
									// READ MAC ADDRESS OF ESP8266, and return unique Gateway ID consisting of MAC address and 2bytes 0xFF
	pullDataReq[4] = MAC_array[0];
	pullDataReq[5] = MAC_array[1];
	pullDataReq[6] = MAC_array[2];
	pullDataReq[7] = 0xFF;
	pullDataReq[8] = 0xFF;
	pullDataReq[9] = MAC_array[3];
	pullDataReq[10] = MAC_array[4];
	pullDataReq[11] = MAC_array[5];
	//pullDataReq[12] = 0/00; 								// add string terminator, for safety

	pullIndex = 12; // 12-byte header

	//send the update

	uint8_t *pullPtr;
	pullPtr = pullDataReq,
#ifdef _TTNSERVER
	sendUdp(ttnServer, _TTNPORT, pullDataReq, pullIndex);
	yield();
#endif

#if DUSB >= 1
	if (pullPtr != pullDataReq)
	{
		dbgpl(F("pullPtr != pullDatReq"));
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
		dbgp(F("M PKT_PULL_DATA request, len=<"));
		dbgp(pullIndex);
		dbgp(F("> "));
		for (i = 0; i < pullIndex; i++)
		{
			dbgp(pullDataReq[i], HEX); // debug: display JSON stat
			dbgp(':');
		}
		dbgpl();
		if (debug >= 2)
			Serial.flush();
	}
#endif
	return;
} //pullData

// ----------------------------------------------------------------------------
// Send UP periodic status message to server even when we do not receive any
// data.
// Parameters:
//	- <none>
// ----------------------------------------------------------------------------
void sendstat()
{

	uint8_t status_report[STATUS_SIZE]; // status report as a JSON object
	char stat_timestamp[32];			// XXX was 24
	time_t t;
	char clat[10] = {0};
	char clon[10] = {0};

	int stat_index = 0;
	uint8_t token_h = (uint8_t)rand(); // random token
	uint8_t token_l = (uint8_t)rand(); // random token

	// pre-fill the data buffer with fixed fields
	status_report[0] = PROTOCOL_VERSION; // 0x01
	status_report[1] = token_h;
	status_report[2] = token_l;
	status_report[3] = PKT_PUSH_DATA; // 0x00

	// READ MAC ADDRESS OF ESP8266, and return unique Gateway ID consisting of MAC address and 2bytes 0xFF
	status_report[4] = MAC_array[0];
	status_report[5] = MAC_array[1];
	status_report[6] = MAC_array[2];
	status_report[7] = 0xFF;
	status_report[8] = 0xFF;
	status_report[9] = MAC_array[3];
	status_report[10] = MAC_array[4];
	status_report[11] = MAC_array[5];

	stat_index = 12; // 12-byte header

	t = now(); // get timestamp for statistics

	// XXX Using CET as the current timezone. Change to your timezone
	sprintf(stat_timestamp, "%04d-%02d-%02d %02d:%02d:%02d CET", year(), month(), day(), hour(), minute(), second());
	yield();

	ftoa(lat, clat, 5); // Convert lat to char array with 5 decimals
	ftoa(lon, clon, 5); // As IDE CANNOT prints floats

	// Build the Status message in JSON format, XXX Split this one up...
	delay(1);

	int j = snprintf((char *)(status_report + stat_index), STATUS_SIZE - stat_index,
					 "{\"stat\":{\"time\":\"%s\",\"lati\":%s,\"long\":%s,\"alti\":%i,\"rxnb\":%u,\"rxok\":%u,\"rxfw\":%u,\"ackr\":%u.0,\"dwnb\":%u,\"txnb\":%u,\"pfrm\":\"%s\",\"mail\":\"%s\",\"desc\":\"%s\"}}",
					 stat_timestamp, clat, clon, (int)alt, cp_nb_rx_rcv, cp_nb_rx_ok, cp_up_pkt_fwd, 0, 0, 0, platform, email, description);

	yield(); // Give way to the internal housekeeping of the ESP8266

	stat_index += j;
	status_report[stat_index] = 0; // add string terminator, for safety

#if DUSB >= 1
	if ((debug >= 2) && (pdebug & P_MAIN))
	{
		dbgp(F("M stat update: <"));
		dbgp(stat_index);
		dbgp(F("> "));
		dbgpl((char *)(status_report + 12)); // DEBUG: display JSON stat
	}
#endif
	if (stat_index > STATUS_SIZE)
	{
#if DUSB >= 1
		dbgpl(F("A sendstat:: ERROR buffer too big"));
#endif
		return;
	}

	//send the update
#ifdef _TTNSERVER
	sendUdp(ttnServer, _TTNPORT, status_report, stat_index);
	yield();
#endif

#ifdef _THINGSERVER
	sendUdp(thingServer, _THINGPORT, status_report, stat_index);
#endif
	return;
} //sendstat

// ============================================================================
// MAIN PROGRAM CODE (SETUP AND LOOP)

// ----------------------------------------------------------------------------
// Setup code (one time)
// _state is S_INIT
// ----------------------------------------------------------------------------
void setup()
{
	lcd_init();

	// Pins are defined and set in loraModem.h
	pinMode(pins.ss, OUTPUT);
	digitalWrite(pins.ss, HIGH);
	pinMode(pins.rst, OUTPUT);
	pinMode(pins.dio0, INPUT); // This pin is interrupt
	pinMode(pins.dio1, INPUT); // This pin is interrupt
							   //pinMode(pins.dio2, INPUT);

	// Init the SPI pins
	SPI.begin();
	delay(500);

	Serial.begin(_BAUDRATE); // As fast as possible for bus
	delay(100);

#if DUSB >= 1
	Serial.flush();

	delay(500);

	if (SPIFFS.begin())
	{
		dbgpl(F("SPIFFS init success"));
	}
	else
	{
	}
#endif

	dbgp(F("Assert="));
#if defined CFG_noassert
	dbgpl(F("No Asserts"));
#else
	dbgpl(F("Do Asserts"));
#endif

	delay(500);
	yield();
#if DUSB >= 1
	if (debug >= 1)
	{
		dbgp(F("debug="));
		dbgpl(debug);
		yield();
	}
#endif

	char MAC_char[19];
	MAC_char[18] = 0;

	WiFi.macAddress(MAC_array);

	sprintf(MAC_char, "%02x:%02x:%02x:%02x:%02x:%02x",
			MAC_array[0], MAC_array[1], MAC_array[2], MAC_array[3], MAC_array[4], MAC_array[5]);
	dbgp("MAC: ");
	dbgp(MAC_char);
	dbgp(F(", len="));
	dbgpl(strlen(MAC_char));

	asi_begin();
	asi_loop();
	asi_forceGetAllRadioSettings();
	yield();

	/* int8_t res = WiFi.waitForConnectResult();
	if (res != WL_CONNECTED)
	{
		dbgpl("No conectado");
	}
	// If we are here we are connected to WLAN
	// So now test the UDP function
	if (!UDPconnect())
	{
		dbgpl(F("Error UDPconnect"));
	}
	delay(200);
*/

	//################################################### Time ###################
	// Set the NTP Time
	// As long as the time has not been set we try to set the time.
	// #if NTP_INTR == 1
	// 	setupTime(); // Set NTP time host and interval
	// #else
	// 	// If not using the standard libraries, do a manual setting
	// 	// of the time. This meyhod works more reliable than the
	// 	// interrupt driven method.

	// 	//setTime((time_t)getNtpTime());
	// 	while (timeStatus() == timeNotSet)
	// 	{
	// #if DUSB >= 1
	// 		if ((debug >= 0) && (pdebug & P_MAIN))
	// 			dbgpl(F("M setupTime:: Time not set (yet)"));
	// #endif
	// 		delay(500);
	// 		time_t newTime;
	// 		newTime = (time_t)getNtpTime();
	// 		if (newTime != 0)
	// 			setTime(newTime);
	// 	}
	// 	// When we are here we succeeded in getting the time
	// 	startTime = now(); // Time in seconds
	// #if DUSB >= 1
	// 	dbgp("Time: ");
	// 	printTime();
	// 	dbgpl();
	// #endif
	// 	//writeGwayCfg(CONFIGFILE);
	// #if DUSB >= 1
	// 	//dbgpl(F("Gateway configuration saved"));
	// #endif
	// #endif //NTP_INTR

	delay(100); // Wait after setup

	// #################################### LORA ########################################

	// We choose the Gateway ID to be the Ethernet Address of our Gateway card
	// display results of getting hardware address
	//
	dbgp("Gateway ID: ");
	printHexDigit(MAC_array[0]);
	printHexDigit(MAC_array[1]);
	printHexDigit(MAC_array[2]);
	printHexDigit(0xFF);
	printHexDigit(0xFF);
	printHexDigit(MAC_array[3]);
	printHexDigit(MAC_array[4]);
	printHexDigit(MAC_array[5]);

	dbgp(", Listening at SF");
	dbgp(global_sf);
	dbgp(" on ");
	dbgp((double)freq / 1000000);
	dbgpl(" Mhz.");
	// Setup ad initialise LoRa state machine of _loramModem.ino
	_state = S_INIT;
	initLoraModem();

	if (cadGet())
	{
		_state = S_SCAN;
		global_sf = SF7;
		cadScanner(); // Always start at SF7
	}
	else
	{
		_state = S_RX;
		rxLoraModem();
	}
	LoraUp.payLoad[0] = 0;
	LoraUp.payLength = 0; // Init the length to 0

	// init interrupt handlers, which are shared for GPIO15 / D8,
	// we switch on HIGH interrupts
	if (pins.dio0 == pins.dio1)
	{
		//SPI.usingInterrupt(digitalPinToInterrupt(pins.dio0));
		attachInterrupt(pins.dio0, Interrupt_0, RISING); // Share interrupts
	}
	// Or in the traditional Comresult case
	else
	{
		//SPI.usingInterrupt(digitalPinToInterrupt(pins.dio0));
		//SPI.usingInterrupt(digitalPinToInterrupt(pins.dio1));
		attachInterrupt(pins.dio0, Interrupt_0, RISING); // Separate interrupts
		attachInterrupt(pins.dio1, Interrupt_1, RISING); // Separate interrupts
	}

	// ###################### MQTT Init #################################
	if (settings_protocol() == MODO_AGROTOOLS)
	{
		mqtt_client.setServer(settings_tb_mqtt_server(), settings_tb_mqtt_port());
		mqtt_client.setCallback(mqtt_callback);
	}
	else
	{
		mqtt_client.setServer(settings_mqtt_server(), settings_mqtt_port());
		mqtt_client.setCallback(mqtt_callback);
	}

	//lcd_line3(settings_mqtt_server());

	dbgpl(F("--------------------------------------"));
} //setup

// ----------------------------------------------------------------------------
// LOOP
// This is the main program that is executed time and time again.
// We need to give way to the backend WiFi processing that
// takes place somewhere in the ESP8266 firmware and therefore
// we include yield() statements at important points.
//
// Note: If we spend too much time in user processing functions
//	and the backend system cannot do its housekeeping, the watchdog
// function will be executed which means effectively that the
// program crashes.
// We use yield() a lot to avoid ANY watch dog activity of the program.
//
// NOTE2: For ESP make sure not to do large array declarations in loop();
// ----------------------------------------------------------------------------

volatile uint8_t mqtt_down_flag = 0;

#define TINY_GSM_MODEM_SIM800

void loop()
{
	uint32_t uSeconds; // micro seconds
	int packetSize;
	uint32_t nowSeconds = now();
	static bool udpInit = false; // Flags para inicializar UDP y Time.
	static bool timeInit = false;

	S_PROTOCOL protocolo = settings_protocol();
	S_BACKBONE backbone = asi_loraSettings()._backbone == 2 ? BACKBONE_GPRS : BACKBONE_WIFI; //settings_backbone();

	// ######################## UDP Init ####################################################
	if (!udpInit && (protocolo == SEMTECH_PF_UDP) && (backbone == BACKBONE_WIFI))
	{
		if (WiFi.status() == WL_CONNECTED)
		{
			if (!WiFi.hostByName(NTP_TIMESERVER, ntpServer)) // Get IP address of Timeserver
			{
				die("Setup:: ERROR hostByName NTP");
			};
			delay(100);
#ifdef _TTNSERVER
			if (!WiFi.hostByName(_TTNSERVER, ttnServer)) // Use DNS to get server IP once
			{
				die("Setup:: ERROR hostByName TTN");
			};
			delay(100);
#endif
			udpInit = true;
		}
	}

	// Cambiar el cliente para mqtt
	/* Todo la configuracion gprs pasa por aca */
	static uint32_t gprs_cwt = 0; // GPRS Watchdog Timer

	if (backbone == BACKBONE_GPRS)
	{
		mqtt_client.setClient(gprsClient);
		
		if (((millis()- gprs_cwt) > 20000)) // Chequear cada 20 segs
		{
			gprs_cwt = millis();
			if(!gprs_connected()){
				gprs_init();
			}
		}
		else
		{
			// Conectado -> Obtener el tiempo
			/*if (!timeInit)
			{
				uint8_t res = modem.NTPServerSync();
				if (res == 1)
				{
					String tiempo = modem.getGSMDateTime(DATE_FULL);
					yield();
					// lcd_line3(tiempo.c_str());
					timeInit = true;
					// 13/09/11,20:23:25+32
					uint8_t ano, mes, dia, hora, minu, segu;
					//int res = sscanf(ti empo.c_str(),"%d/%d/%d,%d:%d:%d",&ano,&mes,&dia,&hora,&minu,&segu);
					//if(res == 6){
					//	setTime(hora,minu,segu,dia,mes,ano);

					//}
				}
			}*/
		}
	}
	else if (backbone == BACKBONE_WIFI)
	{
		mqtt_client.setClient(espClient);
		if ((!timeInit) && (WiFi.status() == WL_CONNECTED))
		{
			// Init time
			time_t newTime;
			newTime = (time_t)getNtpTime();
			if (newTime != 0)
			{
				setTime(newTime);
				timeInit = true;
			}
		}
	}

	asi_loop(); //Interface Loop

	// check for event value, which means that an interrupt has arrived.
	// In this case we handle the interrupt ( e.g. message received)
	// in userspace in loop().

	if (protocolo == MQTTBRIDGE_TCP)
	{
		//mqtt_client.loop();
		if (mqtt_down_flag == 1) // Hay un Downlink packet
		{
			dbgp("mqtt_down ");
			dbgpl(_state);
			mqtt_down_flag = 0;
			_state = S_TX;
		}
		//yield();
	}

	stateMachine(); // do the state machine

	// After a quiet period, make sure we reinit the modem and state machine.
	// The interval is in seconds (about 15 seconds) as this re-init
	// is a heavy operation.
	// SO it will kick in if there are not many messages for the gateway.
	// Note: Be careful that it does not happen too often in normal operation.
	//
	if (((nowSeconds - statr[0].tmst) > _MSG_INTERVAL) &&
		(msgTime <= statr[0].tmst))
	{
#if DUSB >= 1
		if ((debug >= 1) && (pdebug & P_MAIN))
		{
			dbgp("M REINIT:: ");
			dbgp(_MSG_INTERVAL);
			dbgp(F(" "));
			SerialStat(0);
		}
#endif

		// Radio HEALTH
		

		// startReceiver() ??
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
		writeRegister(REG_IRQ_FLAGS, (uint8_t)0xFF); // Reset all interrupt flags
		msgTime = nowSeconds;
	}

	// If event is set, we know that we have a (soft) interrupt.
	// After all necessary web/OTA services are scanned, we will
	// reloop here for timing purposes.
	// Do as less yield() as possible.
	// XXX 180326
	if (_event == 1)
	{
		return;
	}
	else
		yield();

	//################################################ DOWNSTREAM UDP ########################################
	if ((protocolo == SEMTECH_PF_UDP) && (WiFi.status() == WL_CONNECTED))
	{
		while ((packetSize = Udp.parsePacket()) > 0)
		{
#if DUSB >= 2
			dbgpl(F("loop:: readUdp calling"));
#endif
			// DOWNSTREAM
			// Packet may be PKT_PUSH_ACK (0x01), PKT_PULL_ACK (0x03) or PKT_PULL_RESP (0x04)
			// This command is found in byte 4 (buffer[3])
			if (readUdp(packetSize) <= 0)
			{
#if DUSB >= 1
				if ((debug > 0) && (pdebug & P_MAIN))
					dbgpl(F("M readUDP error"));
#endif
				break;
			}
			// Now we know we succesfully received message from host
			else
			{
				//_event=1;								// Could be done double if more messages received
			}
		}
	}

	// ########################## LAN Reconect ##################################################
	// If we are not connected, try to connect.
	// We will not read Udp in this loop cycle then

	/*if ((WlanConnect(1) < 0) || (backbone == BACKBONE_GPRS))
	{
#if DUSB >= 1
		if ((debug >= 0) && (pdebug & P_MAIN))
			dbgpl(F("M ERROR reconnect WLAN"));
#endif
		yield();
		//return; // Exit loop if no WLAN connected
	}
	else
	{

		// So if we are connected
		// Receive UDP PUSH_ACK messages from server. (*2, par. 3.3)
		// This is important since the TTN broker will return confirmation
		// messages on UDP for every message sent by the gateway. So we have to consume them.
		// As we do not know when the server will respond, we test in every loop.
		//
		if (protocolo == SEMTECH_PF_UDP)
		{
			while ((packetSize = Udp.parsePacket()) > 0)
			{
#if DUSB >= 2
				dbgpl(F("loop:: readUdp calling"));
#endif
				// DOWNSTREAM
				// Packet may be PKT_PUSH_ACK (0x01), PKT_PULL_ACK (0x03) or PKT_PULL_RESP (0x04)
				// This command is found in byte 4 (buffer[3])
				if (readUdp(packetSize) <= 0)
				{
#if DUSB >= 1
					if ((debug > 0) && (pdebug & P_MAIN))
						dbgpl(F("M readUDP error"));
#endif
					break;
				}
				// Now we know we succesfully received message from host
				else
				{
					//_event=1;								// Could be done double if more messages received
				}
			}
		}
	}*/

	// ########################## END LAN Reconect ##################################################

	yield(); // XXX 26/12/2017

	// ##################################################### Stat ############################################

	// stat PUSH_DATA message (*2, par. 4)
	//

	if ((nowSeconds - statTime) >= settings_stats_interval())
	{ // Wake up every xx seconds

		if (protocolo == MODO_AGROTOOLS)
		{
			mqtt_sendStat();
			/* ¿Algo? */
		}
#if DUSB >= 1
		if ((debug >= 1) && (pdebug & P_MAIN))
		{
			dbgp(F("M STAT:: ..."));
			Serial.flush();
		}
#endif
		if (protocolo == SEMTECH_PF_UDP)
		{
			sendstat(); // Show the status message and send to server
		}
		else if (protocolo == MQTTBRIDGE_TCP)
		{
			mqtt_sendStat();
		}

#if DUSB >= 1
		if ((debug >= 1) && (pdebug & P_MAIN))
		{
			dbgpl(F(" done"));
			if (debug >= 2)
				Serial.flush();
		}
#endif

		statTime = nowSeconds;
	}

	yield();

	if (protocolo == SEMTECH_PF_UDP)
	{
		// PULL Data #################################################
		// send PULL_DATA message (*2, par. 4)
		//
		nowSeconds = now();
		if ((nowSeconds - pulltime) >= _PULL_INTERVAL)
		{ // Wake up every xx seconds
#if DUSB >= 1
			if ((debug >= 2) && (pdebug & P_MAIN))
			{
				dbgpl(F("M PULL"));
				if (debug >= 1)
					Serial.flush();
			}
#endif
			pullData(); // Send PULL_DATA message to server
			startReceiver();
			pulltime = nowSeconds;
		}
	}
	else
	{
		// MQTT tiene ping propio
	}

	// If we do our own NTP handling (advisable)
	// We do not use the timer interrupt but use the timing
	// of the loop() itself which is better for SPI
#if NTP_INTR == 0
	// Set the time in a manual way. Do not use setSyncProvider
	// as this function may collide with SPI and other interrupts
	yield(); // 26/12/2017
	nowSeconds = now();
	if (nowSeconds - ntptimer >= _NTP_INTERVAL)
	{
		timeInit = false; // Setear el flag para forzar resync en el siguiente loop
		/*yield();
		time_t newTime;
		newTime = (time_t)getNtpTime();
		if (newTime != 0)
			setTime(newTime);*/
		ntptimer = nowSeconds;
	}
#endif

	if (protocolo == MQTTBRIDGE_TCP || protocolo == MODO_AGROTOOLS)
	{
		//######################### MQTT ############################
		// Reconectar si estoy fuera.
		static uint32_t mqtt_loopwt = 0; // MQTT Loop Watchdog Timer
		
		if(((millis() - mqtt_loopwt) > 50)){
			mqtt_loopwt = millis();
			if(!(mqtt_client.loop())){
				// No hay conexion
				mqtt_reconnect();
			}
		}
		
	}

	// lcd_update(cp_nb_rx_rcv, cp_nb_rx_ok, freq, global_sf);
} //loop

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

	dbgpl("Reconfig LORA");
}

bool decode_string(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
	uint8_t buffer[1024] = {0};
	char *destino = (char *)*arg;

	/* We could read block-by-block to avoid the large buffer... */
	if (stream->bytes_left > sizeof(buffer) - 1)
		return false;

	if (!pb_read(stream, (pb_byte_t *)destino, stream->bytes_left))
		return false;

	return true;
}

bool pb_phy_payload_decode(pb_istream_t *stream, const pb_field_t *field, void **arg)
{

	/* We could read block-by-block to avoid the large buffer... */
	/* El payLoad global */
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

	/* We could read block-by-block to avoid the large buffer... */
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

	/* We could read block-by-block to avoid the large buffer... */
	if (stream->bytes_left > 50 - 1)
		return false;

	downlink_size = stream->bytes_left;

	if (!pb_read(stream, (pb_byte_t *)destino, stream->bytes_left))
		return false;

	return true;
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{

	if(settings_protocol() == MODO_AGROTOOLS){
		return;
	}

	dbgp("Message arrived [");
	dbgp(topic);
	dbgp("] Length ");
	dbgp(length);
	dbgpl();

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
		dbgp("Decoding failed: ");
		dbgpl(PB_GET_ERROR(&stream));
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

	dbgp("bw ");
	dbgpl(LoraDown.bw);
	dbgp("sfTx ");
	dbgpl(LoraDown.sfTx);
	dbgp("tmst ");
	dbgpl(LoraDown.tmst);
	dbgp("DelayS ");
	dbgpl((long)dm.tx_info.timing_info.delay_timing_info.delay.seconds);
	dbgp("DelayN ");
	dbgpl((long)dm.tx_info.timing_info.delay_timing_info.delay.nanos);

	dbgp("UplinkTimingContext ");
	dbgpl((unsigned long)uplink_timing_context);

	dbgp("fff ");
	dbgpl(LoraDown.fff);
	dbgp("length ");
	dbgpl(LoraDown.payLength);

	mqtt_send_ack(downlink_id);
	cp_dwnb++;

	/*gw_GatewayStats statsMsg = gw_GatewayStats_init_zero;
	uint8_t pbbuffer[128];
	char realIp[20];
	statsMsg.ip.funcs.decode = decode_string;
	statsMsg.ip.arg = realIp;
	
	pb_istream_t stream = pb_istream_from_buffer(payload, length);
	bool status = pb_decode(&stream, gw_GatewayStats_fields, &statsMsg);
	if (!status)
        {
            dbgp("Decoding failed: "); dbgpl(PB_GET_ERROR(&stream));
            return;
        }
	dbgpl(realIp);
	dbgpl("rx_pack");
	dbgpl((statsMsg.rx_packets_received));*/
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
	char *str = "192.168.88.4"; // TODO

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
	//struct LoraUp * str = (struct LoraUp *) *arg;

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

	dbgpl("ID enc");
	dbgpl((uint32_t)*arg);
	for (uint8_t i = 0; i < downlink_size; i++)
	{
		if (str[i] <= 0xF)
			dbgp('0');
		dbgp(str[i], HEX);
		dbgp(' ');
	}

	if (!pb_encode_tag_for_field(stream, field))
		return false;

	return pb_encode_string(stream, (uint8_t *)str, downlink_size);
}

void mqtt_sendStat()
{
	if(settings_protocol() == MODO_AGROTOOLS){

		
		return;
	}

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
	statsMsg.rx_packets_received = cp_nb_rx_rcv;
	statsMsg.rx_packets_received_ok = cp_nb_rx_ok;
	statsMsg.tx_packets_received = cp_dwnb;
	statsMsg.tx_packets_emitted = cp_txnb;

	uint8_t buffer[120];
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	if (!pb_encode(&stream, gw_GatewayStats_fields, &statsMsg))
	{
		dbgp("Encoding failed: ");
		dbgpl(PB_GET_ERROR(&stream));
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
	//upMsg.rx_info.rssi = up_packet.prssi;
	//upMsg.rx_info.lora_snr = up_packet.snr;
	upMsg.rx_info.channel = 1;

	upMsg.rx_info.location.source = common_LocationSource_CONFIG;
	upMsg.rx_info.location.latitude = -35.1684698;
	upMsg.rx_info.location.longitude = -59.0927072;
	upMsg.rx_info.context.funcs.encode = pb_set_timing_context;
	upMsg.rx_info.fine_timestamp_type = gw_FineTimestampType_NONE;
	upMsg.rx_info.which_fine_timestamp = gw_UplinkRXInfo_plain_fine_timestamp_tag;

	uint8_t buffer[200];

	dbgpl("Encoding");
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	if (!pb_encode(&stream, gw_UplinkFrame_fields, &upMsg))
	{
		dbgp("Encoding failed: ");
		dbgpl(PB_GET_ERROR(&stream));
		return;
	}
	dbgpl("Publishings");
	mqtt_client.publish("gateway/4c11aeffff045b23/event/up", buffer, stream.bytes_written);
	cp_up_pkt_fwd++;
}

void mqtt_send_ack(uint8_t *downId)
{
	gw_DownlinkTXAck dwack = gw_DownlinkTXAck_init_zero;

	dwack.gateway_id.funcs.encode = pb_set_gateway_id;

	dwack.downlink_id.arg = downId;
	dwack.downlink_id.funcs.encode = pb_set_downlink_id;

	uint8_t buffer[100];

	/*dbgp("ACK Down ID ");
	for(uint8_t i=0; i<downlink_size; i++){
		if (downId[i]<=0xF) dbgp('0');
		dbgp(downId[i], HEX);
		dbgp(' ');
	}*/

	dbgpl("Encoding");
	dbgpl((uint32_t)*downId);
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	if (!pb_encode(&stream, gw_DownlinkTXAck_fields, &dwack))
	{
		dbgp("Encoding failed: ");
		dbgpl(PB_GET_ERROR(&stream));
		return;
	}
	dbgpl("PublishingsACK");
	mqtt_client.publish("gateway/4c11aeffff045b23/event/ack", buffer, stream.bytes_written);
}

void mqtt_reconnect()
{
	// Loop until we're reconnected
	if (!mqtt_client.connected())
	{
		dbgp("Attempting MQTT connection..");
		// Create a random client ID
		String clientId = "ESP8266Client-";
		clientId += String(random(0xffff), HEX);
		// Attempt to connect
		if (settings_protocol() == MODO_AGROTOOLS)
		{
			if (mqtt_client.connect(clientId.c_str(), settings_tb_mqtt_user(), ""))
			{
				mqtt_client.subscribe("v1/devices/me/rpc/request/+");
			}
			else
			{
				dbgp("failed, rc=");
				dbgp(mqtt_client.state());
				dbgpl(" try again in 5 seconds");
			}
		}
		else
		{
			// Settings Chirpstack
			if (mqtt_client.connect(clientId.c_str(), settings_mqtt_user(), settings_mqtt_pass()))
			{
				dbgpl("connected ");
				// Once connected, publish an announcement...
				// mqtt_client.publish("outTopic", "hello world");
				// ... and resubscribe
				dbgp(mqtt_client.subscribe("gateway/4c11aeffff045b23/command/down"));
				//mqtt_client.subscribe("gateway/4c11aeffff045b23/command/+");
			}
			else
			{
				dbgp("failed, rc=");
				dbgp(mqtt_client.state());
				dbgpl(" try again in 5 seconds");
				// Wait 5 seconds before retrying
				// delay(5000);
			}
		}
	}
}

void gprs_init()
{

	// !!!!!!!!!!!
	// Set your reset, enable, power pins here
	// !!!!!!!!!!!

	// Restart takes quite some time
	// To skip it, call init() instead of restart()
	// SerialMon.println("Initializing modem...");
	//lcd_line3("Restarting GPRS...");
	modem.restart();
	yield();
	// modem.init();

	String modemInfo = modem.getModemInfo();
	//lcd_line3(modemInfo.c_str());

	//SerialMon.print("Modem Info: ");
	//SerialMon.println(modemInfo);

	// Unlock your SIM card with a PIN if needed
	/* if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }*/

	modem.gprsConnect(settings_apn(), settings_gprs_user(), settings_gprs_pass());

	// SerialMon.print("Waiting for network...");
	if (!modem.waitForNetwork())
	{
		// SerialMon.println(" fail");
		// delay(10000);
		return;
	}

	// SerialMon.println(" success");

	if (modem.isNetworkConnected())
	{
		// SerialMon.println("Network connected");
		// lcd_line3("GPRS Nwrk CONN");
	}

	// GPRS connection parameters are usually set after network registration
	//SerialMon.print(F("Connecting to "));
	//SerialMon.print(apn);
	if (!modem.gprsConnect(settings_apn(), settings_gprs_user(), settings_gprs_pass()))
	{
		//SerialMon.println(" fail");
		// delay(10000);
		// lcd_line3("GPRS CONN Fail");
		return;
	}
	//SerialMon.println(" success");

	if (modem.isGprsConnected())
	{
		// SerialMon.println("GPRS connected");
		// lcd_line3("GPRS CONN Ok");
	}
}

bool gprs_connected()
{
	return modem.isGprsConnected();
}

void pf_settings_callback(PFListaSettings settings)
{
	/* Callback para cambios en settings */
	if (settings._protocolo == 2)
	{
		mqtt_client.disconnect(); // Desconectar para reconectar
		mqtt_client.setServer(settings._mqttHost.c_str(), settings._mqttPort);
		mqtt_client.setCallback(mqtt_callback);
	}
	else if (settings._protocolo == 3)
	{
		mqtt_client.disconnect(); // Desconectar para reconectar
		mqtt_client.setServer(settings._mqttHost.c_str(), settings._mqttPort);
		mqtt_client.setCallback(mqtt_callback);
	}
}

void mqtt_sendTBPacket(struct LoraUp  up_packet)
{
	char mqtt_msg[200];
	strncpy(mqtt_msg, "{\"rxpl\":\"", 150);
	memcpy(mqtt_msg + 9, LoraUp.payLoad, LoraUp.payLength);
	strncpy(mqtt_msg + 9 + LoraUp.payLength, "\"}",141);

	if(mqtt_client.publish("v1/devices/me/telemetry", mqtt_msg, strlen(mqtt_msg))){
		cp_up_pkt_fwd++;
		
	}else{
		// retry 1
		Serial.println("FALLO PUBLISH!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		Serial.println(LoraUp.payLength);
		Serial.write(LoraUp.payLoad,128);
		Serial.println();
		if(mqtt_client.publish("v1/devices/me/telemetry", mqtt_msg, strlen(mqtt_msg))){
			cp_up_pkt_fwd++;
		}else{
			cp_up_pkt_up_fail++;
		}

	}
}