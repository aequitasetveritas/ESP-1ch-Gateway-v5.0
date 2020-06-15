// 1-channel LoRa Gateway for ESP8266
// Copyright (c) 2016, 2017, 2018 Maarten Westenberg version for ESP8266
// Version 5.3.3
// Date: 2018-08-25
//
// 	based on work done by Thomas Telkamp for Raspberry PI 1ch gateway
//	and many others.
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the MIT License
// which accompanies this distribution, and is available at
// https://opensource.org/licenses/mit-license.php
//
// NO WARRANTY OF ANY KIND IS PROVIDED
//
// Author: Maarten Westenberg (mw12554@hotmail.com)
//
// This file contains the utilities for time and other functions
// ========================================================================================

// ----------------------------------------------------------------------------
// Fill a HEXadecimal String  from a 4-byte char array
//
// ----------------------------------------------------------------------------
#include "utils.h"
#include <Time.h>
#include "macro_helpers.h"
extern uint8_t debug;	 // Debug level! 0 is no msgs, 1 normal, 2 extensive
extern uint8_t pdebug;

static void printHEX(char * hexa, const char sep, String& response) 
{
	char m;
	m = hexa[0]; if (m<016) response+='0'; response += String(m, HEX);  response+=sep;
	m = hexa[1]; if (m<016) response+='0'; response += String(m, HEX);  response+=sep;
	m = hexa[2]; if (m<016) response+='0'; response += String(m, HEX);  response+=sep;
	m = hexa[3]; if (m<016) response+='0'; response += String(m, HEX);  response+=sep;
}

// ----------------------------------------------------------------------------
// stringTime
// Print the time t into the String reponse. t is of type time_t in seconds.
// Only when RTC is present we print real time values
// t contains number of seconds since system started that the event happened.
// So a value of 100 would mean that the event took place 1 minute and 40 seconds ago
// ----------------------------------------------------------------------------
static void stringTime(time_t t, String& response) {

	if (t==0) { response += "--"; return; }
	
	// now() gives seconds since 1970
	// as millis() does rotate every 50 days
	// So we need another timing parameter
	time_t eTime = t;
	
	// Rest is standard
	byte _hour   = hour(eTime);
	byte _minute = minute(eTime);
	byte _second = second(eTime);
	
	switch(weekday(eTime)) {
		case 1: response += "Sunday "; break;
		case 2: response += "Monday "; break;
		case 3: response += "Tuesday "; break;
		case 4: response += "Wednesday "; break;
		case 5: response += "Thursday "; break;
		case 6: response += "Friday "; break;
		case 7: response += "Saturday "; break;
	}
	response += String() + day(eTime) + "-";
	response += String() + month(eTime) + "-";
	response += String() + year(eTime) + " ";
	
	if (_hour < 10) response += "0";
	response += String() + _hour + ":";
	if (_minute < 10) response += "0";
	response += String() + _minute + ":";
	if (_second < 10) response += "0";
	response += String() + _second;
}


// ----------------------------------------------------------------------------
// SerialTime
// Print the current time on the Serial (USB), with leading 0.
// ----------------------------------------------------------------------------
void SerialTime() 
{

	uint32_t thrs = hour();
	uint32_t tmin = minute();
	uint32_t tsec = second();
			
	if (thrs<10) {dbgp('0');} dbgp(thrs);
	dbgp(':');
	if (tmin<10) {dbgp('0');} dbgp(tmin);
	dbgp(':');
	if (tsec<10) {dbgp('0');} dbgp(tsec);
			
	if (debug>=2) {Serial.flush();}
		
	return;
}

// ----------------------------------------------------------------------------
// SerialStat
// Print the statistics on Serial (USB) port
// ----------------------------------------------------------------------------

void SerialStat(uint8_t intr) 
{
#if DUSB>=1
	if (debug>=0) {
		dbgp(F("I="));

		if (intr & IRQ_LORA_RXTOUT_MASK) dbgp(F("RXTOUT "));		// 0x80
		if (intr & IRQ_LORA_RXDONE_MASK) dbgp(F("RXDONE "));		// 0x40
		if (intr & IRQ_LORA_CRCERR_MASK) dbgp(F("CRCERR "));		// 0x20
		if (intr & IRQ_LORA_HEADER_MASK) dbgp(F("HEADER "));		// 0x10
		if (intr & IRQ_LORA_TXDONE_MASK) dbgp(F("TXDONE "));		// 0x08
		if (intr & IRQ_LORA_CDDONE_MASK) dbgp(F("CDDONE "));		// 0x04
		if (intr & IRQ_LORA_FHSSCH_MASK) dbgp(F("FHSSCH "));		// 0x02
		if (intr & IRQ_LORA_CDDETD_MASK) dbgp(F("CDDETD "));		// 0x01

		if (intr == 0x00) dbgp(F("  --  "));
			
		dbgp(F(", F="));
		dbgp(ifreq);
		dbgp(F(", SF="));
		dbgp(sf);
		dbgp(F(", E="));
		dbgp(_event);
			
		dbgp(F(", S="));
		//dbgp(_state);
		switch (_state) {
			case S_INIT:
				dbgp(F("INIT "));
			break;
			case S_SCAN:
				dbgp(F("SCAN "));
			break;
			case S_CAD:
				dbgp(F("CAD  "));
			break;
			case S_RX:
				dbgp(F("RX   "));
			break;
			case S_TX:
				dbgp(F("TX   "));
			break;
			case S_TXDONE:
				dbgp(F("TXDONE"));
			break;
			default:
				dbgp(F(" -- "));
		}
		dbgp(F(", eT="));
		dbgp( micros() - eventTime );
		dbgp(F(", dT="));
		dbgp( micros() - doneTime );
		dbgpl();
	}
#endif
}


	
// ----------------------------------------------------------------------------
// SerialName(id, response)
// Check whether for address a (4 bytes in Unsigned Long) there is a name
// This function only works if _TRUSTED_NODES is set
// ----------------------------------------------------------------------------

int SerialName(char * a, String& response)
{
#if _TRUSTED_NODES>=1
	uint32_t id = ((a[0]<<24) | (a[1]<<16) | (a[2]<<8) | a[3]);

	int i;
	for ( i=0; i< (sizeof(nodes)/sizeof(nodex)); i++) {

		if (id == nodes[i].id) {
#if DUSB >=1
			if (( debug>=3 ) && ( pdebug & P_GUI )) {
				dbgp(F("G Name="));
				dbgp(nodes[i].nm);
				dbgp(F(" for node=0x"));
				dbgp(nodes[i].id,HEX);
				dbgpl();
			}
#endif
			response += nodes[i].nm;
			return(i);
		}
	}

#endif
	return(-1);									// If no success OR is TRUSTED NODES not defined
}

#if _LOCALSERVER==1
// ----------------------------------------------------------------------------
// inDecodes(id)
// Find the id in Decodes array, and return the index of the item
// Parameters:
//		id: The first field in the array (normally DevAddr id). Must be char[4]
// Returns:
//		The index of the ID in the Array. Returns -1 if not found
// ----------------------------------------------------------------------------
int inDecodes(char * id) {

	uint32_t ident = ((id[3]<<24) | (id[2]<<16) | (id[1]<<8) | id[0]);

	int i;
	for ( i=0; i< (sizeof(decodes)/sizeof(codex)); i++) {
		if (ident == decodes[i].id) {
			return(i);
		}
	}
	return(-1);
}
#endif

// ----------------------------------------------------------------------------
// DIE is not use actively in the source code anymore.
// It is replaced by a dbgp command so we know that we have a problem
// somewhere.
// There are at least 3 other ways to restart the ESP. Pick one if you want.
// ----------------------------------------------------------------------------
void die(const char *s)
{
	dbgpl(s);
	if (debug >= 2)
		Serial.flush();

	delay(50);
	// system_restart();						// SDK function
	// ESP.reset();
	abort(); // Within a second
}

// ----------------------------------------------------------------------------
// gway_failed is a function called by ASSERT in ESP-sc-gway.h
//
// ----------------------------------------------------------------------------
void gway_failed(const char *file, uint16_t line)
{
	dbgp(F("Program failed in file: "));
	dbgp(file);
	dbgp(F(", line: "));
	dbgpl(line);
	if (debug >= 2)
		Serial.flush();
}