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
// This file contains the LoRa modem specific code enabling to receive
// and transmit packages/messages.
// ========================================================================================

#include <Base64.h>
#include <AES-128_V10.h>
#include "_loraModem.h"
#include "settings.h"

#include "macro_helpers.h"

bool gbl_fakeLoraWanFlag = false;

void fakeLoraWanWrap(uint8_t *message, uint8_t *messageLen)
{
	uint8_t mhdr = 0x40; // Propietary Unconfirmed data up
	uint8_t fhdr[7];
	// Address
	fhdr[0] = message[13];
	fhdr[1] = message[12]; // Los tres ultimos bytes de la direccion
	fhdr[2] = message[11];
	fhdr[3] = message[2]; // El primer byte de la tipo
	// FCtrl Uplink
	fhdr[4] = 0x00;
	// FCnt
	fhdr[5] = message[14];
	fhdr[6] = message[15];

	uint16_t FCount = (((uint16_t)fhdr[6]) << 8) | ((uint16_t)fhdr[5]);
	dbgp("FCount ");
	dbgpl(FCount);
	uint8_t fport = 0x10;
	

	uint8_t temp_buffer[100];

	uint8_t AppSKey[] = _APPSKEY;
	uint8_t NwkSKey[] = _NWKSKEY;
	uint8_t DevAddr[4];
	DevAddr[0] = message[2];
	DevAddr[1] = message[11];
	DevAddr[2] = message[12];
	DevAddr[3] = message[13];
	// Encryptacion del payload FRMPayload usando la AppSKey
	uint8_t CodeLength = encodePacket((uint8_t *)(message), *messageLen, FCount, DevAddr, AppSKey, 0);

	temp_buffer[0] = mhdr;
	memcpy(&temp_buffer[1], fhdr, 7);
	temp_buffer[8] = fport;
	memcpy(&temp_buffer[9], message, CodeLength); // Copio el FRMPayload encriptado

	(*messageLen) = CodeLength + 9;

	// Calculo MIC con la NwkSKey y DevAdr. micPacket agrega el mic al final
	(*messageLen) += micPacket((uint8_t *)(temp_buffer), *messageLen, FCount, DevAddr, NwkSKey, 0);

	// Copio temp_buffer en el buffer original.
	memcpy(message, temp_buffer, *messageLen);

}

// ----------------------------------------------------------------------------
// DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN DOWN
// Send DOWN a LoRa packet over the air to the node. This function does all the
// decoding of the server message and prepares a Payload buffer.
// The payload is actually transmitted by the sendPkt() function.
// This function is used for regular downstream messages and for JOIN_ACCEPT
// messages.
// NOTE: This is not an interrupt function, but is started by loop().
// The _status is set an the end of the function to TX and in _stateMachine
// function the actual transmission function is executed.
// The LoraDown.tmst contains the timestamp that the tranmission should finish.
// ----------------------------------------------------------------------------
int sendPacket(uint8_t *buf, uint8_t length)
{
	// Received package with Meta Data (for example):
	// codr	: "4/5"
	// data	: "Kuc5CSwJ7/a5JgPHrP29X9K6kf/Vs5kU6g=="	// for example
	// freq	: 868.1 									// 868100000
	// ipol	: true/false
	// modu : "LORA"
	// powe	: 14										// Set by default
	// rfch : 0											// Set by default
	// size	: 21
	// tmst : 1800642 									// for example
	// datr	: "SF7BW125"

	// 12-byte header;
	//		HDR (1 byte)
	//
	//
	// Data Reply for JOIN_ACCEPT as sent by server:
	//		AppNonce (3 byte)
	//		NetID (3 byte)
	//		DevAddr (4 byte) [ 31..25]:NwkID , [24..0]:NwkAddr
	//		DLSettings (1 byte)
	//		RxDelay (1 byte)
	//		CFList (fill to 16 bytes)

	
	StaticJsonDocument<312> jsonBuffer;
	char *bufPtr = (char *)(buf);
	buf[length] = 0;

#if DUSB >= 1
	if (debug >= 2)
	{
		dbgpl((char *)buf);
		dbgp(F("<"));
		Serial.flush();
	}
#endif
	// Use JSON to decode the string after the first 4 bytes.
	// The data for the node is in the "data" field. This function destroys original buffer
	auto error = deserializeJson(jsonBuffer, bufPtr);

	if (error)
	{
#if DUSB >= 1
		if ((debug >= 1) && (pdebug & P_TX))
		{
			dbgp(F("T sendPacket:: ERROR Json Decode"));
			if (debug >= 2)
			{
				dbgp(':');
				dbgpl(bufPtr);
			}
			Serial.flush();
		}
#endif
		return (-1);
	}
	yield();

	// Meta Data sent by server (example)
	// {"txpk":{"codr":"4/5","data":"YCkEAgIABQABGmIwYX/kSn4Y","freq":868.1,"ipol":true,"modu":"LORA","powe":14,"rfch":0,"size":18,"tmst":1890991792,"datr":"SF7BW125"}}

	// Used in the protocol of Gateway:
	JsonObject root = jsonBuffer.as<JsonObject>();
	const char *data = root["txpk"]["data"]; // Downstream Payload
	uint8_t psize = root["txpk"]["size"];
	bool ipol = root["txpk"]["ipol"];
	uint8_t powe = root["txpk"]["powe"]; // e.g. 14 or 27
	LoraDown.tmst = (uint32_t)root["txpk"]["tmst"].as<unsigned long>();
	const float ff = root["txpk"]["freq"]; // eg 869.525

	// Not used in the protocol of Gateway TTN:
	const char *datr = root["txpk"]["datr"]; // eg "SF7BW125"
	//const char *modu = root["txpk"]["modu"]; // =="LORA"
	//const char *codr = root["txpk"]["codr"]; // e.g. "4/5"
	//if (root["txpk"].containsKey("imme") ) {
	//	const bool imme = root["txpk"]["imme"];			// Immediate Transmit (tmst don't care)
	//}

	if (data != NULL)
	{
#if DUSB >= 1
		if ((debug >= 2) && (pdebug & P_TX))
		{
			dbgp(F("T data: "));
			dbgpl((char *)data);
			if (debug >= 2)
				Serial.flush();
		}
#endif
	}
	else
	{ // There is no data!
#if DUSB >= 1
		if ((debug > 0) && (pdebug & P_TX))
		{
			dbgpl(F("T sendPacket:: ERROR: data is NULL"));
			if (debug >= 2)
				Serial.flush();
		}
#endif
		return (-1);
	}

	LoraDown.sfTx = atoi(datr + 2); // Convert "SF9BW125" or what is received from gateway to number
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

	LoraDown.iiq = (ipol ? 0x40 : 0x27);								   // if ipol==true 0x40 else 0x27
	LoraDown.crc = 0x00;												   // switch CRC off for TX
	LoraDown.payLength = Base64.decodedLength((char *)data, strlen(data)); // Length of the Payload data
	Base64.decode((char *)payLoad, (char *)data, strlen(data));			   // Fill payload w decoded message

	// Compute wait time in microseconds
	// uint32_t w = (uint32_t)(LoraDown.tmst - micros()); // Wait Time compute

// _STRICT_1CH determines ho we will react on downstream messages.
// If STRICT==0, we will receive messags from the TTN gateway presumably on SF12/869.5MHz
// And since the Gateway is a single channel gateway, and its nodes are probably
// single channle too. They will not listen to that frequency.
// When STRICT==1, we will answer (in the RX1 timeslot) on the frequency we receive on.
//
#if _STRICT_1CH == 1
	// If possible use RX1 timeslot as this is our frequency.
	// Do not use RX2 or JOIN2 as they contain other frequencies

	if ((w > 1000000) && (w < 3000000))
	{
		LoraDown.tmst -= 1000000;
	} // Is tmst correction necessary
	else if ((w > 6000000) && (w < 7000000))
	{
		LoraDown.tmst -= 500000;
	}
	LoraDown.powe = 14; // On all freqs except 869.5MHz power is limited
	//LoraDown.sfTx = sfi;									// Take care, TX sf not to be mixed with SCAN
	LoraDown.fff = freq; // Use the current frequency
#else
	LoraDown.powe = powe;

	// convert double frequency (MHz) into uint32_t frequency in Hz.
	LoraDown.fff = (uint32_t)((uint32_t)((ff + 0.000035) * 1000)) * 1000;
#endif

	LoraDown.payLoad = payLoad;

#if DUSB >= 1
	if ((debug >= 1) && (pdebug & P_TX))
	{

		dbgp(F("T LoraDown tmst="));
		dbgp(LoraDown.tmst);
		//dbgp(F(", w="));
		//dbgp(w);

		if (debug >= 2)
		{
			dbgp(F(" Request:: "));
			dbgp(F(" tmst="));
			dbgp(LoraDown.tmst);
			dbgp(F(" wait="));
			dbgpl(w);

			dbgp(F(" strict="));
			dbgp(_STRICT_1CH);
			dbgp(F(" datr="));
			dbgpl(datr);
			dbgp(F(" Rfreq="));
			dbgp(freq);
			dbgp(F(", Request="));
			dbgp(freq);
			dbgp(F(" ->"));
			dbgpl(LoraDown.fff);
			dbgp(F(" sf  ="));
			dbgp(atoi(datr + 2));
			dbgp(F(" ->"));
			dbgpl(LoraDown.sfTx);

			dbgp(F(" modu="));
			dbgpl(modu);
			dbgp(F(" powe="));
			dbgpl(powe);
			dbgp(F(" codr="));
			dbgpl(codr);

			dbgp(F(" ipol="));
			dbgpl(ipol);
		}
		dbgpl();
	}
#endif

	if (LoraDown.payLength != psize)
	{
#if DUSB >= 1
		dbgp(F("sendPacket:: WARNING payLength: "));
		dbgp(LoraDown.payLength);
		dbgp(F(", psize="));
		dbgpl(psize);
		if (debug >= 2)
			Serial.flush();
#endif
	}
#if DUSB >= 1
	else if ((debug >= 2) && (pdebug & P_TX))
	{
		dbgp(F("T Payload="));
		int i;
		for (i = 0; i < LoraDown.payLength; i++)
		{
			dbgp(payLoad[i], HEX);
			dbgp(':');
		}
		dbgpl();
		if (debug >= 2)
			Serial.flush();
	}
#endif
	cp_up_pkt_fwd++;

#if DUSB >= 1
	if ((debug >= 2) && (pdebug & P_TX))
	{
		dbgpl(F("T sendPacket:: fini OK"));
	}
#endif // DUSB

	// All data is in Payload and parameters and need to be transmitted.
	// The function is called in user-space
	_state = S_TX; // _state set to transmit

	return 1;
} //sendPacket

// ----------------------------------------------------------------------------
// UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP
// Based on the information read from the LoRa transceiver (or fake message)
// build a gateway message to send upstream (to the user somewhere on the web).
//
// parameters:
// 	tmst: Timestamp to include in the upstream message
// 	buff_up: The buffer that is generated for upstream
// 	message: The payload message to include in the the buff_up
//	messageLength: The number of bytes received by the LoRa transceiver
// 	internal: Boolean value to indicate whether the local sensor is processed
//
// returns:
//	buff_index
// ----------------------------------------------------------------------------
int buildPacket(uint32_t tmst, uint8_t *buff_up, struct LoraUp up_packet, bool internal)
{
	long SNR;
	int rssicorr;
	int prssi; // packet rssi

	char cfreq[12] = {0}; // Character array to hold freq in MHz
	//lastTmst = tmst;									// Following/according to spec
	int buff_index = 0;
	

	uint8_t *message = up_packet.payLoad;
	char messageLength = up_packet.payLength;

	
	// Chequear si el paquete es custom agrotools
	dbgpl("RX lora");
	for (uint8_t i = 0; i < up_packet.payLength; i++)
	{
		if (message[i] < 0x10)
		{
			dbgp(0, HEX);
		}
		dbgp(message[i], HEX);
		dbgp(" ");
	}

	dbgpl("");

	dbgp("up_packet Length ");
	dbgpl((int)messageLength);
	dbgp("Calculed Length ");
	dbgpl(message[19] + 20); // 16 = payload + overhead
	if(settings_protocol() == MODO_AGROTOOLS){
		LoraUp.sf = 12;
		// Use gBase64 library to fill in the data string
		uint8_t tmp[128];
		if(messageLength <= 128){
			LoraUp.payLength = Base64.encode((char *)tmp, (char *)message, messageLength);
			if(LoraUp.payLength < 128){
				memcpy((char *)LoraUp.payLoad, tmp, LoraUp.payLength);
			}else{
				return 0;
			}
		}
	}
	else if ((messageLength == (message[19] + 20)) || (messageLength == (message[19] + 16)))
	{
		// Custom agrotools - Agregar LoraWan wrap
		dbgpl("Custom agrotools");
		fakeLoraWanWrap(message, (uint8_t *)&messageLength);
		gbl_fakeLoraWanFlag = true;
		
		// Actualizo el objeto global LoraUp
		LoraUp.payLength = messageLength;
		LoraUp.sf = 10;
		if(messageLength <= 128){
			memcpy(LoraUp.payLoad,message,messageLength);
		}

		dbgpl("RX fake LoraWan wrap");
		for (uint8_t i = 0; i < messageLength; i++)
		{
			if (message[i] < 0x10)
			{
				dbgp(0, HEX);
			}
			dbgp(message[i], HEX);
			dbgp(" ");
		}

		dbgpl("");
	}
	else
	{
		// No es custom agrotools - Procesar normalmente
	}

#if _CHECK_MIC == 1
	unsigned char NwkSKey[16] = _NWKSKEY;
	checkMic(message, messageLength, NwkSKey);
#endif // _CHECK_MIC

	// Read SNR and RSSI from the register. Note: Not for internal sensors!
	// For internal sensor we fake these values as we cannot read a register
	if (internal)
	{
		SNR = 12;
		prssi = 50;
		rssicorr = 157;
	}
	else
	{
		SNR = up_packet.snr;
		prssi = up_packet.prssi; // read register 0x1A, packet rssi
		rssicorr = up_packet.rssicorr;
	}

#if STATISTICS >= 1
	// Receive statistics, move old statistics down 1 position
	// and fill the new top line with the latest received sensor values.
	// This works fine for the sensor, EXCEPT when we decode data for _LOCALSERVER
	//
	for (int m = (MAX_STAT - 1); m > 0; m--)
		statr[m] = statr[m - 1];

		// From now on we can fill start[0] with sensor data
#if _LOCALSERVER == 1
	statr[0].datal = 0;
	int index;
	if ((index = inDecodes((char *)(up_packet.payLoad + 1))) >= 0)
	{

		uint16_t frameCount = up_packet.payLoad[7] * 256 + up_packet.payLoad[6];

		for (int k = 0; (k < up_packet.payLength) && (k < 23); k++)
		{
			statr[0].data[k] = up_packet.payLoad[k + 9];
		};

		// XXX Check that k<23 when leaving the for loop
		// XXX or we can not display in statr

		uint8_t DevAddr[4];
		DevAddr[0] = up_packet.payLoad[4];
		DevAddr[1] = up_packet.payLoad[3];
		DevAddr[2] = up_packet.payLoad[2];
		DevAddr[3] = up_packet.payLoad[1];

		statr[0].datal = encodePacket((uint8_t *)(statr[0].data),
									  up_packet.payLength - 9 - 4,
									  (uint16_t)frameCount,
									  DevAddr,
									  decodes[index].appKey,
									  0);
	}
#endif //_LOCALSERVER
	statr[0].tmst = now();

	
	statr[0].ch = ifreq;
	statr[0].prssi = prssi - rssicorr;
#if RSSI == 1
	statr[0].rssi = _rssi - rssicorr;
#endif // RSII
	statr[0].sf = up_packet.sf;
#if DUSB >= 2
	if (debug >= 0)
	{
		if ((message[4] != 0x26) || (message[1] == 0x99))
		{
			dbgp(F("addr="));
			for (int i = messageLength; i > 0; i--)
			{
				if (message[i] < 0x10)
					dbgp('0');
				dbgp(message[i], HEX);
				dbgp(' ');
			}
			dbgpl();
		}
	}
#endif //DUSB
	statr[0].node = (message[1] << 24 | message[2] << 16 | message[3] << 8 | message[4]);

#if STATISTICS >= 2
	// Fill in the statistics that we will also need for the GUI.
	// So
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
#endif //STATISTICS >= 2

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
#endif //STATISTICS >= 3

#endif //STATISTICS >= 2

#if DUSB >= 1
	if ((debug >= 2) && (pdebug & P_RADIO))
	{
		dbgp(F("R buildPacket:: pRSSI="));
		dbgp(prssi - rssicorr);
		dbgp(F(" RSSI: "));
		dbgp(_rssi - rssicorr);
		dbgp(F(" SNR: "));
		dbgp(SNR);
		dbgp(F(" Length: "));
		dbgp((int)messageLength);
		dbgp(F(" -> "));
		int i;
		for (i = 0; i < messageLength; i++)
		{
			dbgp(message[i], HEX);
			dbgp(' ');
		}
		dbgpl();
		yield();
	}
#endif // DUSB


	if(settings_protocol()==MODO_AGROTOOLS){
		return 0;
	}

	int j;

	// XXX Base64 library is nopad. So we may have to add padding characters until
	// 	message Length is multiple of 4!
	// Encode message with messageLength into b64
#if DUSB >= 1
	int encodedLen = Base64.encodedLength(messageLength); // max 341
	if ((debug >= 1) && (encodedLen > 255) && (pdebug & P_RADIO))
	{
		dbgp(F("R buildPacket:: b64 err, len="));
		dbgpl(encodedLen);
		if (debug >= 2)
			Serial.flush();
		return (-1);
	}
#endif													// DUSB
	//Base64.encode(b64, (char *)message, messageLength); // max 341
	// start composing datagram with the header
	uint8_t token_h = (uint8_t)rand(); // random token
	uint8_t token_l = (uint8_t)rand(); // random token

	// pre-fill the data buffer with fixed fields
	buff_up[0] = PROTOCOL_VERSION; // 0x01 still

	buff_up[1] = token_h;
	buff_up[2] = token_l;

	buff_up[3] = PKT_PUSH_DATA; // 0x00

	// READ MAC ADDRESS OF ESP8266, and insert 0xFF 0xFF in the middle
	buff_up[4] = MAC_array[0];
	buff_up[5] = MAC_array[1];
	buff_up[6] = MAC_array[2];
	buff_up[7] = 0xFF;
	buff_up[8] = 0xFF;
	buff_up[9] = MAC_array[3];
	buff_up[10] = MAC_array[4];
	buff_up[11] = MAC_array[5];

	buff_index = 12; // 12-byte binary (!) header

	// start of JSON structure that will make payload
	memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[", 9);
	buff_index += 9;
	buff_up[buff_index] = '{';
	++buff_index;
	j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index, "\"tmst\":%u", tmst);
#if DUSB >= 1
	if ((j < 0) && (debug >= 1) && (pdebug & P_RADIO))
	{
		dbgpl(F("buildPacket:: Error "));
	}
#endif
	buff_index += j;
	ftoa(((double)freq) / 1000000.0, cfreq, 6); // XXX This can be done better
	j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index, ",\"chan\":%1u,\"rfch\":%1u,\"freq\":%s", 0, 0, cfreq);
	buff_index += j;
	memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
	buff_index += 9;
	memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
	buff_index += 14;

	/* Lora datarate & bandwidth, 16-19 useful chars */
	switch (up_packet.sf)
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

	// Use gBase64 library to fill in the data string
	// encodedLen = Base64.encodedLength(messageLength); // max 341
	j = Base64.encode((char *)(buff_up + buff_index), (char *)message, messageLength);

	buff_index += j;
	buff_up[buff_index] = '"';
	++buff_index;

	// End of packet serialization
	buff_up[buff_index] = '}';
	++buff_index;
	buff_up[buff_index] = ']';
	++buff_index;

	// end of JSON datagram payload */
	buff_up[buff_index] = '}';
	++buff_index;
	buff_up[buff_index] = 0; // add string terminator, for safety

#if STAT_LOG == 1
	// Do statistics logging. In first version we might only
	// write part of the record to files, later more

	addLog((unsigned char *)(buff_up), buff_index);
#endif

#if DUSB >= 1
	if ((debug >= 2) && (pdebug & P_RX))
	{
		dbgp(F("R RXPK:: "));
		dbgpl((char *)(buff_up + 12)); // debug: display JSON payload
		dbgp(F("R RXPK:: package length="));
		dbgpl(buff_index);
	}
#endif
	return (buff_index);
} // buildPacket

// ----------------------------------------------------------------------------
// UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP UP
// Receive a LoRa package over the air, LoRa and deliver to server(s)
//
// Receive a LoRa message and fill the buff_up char buffer.
// returns values:
// - returns the length of string returned in buff_up
// - returns -1 or -2 when no message arrived, depending connection.
//
// This is the "highlevel" function called by loop()
// ----------------------------------------------------------------------------
int receivePacket()
{
	uint8_t buff_up[TX_BUFF_SIZE]; // buffer to compose the upstream packet to backend server
	



	// Regular message received, see SX1276 spec table 18
	// Next statement could also be a "while" to combine several messages received
	// in one UDP message as the Semtech Gateway spec does allow this.
	// XXX Not yet supported

	// Take the timestamp as soon as possible, to have accurate reception timestamp
	// TODO: tmst can jump if micros() overflow.
	uint32_t tmst = (uint32_t)micros(); // Only microseconds, rollover in 5X minutes
	//lastTmst = tmst;								// Following/according to spec
	dbgp("RX tmst ");dbgpl(tmst);
	
	LoraUp.rx_local_timestamp = tmst;
	// Handle the physical data read from LoraUp
	if (LoraUp.payLength > 0)
	{
		//Serial.println("PACKETTRX");
		// externally received packet, so last parameter is false (==LoRa external)
		int build_index = buildPacket(tmst, buff_up, LoraUp, false);

		// REPEATER is a special function where we retransmit received
		// message on _ICHANN to _OCHANN.
		// Note:: For the moment _OCHANN is not allowed to be same as _ICHANN
#if REPEATER == 1
		if (!sendLora(LoraUp.payLoad, LoraUp.payLength))
		{
			return (-3);
		}
#endif

		// This is one of the potential problem areas.
		// If possible, USB traffic should be left out of interrupt routines
		// rxpk PUSH_DATA received from node is rxpk (*2, par. 3.2)
#ifdef _TTNSERVER

		//#Punto de envio
		S_PROTOCOL proto = settings_protocol();
		dbgp("up SF ");dbgpl(LoraUp.sf);
		if(gbl_fakeLoraWanFlag == true){
		//	LoraUp.sf = 10; // Cambiar el SF artificialmente para subir el limite de bytes
			gbl_fakeLoraWanFlag = false;
		}
		if (proto == MQTTBRIDGE_TCP)
		{
			mqtt_sendUplink(LoraUp);
		}
		else if (proto == SEMTECH_PF_UDP)
		{
			// Serial.write((char*)buff_up, build_index);
			// if (!sendUdp(ttnServer, _TTNPORT, buff_up, build_index))
			// {
			// 	return (-1); // received a message
			// }
		}else if( proto == MODO_AGROTOOLS){
			mqtt_sendTBPacket(LoraUp);
		}

		yield();
#endif
		// Use our own defined server or a second well kon server
#ifdef _THINGSERVER
		if (!sendUdp(thingServer, _THINGPORT, buff_up, build_index))
		{
			return (-2); // received a message
		}
#endif

		// Reset the message area
		LoraUp.payLength = 0;
		LoraUp.payLoad[0] = 0x00;
		return (build_index);
	}

	return (0); // failure no message read

} //receivePacket



// ----------------------------------------------------------------------------
// MICPACKET()
// Provide a valid MIC 4-byte code (par 2.4 of spec, RFC4493)
// 		see also https://tools.ietf.org/html/rfc4493
//
// Although our own handler may choose not to interpret the last 4 (MIC) bytes
// of a PHYSPAYLOAD physical payload message of in internal sensor,
// The official TTN (and other) backends will intrpret the complete message and
// conclude that the generated message is bogus.
// So we sill really simulate internal messages coming from the -1ch gateway
// to come from a real sensor and append 4 MIC bytes to every message that are 
// perfectly legimate
// Parameters:
//	- data:			uint8_t array of bytes = ( MHDR | FHDR | FPort | FRMPayload )
//	- len:			8=bit length of data, normally less than 64 bytes
//	- FrameCount:	16-bit framecounter
//	- dir:			0=up, 1=down
//
// B0 = ( 0x49 | 4 x 0x00 | Dir | 4 x DevAddr | 4 x FCnt |  0x00 | len )
// MIC is cmac [0:3] of ( aes128_cmac(NwkSKey, B0 | Data )
//
// ----------------------------------------------------------------------------
uint8_t micPacket(uint8_t *data, uint8_t len, uint16_t FrameCount, uint8_t * DevAddr, uint8_t * NwkSKey, uint8_t dir) {


	//uint8_t NwkSKey[16] = _NWKSKEY;
	uint8_t Block_B[16];
	uint8_t X[16];
	uint8_t Y[16];
	
	// ------------------------------------
	// build the B block used by the MIC process
	Block_B[0]= 0x49;						// 1 byte MIC code
			
	Block_B[1]= 0x00;						// 4 byte 0x00
	Block_B[2]= 0x00;
	Block_B[3]= 0x00;
	Block_B[4]= 0x00;
	
	Block_B[5]= dir;						// 1 byte Direction
	
	Block_B[6]= DevAddr[3];					// 4 byte DevAddr
	Block_B[7]= DevAddr[2];
	Block_B[8]= DevAddr[1];
	Block_B[9]= DevAddr[0];
	
	Block_B[10]= (FrameCount & 0x00FF);		// 4 byte FCNT
	Block_B[11]= ((FrameCount >> 8) & 0x00FF);
	Block_B[12]= 0x00; 						// Frame counter upper Bytes
	Block_B[13]= 0x00;						// These are not used so are 0
	
	Block_B[14]= 0x00;						// 1 byte 0x00
	
	Block_B[15]= len;						// 1 byte len
	
	// ------------------------------------
	// Step 1: Generate the subkeys
	//
	uint8_t k1[16];
	uint8_t k2[16];
	generate_subkey(NwkSKey, k1, k2);
	
	// ------------------------------------
	// Copy the data to a new buffer which is prepended with Block B0
	//
	uint8_t micBuf[len+16];					// B0 | data
	for (uint8_t i=0; i<16; i++) micBuf[i]=Block_B[i];
	for (uint8_t i=0; i<len; i++) micBuf[i+16]=data[i];
	
	// ------------------------------------
	// Step 2: Calculate the number of blocks for CMAC
	//
	uint8_t numBlocks = len/16 + 1;			// Compensate for B0 block
	if ((len % 16)!=0) numBlocks++;			// If we have only a part block, take it all
	
	// ------------------------------------
	// Step 3: Calculate padding is necessary
	//
	uint8_t restBits = len%16;				// if numBlocks is not a multiple of 16 bytes
	
	
	// ------------------------------------
	// Step 5: Make a buffer of zeros
	//
	memset(X, 0, 16);
	
	// ------------------------------------
	// Step 6: Do the actual encoding according to RFC
	//
	for(uint8_t i= 0x0; i < (numBlocks - 1); i++) {
		for (uint8_t j=0; j<16; j++) Y[j] = micBuf[(i*16)+j];
		mXor(Y, X);
		AES_Encrypt(Y, NwkSKey);
		for (uint8_t j=0; j<16; j++) X[j] = Y[j];
	}
	

	// ------------------------------------
	// Step 4: If there is a rest Block, padd it
	// Last block. We move step 4 to the end as we need Y
	// to compute the last block
	// 
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
	
	// ------------------------------------
	// Step 7: done, return the MIC size. 
	// Only 4 bytes are returned (32 bits), which is less than the RFC recommends.
	// We return by appending 4 bytes to data, so there must be space in data array.
	//
	data[len+0]=Y[0];
	data[len+1]=Y[1];
	data[len+2]=Y[2];
	data[len+3]=Y[3];
	return 4;
}


// ENCODEPACKET
// In Sensor mode, we have to encode the user payload before sending.
// The same applies to decoding packages in the payload for _LOCALSERVER.
// The library files for AES are added to the library directory in AES.
// For the moment we use the AES library made by ideetron as this library
// is also used in the LMIC stack and is small in size.
//
// The function below follows the LoRa spec exactly.
//
// The resulting mumber of Bytes is returned by the functions. This means
// 16 bytes per block, and as we add to the last block we also return 16
// bytes for the last block.
//
// The LMIC code does not do this, so maybe we shorten the last block to only
// the meaningful bytes in the last block. This means that encoded buffer
// is exactly as big as the original message.
//
// NOTE:: Be aware that the LICENSE of the used AES library files 
//	that we call with AES_Encrypt() is GPL3. It is used as-is,
//  but not part of this code.
//
// cmac = aes128_encrypt(K, Block_A[i])
// ----------------------------------------------------------------------------
uint8_t encodePacket(uint8_t *Data, uint8_t DataLength, uint16_t FrameCount, uint8_t *DevAddr, uint8_t *AppSKey, uint8_t Direction) {

#if DUSB>=1
	if (( debug>=2 ) && ( pdebug & P_GUI )) {
		dbgp(F("G encodePacket:: DevAddr="));
		for (int i=0; i<4; i++ ) { dbgp(DevAddr[i],HEX); dbgp(' '); }
		dbgp(F("G encodePacket:: AppSKey="));
		for (int i=0; i<16; i++ ) { dbgp(AppSKey[i],HEX); dbgp(' '); }
		dbgpl();
	}
#endif

	//unsigned char AppSKey[16] = _APPSKEY ;	// see ESP-sc-gway.h
	uint8_t i, j;
	uint8_t Block_A[16];
	uint8_t bLen=16;						// Block length is 16 except for last block in message
		
	uint8_t restLength = DataLength % 16;	// We work in blocks of 16 bytes, this is the rest
	uint8_t numBlocks  = DataLength / 16;	// Number of whole blocks to encrypt
	if (restLength>0) numBlocks++;			// And add block for the rest if any

	for(i = 1; i <= numBlocks; i++) {
		Block_A[0] = 0x01;
		
		Block_A[1] = 0x00; 
		Block_A[2] = 0x00; 
		Block_A[3] = 0x00; 
		Block_A[4] = 0x00;

		Block_A[5] = Direction;				// 0 is uplink

		Block_A[6] = DevAddr[3];			// Only works for and with ABP
		Block_A[7] = DevAddr[2];
		Block_A[8] = DevAddr[1];
		Block_A[9] = DevAddr[0];

		Block_A[10] = (FrameCount & 0x00FF);
		Block_A[11] = ((FrameCount >> 8) & 0x00FF);
		Block_A[12] = 0x00; 				// Frame counter upper Bytes
		Block_A[13] = 0x00;					// These are not used so are 0

		Block_A[14] = 0x00;

		Block_A[15] = i;

		// Encrypt and calculate the S
		AES_Encrypt(Block_A, AppSKey);
		
		// Last block? set bLen to rest
		if ((i == numBlocks) && (restLength>0)) bLen = restLength;
		
		for(j = 0; j < bLen; j++) {
			*Data = *Data ^ Block_A[j];
			Data++;
		}
	}
	//return(numBlocks*16);			// Do we really want to return all 16 bytes in lastblock
	return(DataLength);				// or only 16*(numBlocks-1)+bLen;
}

// ----------------------------------------------------------------------------
// generate_subkey
// RFC 4493, para 2.3
// ----------------------------------------------------------------------------
static void generate_subkey(uint8_t *key, uint8_t *k1, uint8_t *k2) {

	memset(k1, 0, 16);								// Fill subkey1 with 0x00
	
	// Step 1: Assume k1 is an all zero block
	AES_Encrypt(k1,key);
	
	// Step 2: Analyse outcome of Encrypt operation (in k1), generate k1
	if (k1[0] & 0x80) {
		shift_left(k1,16);
		k1[15] ^= 0x87;
	}
	else {
		shift_left(k1,16);
	}
	
	// Step 3: Generate k2
	for (uint8_t i=0; i<16; i++) k2[i]=k1[i];
	if (k1[0] & 0x80) {								// use k1(==k2) according rfc 
		shift_left(k2,16);
		k2[15] ^= 0x87;
	}
	else {
		shift_left(k2,16);
	}
	
	// step 4: Done, return k1 and k2
	return;
}



// ----------------------------------------------------------------------------
// XOR()
// perform x-or function for buffer and key
// Since we do this ONLY for keys and X, Y we know that we need to XOR 16 bytes.
//
// ----------------------------------------------------------------------------
static void mXor(uint8_t *buf, uint8_t *key) {
	for (uint8_t i = 0; i < 16; ++i) buf[i] ^= key[i];
}


// ----------------------------------------------------------------------------
// SHIFT-LEFT
// Shift the buffer buf left one bit
// Parameters:
//	- buf: An array of uint8_t bytes
//	- len: Length of the array in bytes
// ----------------------------------------------------------------------------
static void shift_left(uint8_t * buf, uint8_t len) {
    while (len--) {
        uint8_t next = len ? buf[1] : 0;			// len 0 to 15

        uint8_t val = (*buf << 1);
        if (next & 0x80) val |= 0x01;
        *buf++ = val;
    }
}
