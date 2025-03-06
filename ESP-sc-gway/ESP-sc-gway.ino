#include "ESP-sc-gway.h"

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

#include <Arduino.h>

#include <SPI.h>
#include <TimeLib.h>   // http://playground.arduino.cc/code/time
#include <DNSServer.h> // Local DNSserver
#include <ArduinoJson.h>
#include <FS.h> // ESP8266 Specific
#include <WiFiUdp.h>
#include <pins_arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AM2315.h>
#include <RTClib.h>

// Local include files
#include "_loraModem.h"
#include "loraFiles.h"

// Interfaz de administracion
#include "asi-src/src/asi.h"
#include "asi-src/src/PacketForwarder.h"

#include "estado.h"

#include "utils.h"
#include <pb_common.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include "protobuf/gw.pb.h"

#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>

#include "macro_helpers.h"

// Helpers para acceder a las settings
#include "settings.h"

extern "C"
{
#include "lwip/err.h"
#include "lwip/dns.h"
}

static char MAC_char[19];

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

#endif //ESP_ARCH

//
Adafruit_BMP280 bmp;
Adafruit_AM2315 am2315;
RTC_DS3231 rtc;

// ----------- Declaration of vars --------------
uint8_t debug = 1;	   // Debug level! 0 is no msgs, 1 normal, 2 extensive
uint8_t pdebug = 0xFF; // Allow all atterns (departments)

using namespace std;

byte currentMode = 0x81;

bool sx1272 = true; // Actually we use sx1276/RFM95

uint32_t cp_nb_rx_rcv;		// Number of messages received by gateway
uint32_t cp_nb_rx_ok;		// Number of messages received OK
uint32_t cp_nb_rx_bad;		// Number of messages received bad
uint32_t cp_nb_rx_nocrc;	// Number of messages without CRC
uint32_t cp_up_pkt_fwd;		// Number of packets forwarded
uint32_t cp_up_pkt_up_fail; // Number of packets received but not forwarded
uint32_t cp_dwnb;			// Downlink recibidos
uint32_t cp_txnb;			// Paquetes TX
bool broker_conn;

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

#if NTP_INTR == 0
uint32_t ntptimer = 0;
#endif

bool time_sync = false;

#define TX_BUFF_SIZE 1024 // Upstream buffer to send to MQTT
#define RX_BUFF_SIZE 1024 // Downstream received from MQTT
#define STATUS_SIZE 512	  // Should(!) be enough based on the static text .. was 1024

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

bool mqtt_reconnect();
void mqtt_callback(char *topic, byte *payload, unsigned int length);
int sendPacket(uint8_t *buf, uint8_t length); // _txRx.ino
void SerialTime();							  // _utils.ino

void initLoraModem(); // _loraModem.ino
void cadScanner();
void rxLoraModem();								 // _loraModem.ino
void writeRegister(uint8_t addr, uint8_t value); // _loraModem.ino

void stateMachine(); // _stateMachine.ino

void SerialStat(uint8_t intr); // _utils.ino

// ----------------------------------------------------------------------------
// Print leading '0' digits for hours(0) and second(0) when
// printing values less than 10
// ----------------------------------------------------------------------------
void printDigits(unsigned long digits)
{
	// utility function for digital clock display: prints leading 0
	if (digits < 10)
	{
		dbgp(F("0"));
	}
	dbgp(digits);
}

// ----------------------------------------------------------------------------
// Print utin8_t values in HEX with leading 0 when necessary
// ----------------------------------------------------------------------------
void printHexDigit(uint8_t digit)
{
	// utility function for printing Hex Values with leading 0
	if (digit < 0x10)
	{
		dbgp('0');
	}
	dbgp(digit, HEX);
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

	(void)p;
	int int_part = (int)f;
	float dec_part = f - (float)int_part;

	int int_dec_part = dec_part * (10);
	snprintf(val, 15, "%d.%d00000", int_part, int_dec_part);
}

void heartbeat(){
	static uint32_t st;
	static bool on;
	uint32_t ct = millis();

	if(getEConnMqttHost()){
		// Encender cada 4 segs
		if((ct - st) > 4000){
			digitalWrite(5, HIGH);
			st = ct;
			on = true;
		}

		// 500 ms
		if(on && ((ct - st) > 500)){
			digitalWrite(5, LOW);
			on = false;
		}

	}else{

				// Encender cada 4 segs
		if((ct - st) > 1000){
			digitalWrite(5, HIGH);
			st = ct;
			on = true;
		}

		// 250 ms
		if(on && ((ct - st) > 250)){
			digitalWrite(5, LOW);
			on = false;
		}

	}
}

time_t getTimeFunction()
{
	time_t time_ntp = time(nullptr);

	DateTime ahora = rtc.now();
	time_t time_rtc = ahora.unixtime();

	if (time_rtc < 1577836800)
	{ // Año 2020
		// El RTC no esta ajustado
		if (time_ntp > 1577836800)
		{
			// Asumo NTP ajustado -> Ajustar RTC
			rtc.adjust(DateTime(time_ntp));
			return time_ntp;
		}
		else
		{
			// El ntp tampoco esta ajustado
			return 0;
		}
	}
	else
	{
		// Tiempo Valido
		if (time_sync == false)
		{
			statr[0].tmst = time_rtc;
		}

		time_sync = true;
		return time_rtc;
	}
}
// ============================================================================
// MAIN PROGRAM CODE (SETUP AND LOOP)

// ----------------------------------------------------------------------------
// Setup code (one time)
// _state is S_INIT
// ----------------------------------------------------------------------------
void setup()
{
	//lcd_init();

	// Pins are defined and set in loraModem.h
	pinMode(pins.ss, OUTPUT);
	digitalWrite(pins.ss, HIGH);

	pinMode(pins.rst, OUTPUT); // RESET
	pinMode(pins.dio0, INPUT); // DIO0 This pin is interrupt

	pinMode(5, OUTPUT); // DTR

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

	MAC_char[18] = 0;

	WiFi.setOutputPower(20.5);
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

	estadoInit();

	yield();

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

	setSyncProvider(getTimeFunction); // set the external time provider
	setSyncInterval(60 * 1);		  // set the number of seconds between re-sync

#ifdef CON_EXTENSION

	Wire.begin(2, 0);

	for (int address = 1; address < 127; address++)
	{

		Wire.beginTransmission(address);
		int error = Wire.endTransmission();

		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");
		}
		else if (error == 4)
		{
			Serial.print("Unknow error at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}

	if (!bmp.begin(0x76))
	{
		Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
		wdt_enable(WDTO_8S);
		while (1)
			;
	}

	/* Default settings from datasheet. */
	bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,	  /* Operating Mode. */
					Adafruit_BMP280::SAMPLING_X2,	  /* Temp. oversampling */
					Adafruit_BMP280::SAMPLING_X16,	  /* Pressure oversampling */
					Adafruit_BMP280::FILTER_X16,	  /* Filtering. */
					Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

	if (!rtc.begin())
	{
		Serial.println("Couldn't find RTC");
		wdt_enable(WDTO_8S);
		while (1)
			;
	}

#endif

	dbgpl(F("--------------------------------------"));
} //setup

volatile uint8_t mqtt_down_flag = 0;

void loop()
{

	int packetSize;

	static bool udpInit = false; // Flags para inicializar UDP y Time.
	static bool timeInit = false;

	S_PROTOCOL protocolo = settings_protocol();
	S_BACKBONE backbone = asi_loraSettings()._backbone == 2 ? BACKBONE_GPRS : BACKBONE_WIFI; //settings_backbone();

	asi_loop();		//Interface Loop
	stateMachine(); // do the state machine

	if (timeStatus() != timeSet)
	{
		setSyncProvider(getTimeFunction); // para forzar re-sync
	}
	uint32_t nowSeconds = now(); // Despues de chequear timeStatus

	// Cambiar el cliente para mqtt
	/* Todo la configuracion gprs pasa por aca */
	static uint32_t gprs_cwt = 0; // GPRS Watchdog Timer

	if (backbone == BACKBONE_GPRS)
	{
		mqtt_client.setClient(gprsClient);

		if (((millis() - gprs_cwt) > 20000)) // Chequear cada 20 segs
		{
			gprs_cwt = millis();
			if (!gprs_connected())
			{
				setEConnGprs(false);
				gprs_init();
			}
			else
			{
				setEConnGprs(true);
			}
		}
	}
	else if (backbone == BACKBONE_WIFI)
	{
		mqtt_client.setClient(espClient);
	}

	if (protocolo == MQTTBRIDGE_TCP)
	{
		//mqtt_client.loop();
		if (mqtt_down_flag == 1) // Hay un Downlink packet
		{
			dbgp("mqtt_down ");
			dbgpl(_state);
			mqtt_down_flag = 0;
			//_state = S_TX;
		}
		//yield();
	}

	// After a quiet period, make sure we reinit the modem and state machine.
	// The interval is in seconds (about 15 seconds) as this re-init
	// is a heavy operation.
	// SO it will kick in if there are not many messages for the gateway.
	// Note: Be careful that it does not happen too often in normal operation.
	// Esto ocurre 1 vez a los 15 segundos posteriores al ultimo paquete.
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
			writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t)0x00);
			writeRegister(REG_IRQ_FLAGS, (uint8_t)0xFF); // Reset all interrupt flags
		}
		else
		{
			//_state = S_RX;
			//initLoraModem();
			//rxLoraModem();
		}

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

	static uint32_t timer_stats = 0;
	uint32_t ct = millis();
	if (((ct - timer_stats) / 1000) >= settings_stats_interval())
	{ // Wake up every xx seconds

		timer_stats = ct;
		mqtt_sendStat();

		return;
	}

	if (protocolo == MQTTBRIDGE_TCP || protocolo == MODO_AGROTOOLS)
	{
		//######################### MQTT ############################
		// Reconectar si estoy fuera.
		static uint32_t mqtt_loopwt = 0; // MQTT Loop Watchdog Timer
		static uint32_t last_reconnect_attempt = 0;
		uint32_t t = millis();

		if (((t - mqtt_loopwt) > 5))
		{
			mqtt_loopwt = millis();

			if (!mqtt_client.connected())
			{
				setEConnMqttHost(false);
				if ((t - last_reconnect_attempt) > 10000L)
				{
					last_reconnect_attempt = t;
					if (mqtt_reconnect())
					{
						setEConnMqttHost(true);
						last_reconnect_attempt = 0;
					}
				}
			}

			mqtt_client.loop();
		}
	}

	// Si en 10 min no he recibido nada reiniciar el sistema
	if ((time_sync) && ((nowSeconds - statr[0].tmst) > (10 * 60)))
	{
		char mqtt_stat[200];

		snprintf(mqtt_stat, 200, "{\"freeHeap\":%d,\"heapFrag\":%d,\"forwardFail\": %d,\"msg\":\"NoMsgReinit\",\"DIO0\":%d,\"event\":%d}", ESP.getFreeHeap(), ESP.getHeapFragmentation(), cp_up_pkt_up_fail, digitalRead(15) ? 1 : 0, _event);

		char topic[100];
		snprintf(topic, 100, "gateway/%02x%02x%02xffff%02x%02x%02x/sisreinit", MAC_array[0], MAC_array[1], MAC_array[2], MAC_array[3], MAC_array[4], MAC_array[5]);

		modemWakeup();
		if (!(mqtt_client.publish(topic, mqtt_stat, strlen(mqtt_stat))))
		{
			// Fallo
		}
		modemSleep();

		initLoraModem(); // Esto es para prevenir que D0 ponga en HIGH a GPIO15 al reinicio.
		ESP.reset();
		return;
	}

	/* Cada 300 segundos desde el ultimo paquete reiniciar lora */
	static uint32_t s;
	if ((time_sync) && ((nowSeconds - statr[0].tmst) > (300)) && ((millis() - s) > 300000))
	{
		initLoraModem();
		rxLoraModem();
		_state = S_RX;

		char mqtt_stat[200];
		snprintf(mqtt_stat, 200, "{\"freeHeap\":%d,\"heapFrag\":%d,\"forwardFail\": %d,\"msg\":\"LORAReinit\",\"DIO0\":%d,\"event\":%d}", ESP.getFreeHeap(), ESP.getHeapFragmentation(), cp_up_pkt_up_fail, digitalRead(15) ? 1 : 0, _event);
		char topic[100];
		snprintf(topic, 100, "gateway/%02x%02x%02xffff%02x%02x%02x/lorareinit", MAC_array[0], MAC_array[1], MAC_array[2], MAC_array[3], MAC_array[4], MAC_array[5]);

		modemWakeup();
		if (!(mqtt_client.publish(topic, mqtt_stat, strlen(mqtt_stat))))
		{
			// Fallo
		}
		modemSleep();
		s = millis();
	}

#ifdef CON_EXTENSION
	static uint32_t sensores_timer;
	if ((millis() - sensores_timer) > 7000)
	{
		sensores_timer = millis();
		extension_get_data();
	}
#endif

	heartbeat();

} //loop

void lora_settings_reconfig(int sf, int bw, uint32_t frec)
{
	global_sf = (sf_t)sf;
	freq = frec;
	uint8_t i;
	for (i = 0; i < 10; i++)
	{
		if ((uint32_t)freqs[i] == freq)
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

	if (settings_protocol() == MODO_AGROTOOLS)
	{
		return;
	}

	dbgp("Message arrived [");
	dbgp(topic);
	dbgp("] Length ");
	dbgp(length);
	dbgp(" micros ");
	dbgp(micros());
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
	const char *str;
	char ip[20];
	memset(ip, 0, 20);
	if (WiFi.localIP().isSet())
	{
		memcpy(ip, WiFi.localIP().toString().c_str(), WiFi.localIP().toString().length());
		str = ip; //WiFi.localIP().toString().c_str();//
	}
	else
	{
		str = "192.168.88.4";
	}

	// // TODO

	if (!pb_encode_tag_for_field(stream, field))
		return false;

	return pb_encode_string(stream, (uint8_t *)str, strlen(str));
}

bool pb_set_config_version(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
	const char *str = "1.2.3";

	if (!pb_encode_tag_for_field(stream, field))
		return false;

	return pb_encode_string(stream, (uint8_t *)str, strlen(str));
}

bool pb_set_code_rate(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
	const char *str = "4/5";

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
		{
			dbgp('0');
		}
		dbgp(str[i], HEX);
		dbgp(' ');
	}

	if (!pb_encode_tag_for_field(stream, field))
		return false;

	return pb_encode_string(stream, (uint8_t *)str, downlink_size);
}

bool pb_metadata_key(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
	//const char *str = "temperatura";
	char *str = (char *)*arg;
	if (!pb_encode_tag_for_field(stream, field))
		return false;

	return pb_encode_string(stream, (uint8_t *)str, strlen(str));
}

bool pb_metadata_ip_val(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
	const char *str;
	char ip[20];
	memset(ip, 0, 20);
	if (WiFi.localIP().isSet())
	{
		memcpy(ip, WiFi.localIP().toString().c_str(), WiFi.localIP().toString().length());
		str = ip; //WiFi.localIP().toString().c_str();//
	}
	else
	{
		str = "192.168.88.4";
	}

	if (!pb_encode_tag_for_field(stream, field))
		return false;

	return pb_encode_string(stream, (uint8_t *)str, strlen(str));
}

bool pb_gw_metadata(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
	char *str = (char *)*arg;

	gw_GatewayStats_MetaDataEntry msg = gw_GatewayStats_MetaDataEntry_init_zero;
	gw_GatewayStats_MetaDataEntry msg1 = gw_GatewayStats_MetaDataEntry_init_zero;

	msg.key.funcs.encode = pb_metadata_key;
	msg.key.arg = (void *)"ip";
	msg.value.funcs.encode = pb_metadata_ip_val;

	// msg1.key.funcs.encode = pb_temperatura;
	// msg1.key.arg = (void *)"temperatura";
	// msg1.value.funcs.encode = pb_temperatura_val;

	if (!pb_encode_tag_for_field(stream, field))
		return false;

	//pb_encode_submessage(stream, gw_GatewayStats_MetaDataEntry_fields, &msg1);
	return pb_encode_submessage(stream, gw_GatewayStats_MetaDataEntry_fields, &msg);
}

int extension_get_data()
{
	Wire.requestFrom(0x61, 20); // request 20 bytes from slave device #8

	uint8_t buffer[20];
	uint8_t i = 0;

	while (Wire.available())
	{							   // slave may send less than requested
		buffer[i++] = Wire.read(); // receive a byte as character
	}

	gbl_sensores.latitud = (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
	gbl_sensores.longitud = (buffer[7] << 24) | (buffer[6] << 16) | (buffer[5] << 8) | buffer[4];

	//	uint16_t temperatura = (buffer[9] << 8) | buffer[8]; // Solo con ONE WIRE
	//	uint16_t humedad = (buffer[11] << 8) | buffer[10] // Solo con ONE WIRE

	/// EQUIPO ARMADO AL REVES!!!
	if ((MAC_array[5] == 0x84) && (MAC_array[4] == 0x68))
	{
		gbl_sensores.direccion = (buffer[13] << 8) | buffer[12];
		gbl_sensores.velocidad = (buffer[15] << 8) | buffer[14];
	}
	else
	{
		gbl_sensores.velocidad = (buffer[13] << 8) | buffer[12];
		gbl_sensores.direccion = (buffer[15] << 8) | buffer[14];
	}

	gbl_sensores.pulsos = (buffer[17] << 8) | buffer[16];
	gbl_sensores.tension = (buffer[19] << 8) | buffer[18];

	// Conversion de unidades ADC -> Magnitudes fisicas.
	// tension a centesimas de volts.
	gbl_sensores.tension = (uint16_t)((float)gbl_sensores.tension * TENSION_CAL);
	// direccion a grados
	gbl_sensores.direccion = (gbl_sensores.direccion < DIRECCION_MIN) ? 0 : ((gbl_sensores.direccion - DIRECCION_MIN) * 360 / (DIRECCION_MAX - DIRECCION_MIN)); //(uint16)((float)gbl_sensores.direccion * DIRECCION_CAL);
	// velocidad a centimetros por segundo
	gbl_sensores.velocidad = (gbl_sensores.velocidad < VELOCIDAD_MIN) ? 0 : ((gbl_sensores.velocidad - VELOCIDAD_MIN) * 3000 / (VELOCIDAD_MAX - VELOCIDAD_MIN)); //(uint16)((float)gbl_sensores.direccion * DIRECCION_CAL);

	float presion_f = bmp.readPressure();
	float temperatura_f, humedad_f;
	if (!am2315.readTemperatureAndHumidity(&temperatura_f, &humedad_f))
	{
		temperatura_f = -273.15;
		humedad_f = 0;

		//No calcules
	}
	else
	{
		gbl_sensores.temperatura = (uint16_t)(temperatura_f + 273.15) * 100;
		gbl_sensores.humedad = (uint16_t)(humedad_f * 100);
	}

	gbl_sensores.presion = (uint16_t)(presion_f / 10.0);
}

int sensors_json(char *b)
{
	//extension_get_data();
	int n = snprintf(b, 200, "{\"latitud\":%8.6f, \"longitud\":%8.6f, \"temperatura\":%6.2f, \"humedad\":%6.2f, \"presion\":%6.1f, \"velocidad\":%5.2f, \"direccion\":%d, \"pulsos\":%d, \"tension\":%5.2f}",
					 ((float)gbl_sensores.latitud - 90000000.0) / 1000000.0,
					 ((float)gbl_sensores.longitud - 180000000.0) / 1000000.0,
					 ((float)gbl_sensores.temperatura - 27315) / 100.0,
					 gbl_sensores.humedad / 100.0,
					 gbl_sensores.presion / 10.0,
					 gbl_sensores.velocidad != 0 ? gbl_sensores.velocidad / 27.777777777 : 0, // a km/h,
					 gbl_sensores.direccion,
					 gbl_sensores.pulsos,
					 ((float)gbl_sensores.tension) / 100.0);

	return n;
}

void mqtt_sendStat()
{
	if (settings_protocol() == MODO_AGROTOOLS)
	{
		char mqtt_stat[200];
		snprintf(mqtt_stat, 200, "{\"freeHeap\":%d,\"heapFrag\":%d,\"forwardFail\": %d}", ESP.getFreeHeap(), ESP.getHeapFragmentation(), cp_up_pkt_up_fail);

		modemWakeup();
		if (!(mqtt_client.publish("v1/devices/me/telemetry", mqtt_stat, strlen(mqtt_stat))))
		{
			// Fallo
		}
		modemSleep();

		return;
	}

	gw_GatewayStats statsMsg = gw_GatewayStats_init_zero;

	dbgpl("NOWSS()");
	dbgpl(now());
	statsMsg.gateway_id.funcs.encode = pb_set_gateway_id;
	statsMsg.ip.funcs.encode = pb_set_ip;
	statsMsg.has_time = true;
	statsMsg.time.nanos = 0;
	statsMsg.time.seconds = now();
	statsMsg.has_location = true;
	statsMsg.location.source = common_LocationSource_CONFIG;
	statsMsg.location.altitude = 0;

	double latitud = ((float)gbl_sensores.latitud - 90000000.0) / 1000000.0;
	double longitud = ((float)gbl_sensores.longitud - 180000000.0) / 1000000.0;

	statsMsg.location.latitude = latitud;
	statsMsg.location.longitude = longitud;

	statsMsg.config_version.funcs.encode = pb_set_config_version;
	statsMsg.rx_packets_received = cp_nb_rx_rcv;
	statsMsg.rx_packets_received_ok = cp_nb_rx_ok;
	statsMsg.tx_packets_received = cp_dwnb;
	statsMsg.tx_packets_emitted = cp_txnb;
	statsMsg.meta_data.funcs.encode = pb_gw_metadata;

	uint8_t buffer[120];
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	if (!pb_encode(&stream, gw_GatewayStats_fields, &statsMsg))
	{
		Serial.println("Encoding failed: ");
		Serial.println(PB_GET_ERROR(&stream));
		return;
	}

	char topic[40];
	snprintf(topic, 40, "gateway/%02x%02x%02xffff%02x%02x%02x/event/stats", MAC_array[0], MAC_array[1], MAC_array[2], MAC_array[3], MAC_array[4], MAC_array[5]);
	// Stats Chirpstack
	mqtt_client.publish(topic, buffer, stream.bytes_written);

#ifdef CON_EXTENSION
	// Sensores
	snprintf(topic, 40, "gateway/%02x%02x%02xffff%02x%02x%02x/sensores", MAC_array[0], MAC_array[1], MAC_array[2], MAC_array[3], MAC_array[4], MAC_array[5]);

	char sensores[200];
	int n = sensors_json(sensores);
	mqtt_client.publish(topic, sensores, n);

#endif

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
	// OJO
	upMsg.rx_info.rssi = up_packet.prssi;
	upMsg.rx_info.lora_snr = up_packet.snr;

	upMsg.rx_info.channel = 1;

	upMsg.rx_info.location.source = common_LocationSource_CONFIG;

	double latitud = ((float)gbl_sensores.latitud - 90000000.0) / 1000000.0;
	double longitud = ((float)gbl_sensores.longitud - 180000000.0) / 1000000.0;

	upMsg.rx_info.location.latitude = latitud;
	upMsg.rx_info.location.longitude = longitud;

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

	char topic[40];
	snprintf(topic, 40, "gateway/%02x%02x%02xffff%02x%02x%02x/event/up", MAC_array[0], MAC_array[1], MAC_array[2], MAC_array[3], MAC_array[4], MAC_array[5]);
	mqtt_client.publish(topic, buffer, stream.bytes_written);
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

	char topic[40];
	snprintf(topic, 40, "gateway/%02x%02x%02xffff%02x%02x%02x/event/ack", MAC_array[0], MAC_array[1], MAC_array[2], MAC_array[3], MAC_array[4], MAC_array[5]);

	mqtt_client.publish(topic, buffer, stream.bytes_written);
}

bool mqtt_reconnect()
{
	// Loop until we're reconnected
	modemWakeup();

	dbgp("Attempting MQTT connection..");
	// Create a random client ID
	String clientId = "ESP8266Client-";
	clientId += String(random(0xffff), HEX);
	// Attempt to connect
	if (settings_protocol() == MODO_AGROTOOLS)
	{
		if (mqtt_client.connect(clientId.c_str(), settings_tb_mqtt_user(), "", "v1/devices/me/telemetry", 0, 0, "{\"msg\":\"Offline\"}"))
		{
			mqtt_client.publish("v1/devices/me/telemetry", "{\"msg\":\"On\"}");
			mqtt_client.subscribe("v1/devices/me/rpc/request/+");
			setEConnMqttHost(true);
			//modem.sleepEnable(true);
		}
		else
		{
			setEConnMqttHost(false);
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
			char topic[40];
			snprintf(topic, 40, "gateway/%02x%02x%02xffff%02x%02x%02x/command/down", MAC_array[0], MAC_array[1], MAC_array[2], MAC_array[3], MAC_array[4], MAC_array[5]);

			mqtt_client.subscribe(topic);
			return (mqtt_client.connected());
			//mqtt_client.subscribe("gateway/4c11aeffff045b23/command/+");
		}
		else
		{
			dbgp("failed, rc=");
			dbgp(mqtt_client.state());
			dbgpl(" try again in 5 seconds");
			return false;
			// Wait 5 seconds before retrying
			// delay(5000);
		}
	}

	modemSleep();
}

void gprs_init()
{

	modemWakeup();

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

	//String modemInfo = modem.getModemInfo();
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
		setEConnRed(true);
		// SerialMon.println("Network connected");
		// lcd_line3("GPRS Nwrk CONN");
	}
	else
	{
		setEConnRed(false);
	}

	// GPRS connection parameters are usually set after network registration
	//SerialMon.print(F("Connecting to "));
	//SerialMon.print(apn);
	if (!modem.gprsConnect(settings_apn(), settings_gprs_user(), settings_gprs_pass()))
	{
		setEConnGprs(false);
		//SerialMon.println(" fail");
		// delay(10000);
		// lcd_line3("GPRS CONN Fail");
		return;
	}
	//SerialMon.println(" success");

	if (modem.isGprsConnected())
	{
		setEConnGprs(true);
		// SerialMon.println("GPRS connected");
		// lcd_line3("GPRS CONN Ok");
	}

	modemSleep();
}

bool gprs_connected()
{
	modemWakeup();
	bool c = modem.isGprsConnected();
	modemSleep();
	return c;
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

void mqtt_sendTBPacket(struct LoraUp up_packet)
{
	(void)up_packet;
	char mqtt_msg[200];

	strncpy(mqtt_msg, "{\"rxpl\":\"", 150);
	memcpy(mqtt_msg + 9, LoraUp.payLoad, LoraUp.payLength);

	snprintf(mqtt_msg + 9 + LoraUp.payLength, 141, "\",\"heapFrag\":%d, \"freeHeap\":%d}", ESP.getHeapFragmentation(), ESP.getFreeHeap());

	modemWakeup();

	if (mqtt_client.publish("v1/devices/me/telemetry", mqtt_msg, strlen(mqtt_msg)))
	{
		cp_up_pkt_fwd++;
	}
	else
	{
		// retry 1
		//Serial.println("FALLO PUBLISH!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
		//Serial.println(LoraUp.payLength);
		//Serial.write(LoraUp.payLoad,128);
		//Serial.println();
		if (mqtt_client.publish("v1/devices/me/telemetry", mqtt_msg, strlen(mqtt_msg)))
		{
			cp_up_pkt_fwd++;
		}
		else
		{
			cp_up_pkt_up_fail++;
		}
	}

	modemSleep();
}

void modemWakeup()
{
	//pinMode(5, OUTPUT);
	//digitalWrite(5, LOW);
	//modem.sleepEnable(false);
	//modem.sleepEnable(false);
}

void modemSleep()
{
	//digitalWrite(5, HIGH);
	//pinMode(5,INPUT); // Dejo que el DTR pullup lo lleve
	//modem.sleepEnable(true);
}