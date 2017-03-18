/*
 *
 * HoTT - Hopping Telemetry Transmission - Client
 *
 * Reciever
 * The reciever, that requests a sensor value from a server (sensor), acts as client.
 *
 *
 * Arduino Micro 5v/16mHz w/ Atmega 328
 *
 * Userful links:
 * http://www.rc-network.de/forum/showthread.php/281496-Graupner-HoTT-Telemetrie-Sensoren-Eigenbau-DIY-Telemetrie-Protokoll-entschlüsselt?p=3369807&viewfull=1#post3369807
 * https://github.com/3yc/hott-for-ardupilot/blob/master/PX4/hott-px4-code/px4-hott-module/hott_msgs.h
 * https://github.com/obayer/MultiWii-HoTT/blob/master/MultiWii_2_1/HoTTv4.ino
 *
 * How to connect the arduino with your reciever
 *
 *         TX ----------
 *                     |
 * Arduino              1k5 Ohm                              Sensor
 *                     |
 *         RX ----------------------- Telemetry signal
 *
 */

#ifndef HoTTClient_h
#define HoTTClient_h HoTTClient_h

#include <SoftwareSerial.h>
#include "Arduino.h"


// #define __HOTT_LOGGING__


// Protocoll definitions

// Graupner #33620 Electric Air Module (EAM)
#define HOTT_ELECTRIC_AIR_MODULE_ID					0x8E
#define HOTT_ELECTRIC_AIR_SENSOR_ID					0xE0
// Graupner #33611 General Air Module (GAM)
#define HOTT_GENERAL_AIR_MODULE_ID					0x8D
#define HOTT_GENERAL_AIR_SENSOR_ID					0xD0 
// Graupner #33600 GPS module
#define HOTT_GPS_MODULE_ID							0x8A
#define HOTT_GPS_SENSOR_ID							0xA0
// Graupner #33601 Vario Module
#define HOTT_VARIO_MODULE_ID						0x89
#define HOTT_VARIO_SENSOR_ID						0x90
//Graupner #337xx Air ESC 
#define HOTT_AIRESC_MODULE_ID						0x8c
#define HOTT_AIRESC_SENSOR_ID						0xc0 


// inverted characters
#define HOTT_PRIMARY_INVERTED						0
#define HOTT_SECONDARY_INVERTED						1

// ###########################################################################

// Sent, when reciever request telemetry data
#define HOTT_BINARY_MODE_REQUEST_ID					0x80
// or text (configuration) data
#define HOTT_TEXT_MODE_REQUEST_ID					0x7F 
 

// Radio keys
#define HOTT_KEY_LEFT								7
#define HOTT_KEY_SET								9
#define HOTT_KEY_DOWN								11
#define HOTT_KEY_UP									13
#define HOTT_KEY_RIGHT								14
#define HOTT_TEXT_MODE_IDLE             			0x0F 


class HoTTClient {

private:
	uint8_t* _sendRequest(uint8_t module_id);
	bool _parseResponse(uint8_t telemetryData[]);

public:
	uint16_t capacity;				// capacity from 0 to 65536 [mA], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID, HOTT_AIRESC_MODULE_ID
	float current;					// current from 0.0 to 999.9 [A], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID, HOTT_AIRESC_MODULE_ID
	float voltage1;					// [V], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID
	float voltage2;					// [V], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID
	float mainVoltage;				// [V], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID, HOTT_AIRESC_MODULE_ID
	float BECVoltage;				// [V], HOTT_AIRESC_MODULE_ID
	float cellVoltage1;				// [V], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage2;				// [V], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage3;				// [V], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage4;				// [V], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage5;				// [V], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage6;				// [V], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage7;				// [V], valid with HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage8;				// [V], valid with HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage9;				// [V], valid with HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage10;			// [V], valid with HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage11;			// [V], valid with HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage12;			// [V], valid with HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage13;			// [V], valid with HOTT_ELECTRIC_AIR_MODULE_ID
	float cellVoltage14;			// [V], valid with HOTT_ELECTRIC_AIR_MODULE_ID
	int16_t altitude;				// [m], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID, HOTT_VARIO_MODULE_ID, HOTT_GPS_MODULE_ID
	float climbRate;				// [m/s], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID, HOTT_VARIO_MODULE_ID, HOTT_GPS_MODULE_ID
	uint8_t fuelPercentage;			// [1], valid with HOTT_GENERAL_AIR_MODULE
	uint16_t fuel;					// [ml], valid with HOTT_GENERAL_AIR_MODULE
	int8_t temperature1;			// [°C], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID
	int8_t temperature2;			// [°C], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID
	int8_t ESCTemperature;			// [°C], HOTT_AIRESC_MODULE_ID
	uint16_t rpm;					// [1/min], valid with HOTT_GENERAL_AIR_MODULE, HOTT_ELECTRIC_AIR_MODULE_ID, HOTT_AIRESC_MODULE_ID
	uint16_t speed;					// [m/s], valid with HOTT_GPS_MODULE_ID
	uint16_t distance;				// [m], valid with HOTT_GPS_MODULE_ID
	uint8_t direction;				// [°], valid with HOTT_GPS_MODULE_ID
	float logitudeCurrentPosition;	// valid with HOTT_GPS_MODULE_ID
	float latitudeCurrentPosition;	// valid with HOTT_GPS_MODULE_ID
	char message[22];				
	
	HoTTClient();
	
	void start();
	bool probe(uint8_t module_id);
	bool poll(uint8_t module_id);
};
#endif /* HoTTClient_h */
