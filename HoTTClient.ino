/* 
 
**************************************************************/
#include <SoftwareSerial.h>
#include "HoTTClient.h"

HoTTClient sensor;
uint8_t sensorType = 0;

void setup() {
	Serial.begin(19200);
	Serial.println("HoTT-Test");
	Serial.println("Debug connection established.");

#ifdef  __HOTT_LOGGING__
	Serial.println( "Logging activated." );
#endif /* __HOTT_LOGGING__ */

    Serial.println(F("\nSend any character to begin: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
	delay(10);
	
	sensor.start();
	
	// Try to find a resonding sensor
	while (sensorType == 0) {
		Serial.println("Probing...");
		
		Serial.print("EAM ...");
		if (sensor.probe(HOTT_ELECTRIC_AIR_MODULE_ID)) {
			Serial.println("HoTT EAM found.");
			sensorType = HOTT_ELECTRIC_AIR_MODULE_ID;
		}
		Serial.print("GAM ...");
		if (sensor.probe(HOTT_GENERAL_AIR_MODULE_ID)) {
			Serial.println("HoTT GAM found.");
			sensorType = HOTT_GENERAL_AIR_MODULE_ID;
		}
		Serial.print("GPS ...");
		if (sensor.probe(HOTT_GPS_MODULE_ID)) {
			Serial.println("HoTT GPS found.");
			sensorType = HOTT_GPS_MODULE_ID;
		}
		Serial.print("Vario ...");
		if (sensor.probe(HOTT_VARIO_MODULE_ID)) {
			Serial.println("HoTT Vario found.");
			sensorType = HOTT_VARIO_MODULE_ID;
		}
		Serial.print("AirESC ...");
		if (sensor.probe(HOTT_AIRESC_MODULE_ID)) {
			Serial.println("HoTT ESC found.");
			sensorType = HOTT_AIRESC_MODULE_ID;
		}
		delay(1000);
	}
}

void loop() {
	Serial.println("Polling...");

	if (sensor.poll(sensorType)) {
		switch (sensorType) {
			case HOTT_ELECTRIC_AIR_MODULE_ID:
				break;
			case HOTT_GENERAL_AIR_MODULE_ID:
				break;
			case HOTT_GPS_MODULE_ID:
				break;
			case HOTT_VARIO_MODULE_ID:
				Serial.print("Höhe: ");
				Serial.print(sensor.altitude);
				Serial.print("m, Steigrate: ");
				Serial.print(sensor.climbRate);
				Serial.println("m/s");
				break;
			case HOTT_AIRESC_MODULE_ID:
				Serial.print("Spannung: ");
				Serial.print(sensor.mainVoltage);
				Serial.print("V, Strom: ");
				Serial.print(sensor.current);
				Serial.print("A, Kapazität: ");
				Serial.print(sensor.capacity);
				Serial.print("mAh, Drehzahl: ");
				Serial.print(sensor.rpm);
				Serial.print(", Temperatur: ");
				Serial.print(sensor.ESCTemperature);
				Serial.println("°C");
				break;
		}
	} else {
		Serial.println("No valid response");
	}

	delay(1000);
}
