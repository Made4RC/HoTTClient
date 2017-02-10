/*
 *
 * Graupner HoTT (Hopping Telemetry Transmission) to Jeti EX
 *
 */

#include "HoTTClient.h"

SoftwareSerial mySerial(10, 11); // RX, TX

#ifdef  __HOTT_LOGGING__
void printHex(int num, int precision) {
     char tmp[16];
     char format[128];

     sprintf(format, "%%.%dX", precision);

     sprintf(tmp, format, num);
     Serial.print(tmp);
     Serial.print(" ");
}
#endif /* __HOTT_LOGGING__ */

HoTTClient::HoTTClient() {
	capacity = 0;
	current = 0.0;
	voltage1 = 0.0;
	voltage2 = 0.0;
	mainVoltage = 0.0;
	BECVoltage = 0.0;			
	cellVoltage1 = 0.0;
	cellVoltage2 = 0.0;
	cellVoltage3 = 0.0;
	cellVoltage4 = 0.0;
	cellVoltage5 = 0.0;
	cellVoltage6 = 0.0;
	cellVoltage7 = 0.0;
	cellVoltage8 = 0.0;
	cellVoltage9 = 0.0;
	cellVoltage10 = 0.0;
	cellVoltage11 = 0.0;
	cellVoltage12 = 0.0;
	cellVoltage13 = 0.0;
	cellVoltage14 = 0.0;
	altitude = 0;
	climbRate = 0.0;
	fuelPercentage = 0;
	fuel = 0;
	temperature1 = 0;
	temperature2 = 0;
	ESCTemperature = 0;
	rpm = 0;
	speed = 0;
	distance = 0;
	direction = 0;
	logitudeStartPosition = 0.0;
	latitudeStartPosition = 0.0;
	logitudeCurrentPosition = 0.0;
	latitudeCurrentPosition = 0.0;
}

void HoTTClient::start() {
	// define pin modes for tx, rx:
	pinMode(10, INPUT);
	pinMode(11, OUTPUT);

	mySerial.begin(19200);

	delay(10);
#ifdef  __HOTT_LOGGING__
	if (!mySerial) {
		Serial.println("Error");
	} else {
		Serial.println("begun.");
	}
#endif /* __HOTT_LOGGING__ */

	mySerial.listen();
	if (mySerial.isListening()) {
#ifdef  __HOTT_LOGGING__
   		Serial.println("Sensor port is listening!"); 
#endif /* __HOTT_LOGGING__ */
   	}
   
}

uint8_t* HoTTClient::_sendRequest(uint8_t module_id) {
	// Sensor abfragen
	mySerial.listen();
	while (mySerial.available() >= 1) {
		mySerial.read();
	}

	mySerial.write(HOTT_BINARY_MODE_REQUEST_ID);
	delay(1);
	mySerial.write(module_id);
	mySerial.write(module_id);
	delay(1);
	mySerial.read();
	
	// max. 100ms versuchen, die 45 Byte Telemetriedaten zu empfangen
	uint8_t telemetryData[] = {  
		0x00, 										/*  0 						Startbyte 0x7C */
		0x00,										/*  1						Module ID */  
		0x00,										/*  2						Alarm */ 
		0x00, 										/*  3 						Sensor ID */
		0x00, 0x00,									/*  4,  5					Inverted Value 1 and 2 */ 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,			/*  6,  7,  8,  9, 10, 11	Telemetry data content ...*/
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,			/* 12, 13, 14, 15, 16, 17	... */ 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,			/* 18, 19, 20, 21, 22, 23	... */  
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,			/* 24, 25, 26, 27, 28, 29	... */ 
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,			/* 30, 31, 32, 33, 34, 35	... */  
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,			/* 36, 37, 38, 39, 40, 41	... */ 
		0x00,										/* 42						Version Number */ 
		0x00,										/* 43						End sign 0x7D */ 
		0x00										/* 44						Checksum */ 
	}; 


	unsigned long startMillis = millis();
	unsigned long currentMillis = millis();
	uint8_t byteCount = 0;
	while (byteCount<=44) {
//		if (Serial1.available() >= 1) {
//			uint8_t readByte = Serial1.read();
		if (mySerial.available() >= 1) {
			uint8_t readByte = mySerial.read();
//			printHex(readByte,2);
			telemetryData[byteCount] = readByte;
			byteCount++;
			
			startMillis = millis();
		}
		currentMillis = millis();
		if (currentMillis - startMillis >= 100) {
#ifdef  __HOTT_LOGGING__
			Serial.println("Response timeout");
#endif /* __HOTT_LOGGING__ */
			// keine Antwort in der erlaubten Zeit
			return 0;
		}		
	}
#ifdef  __HOTT_LOGGING__
	Serial.println();

	Serial.print("BYTE: ");
	uint8_t a = 0;
	for (a = 0; a <= 44; a++) {
		if (a < 10 ) {
			Serial.print(" ");
		}
		Serial.print(a);
		Serial.print(" ");
	}
	Serial.println("");
#endif /* __HOTT_LOGGING__ */

	// Checksum berechnen und bestätigen
#ifdef  __HOTT_LOGGING__
	Serial.print("Calc: ");
#endif /* __HOTT_LOGGING__ */
	uint8_t sum = 0;
	for(int i = 0; i < 44; i++){
#ifdef  __HOTT_LOGGING__
		printHex(telemetryData[i],2);
#endif /* __HOTT_LOGGING__ */
		sum += telemetryData[i];
	}
#ifdef  __HOTT_LOGGING__
	printHex(telemetryData[44],2);
	Serial.println("");
#endif /* __HOTT_LOGGING__ */
	
	if ( telemetryData[0] != 0x7C ) {
#ifdef  __HOTT_LOGGING__
		Serial.print("Startbyte falsch: erwartet 0x7c empfangen ");
		printHex(telemetryData[3],0);
		Serial.println(" ");		
#endif /* __HOTT_LOGGING__ */
		return 0;  // FIXME Error codes
	}
	if ( telemetryData[1] != module_id ) {
#ifdef  __HOTT_LOGGING__
		Serial.print("Modul-ID falsch: erwartet ");
		printHex(module_id,2);
		Serial.print(" empfangen ");
		printHex(telemetryData[1],2);
		Serial.println(" ");
#endif /* __HOTT_LOGGING__ */
		return 0; // FIXME Error codes
	}
	if ( telemetryData[43] != 0x7D ) {
#ifdef  __HOTT_LOGGING__
		Serial.print("Endbyte falsch: erwartet 0x7d empfangen ");
		printHex(telemetryData[3],43);
		Serial.println(" ");		
#endif /* __HOTT_LOGGING__ */
		return 0; // FIXME Error codes
	}
	if (telemetryData[44] != sum ) {
#ifdef  __HOTT_LOGGING__
		Serial.print("Checksumme falsch: erwartet ");
		printHex(sum,2);
		Serial.print(" empfangen ");
		printHex(telemetryData[44],2);
		Serial.println(" ");
#endif /* __HOTT_LOGGING__ */
		return 0; // FIXME Error codes
	}
	
	switch (module_id) {
		case HOTT_ELECTRIC_AIR_MODULE_ID:
			if (telemetryData[3] != HOTT_ELECTRIC_AIR_SENSOR_ID) {
#ifdef  __HOTT_LOGGING__
		Serial.print("Sensor-ID falsch: erwartet ");
		printHex(HOTT_ELECTRIC_AIR_SENSOR_ID,2);
		Serial.print(" empfangen ");
		printHex(telemetryData[3],2);
		Serial.println(" ");
#endif /* __HOTT_LOGGING__ */
				return 0;
			}
		break;
		case HOTT_GENERAL_AIR_MODULE_ID:
			if (telemetryData[3] != HOTT_GENERAL_AIR_SENSOR_ID) {
#ifdef  __HOTT_LOGGING__
		Serial.print("Sensor-ID falsch: erwartet ");
		printHex(HOTT_GENERAL_AIR_SENSOR_ID,2);
		Serial.print(" empfangen ");
		printHex(telemetryData[3],2);
		Serial.println(" ");
#endif /* __HOTT_LOGGING__ */
				return 0;
			}
		break;
		case HOTT_GPS_MODULE_ID:
			if (telemetryData[3] != HOTT_GPS_SENSOR_ID) {
#ifdef  __HOTT_LOGGING__
		Serial.print("Sensor-ID falsch: erwartet ");
		printHex(HOTT_GPS_SENSOR_ID,2);
		Serial.print(" empfangen ");
		printHex(telemetryData[3],2);
		Serial.println(" ");
#endif /* __HOTT_LOGGING__ */
				return 0;
			}
		break;
		case HOTT_VARIO_MODULE_ID:
			if (telemetryData[3] != HOTT_VARIO_SENSOR_ID) {
#ifdef  __HOTT_LOGGING__
		Serial.print("Sensor-ID falsch: Erwartet ");
		printHex(HOTT_VARIO_SENSOR_ID,2);
		Serial.print(" empfangen ");
		printHex(telemetryData[3],2);
		Serial.println(" ");
#endif /* __HOTT_LOGGING__ */
				return 0;
			}
		break;
		case HOTT_AIRESC_MODULE_ID:
			if (telemetryData[3] != HOTT_AIRESC_SENSOR_ID) {
#ifdef  __HOTT_LOGGING__
		Serial.print("Sensor-ID falsch: Erwartet ");
		printHex(HOTT_AIRESC_SENSOR_ID,2);
		Serial.print(" empfangen ");
		printHex(telemetryData[3],2);
		Serial.println(" ");
#endif /* __HOTT_LOGGING__ */
				return 0;
			}
		break;
	}

	return telemetryData;
}
bool HoTTClient::_parseResponse(uint8_t telemetryData[]) {
	capacity = 0;
	current = 0.0;
	voltage1 = 0.0;
	voltage2 = 0.0;
	mainVoltage = 0.0;
	BECVoltage = 0.0;			
	cellVoltage1 = 0.0;
	cellVoltage2 = 0.0;
	cellVoltage3 = 0.0;
	cellVoltage4 = 0.0;
	cellVoltage5 = 0.0;
	cellVoltage6 = 0.0;
	cellVoltage7 = 0.0;
	cellVoltage8 = 0.0;
	cellVoltage9 = 0.0;
	cellVoltage10 = 0.0;
	cellVoltage11 = 0.0;
	cellVoltage12 = 0.0;
	cellVoltage13 = 0.0;
	cellVoltage14 = 0.0;
	altitude = 0;
	climbRate = 0.0;
	fuelPercentage = 0;
	fuel = 0;
	temperature1 = 0;
	temperature2 = 0;
	ESCTemperature = 0;
	rpm = 0;
	speed = 0;
	distance = 0;
	direction = 0;
	logitudeStartPosition = 0.0;
	latitudeStartPosition = 0.0;
	logitudeCurrentPosition = 0.0;
	latitudeCurrentPosition = 0.0;

	switch (telemetryData[1]) {
		case HOTT_ELECTRIC_AIR_MODULE_ID:
			capacity = word(telemetryData[33], telemetryData[32]) * 10;
			current = word(telemetryData[29], telemetryData[28]);
			voltage1 = word(telemetryData[21], telemetryData[20]);
			voltage2 = word(telemetryData[23], telemetryData[22]);
			mainVoltage = word(telemetryData[31], telemetryData[30]);
			cellVoltage1 = telemetryData[6] / 100 * 2;
			cellVoltage2 = telemetryData[7] / 100 * 2;
			cellVoltage3 = telemetryData[8] / 100 * 2;
			cellVoltage4 = telemetryData[9] / 100 * 2;
			cellVoltage5 = telemetryData[10] / 100 * 2;
			cellVoltage6 = telemetryData[11] / 100 * 2;
			cellVoltage7 = telemetryData[12] / 100 * 2;
			cellVoltage8 = telemetryData[13] / 100 * 2;
			cellVoltage9 = telemetryData[14] / 100 * 2;
			cellVoltage10 = telemetryData[15] / 100 * 2;
			cellVoltage11 = telemetryData[16] / 100 * 2;
			cellVoltage12 = telemetryData[17] / 100 * 2;
			cellVoltage13 = telemetryData[18] / 100 * 2;
			cellVoltage14 = telemetryData[19] / 100 * 2;
			altitude = word(telemetryData[27], telemetryData[26]) - 500;		
			if (word(telemetryData[35], telemetryData[34]) > 30000) {
				climbRate = (word(telemetryData[35], telemetryData[34])-30000) * 0.01;				
			} else {
				climbRate = (30000-word(telemetryData[35], telemetryData[34])) * 0.01;
			}
			temperature1 = telemetryData[24] - 20;	
			temperature2 = telemetryData[25] - 20;	
			rpm = word(telemetryData[38], telemetryData[38]) * 10;		
		break;
		case HOTT_GENERAL_AIR_MODULE_ID:
			capacity = word(telemetryData[33], telemetryData[32]) * 10;
			current = word(telemetryData[29], telemetryData[28]);
			voltage1 = word(telemetryData[13], telemetryData[12]);
			voltage2 = word(telemetryData[15], telemetryData[14]);
			mainVoltage = word(telemetryData[31], telemetryData[30]);
			cellVoltage1 = telemetryData[6] / 100 * 2;
			cellVoltage2 = telemetryData[7] / 100 * 2;
			cellVoltage3 = telemetryData[8] / 100 * 2;
			cellVoltage4 = telemetryData[9] / 100 * 2;
			cellVoltage5 = telemetryData[10] / 100 * 2;
			cellVoltage6 = telemetryData[11] / 100 * 2;
			altitude = word(telemetryData[24], telemetryData[23]) - 500;		
			if (word(telemetryData[26], telemetryData[25]) > 30000) {
				climbRate = (word(telemetryData[26], telemetryData[25])-30000) * 0.01;				
			} else {
				climbRate = (30000-word(telemetryData[26], telemetryData[25])) * 0.01;
			}
			fuelPercentage = telemetryData[18];
			fuel = word(telemetryData[20], telemetryData[19]);			
			temperature1 = telemetryData[16] - 20;	
			temperature2 = telemetryData[17] - 20;	
			rpm = word(telemetryData[22], telemetryData[21]) * 10;		
		break;
		case HOTT_GPS_MODULE_ID:
			altitude = word(telemetryData[22], telemetryData[21]) - 500;
			if (word(telemetryData[24], telemetryData[23]) > 30000) {
				climbRate = (word(telemetryData[24], telemetryData[23])-30000) * 0.01;				
			} else {
				climbRate = (30000-word(telemetryData[24], telemetryData[23])) * 0.01;
			}
			speed = word(telemetryData[8], telemetryData[7]);
			distance = word(telemetryData[20], telemetryData[17]);
			direction = telemetryData[6]*2;
			logitudeStartPosition = 0.0;
			latitudeStartPosition = 0.0;
			logitudeCurrentPosition = 0.0;
			latitudeCurrentPosition = 0.0;
		break;
		case HOTT_VARIO_MODULE_ID:
			altitude = word(telemetryData[6], telemetryData[5]) - 500;
			if (word(telemetryData[12], telemetryData[11]) > 30000) {
				climbRate = (word(telemetryData[12], telemetryData[11])-30000) * 0.01;				
			} else {
				climbRate = (30000-word(telemetryData[12], telemetryData[11])) * 0.01;
			}
		break;
		case HOTT_AIRESC_MODULE_ID:
			capacity = word(telemetryData[11], telemetryData[10]) * 10;
			current = word(telemetryData[15], telemetryData[14]);
			mainVoltage = word(telemetryData[7], telemetryData[6]);
			BECVoltage = word(telemetryData[26], telemetryData[25]);
			rpm = word(telemetryData[19], telemetryData[18]) * 10;
			ESCTemperature = telemetryData[12] - 20;
		break;
	}

	return true;
}

bool HoTTClient::probe(uint8_t module_id) {
	if (_sendRequest(module_id)==0) {
		return false;		
	} else {
		return true;		
	}
}
bool HoTTClient::poll(uint8_t module_id) {
	uint8_t *telemetryData;
	telemetryData = _sendRequest(module_id);
	if (telemetryData!=0) {
		_parseResponse(telemetryData);
		return true;
	}
	return false;
}
