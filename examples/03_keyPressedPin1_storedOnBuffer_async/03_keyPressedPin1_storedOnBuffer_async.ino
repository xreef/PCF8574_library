#include "Arduino.h"
#include "PCF8574.h"

// Set i2c address
PCF8574 pcf8574(0x39);
unsigned long timeElapsed;
void setup()
{
	Serial.begin(115200);

	pcf8574.pinMode(P0, OUTPUT);
	pcf8574.pinMode(P1, INPUT);
	pcf8574.begin();


	timeElapsed = millis();
}

void loop()
{
	// Read and store on buffer all input (pinMode) that are going HIGHT
	pcf8574.readBuffer();
	if (millis()>timeElapsed+2000){
		// read value on buffer than reset value for that pin
		uint8_t val = pcf8574.digitalRead(P1);
		if (val==HIGH) Serial.println("KEY PRESSED STORED ON BUFFER, NOW READED AND RESETTED.");
		timeElapsed = millis();
	}
}
