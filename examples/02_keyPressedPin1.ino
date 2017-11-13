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
}

void loop()
{
	uint8_t val = pcf8574.digitalRead(P1);
	if (val==HIGH) Serial.println("KEY PRESSED");
	delay(50);
}
