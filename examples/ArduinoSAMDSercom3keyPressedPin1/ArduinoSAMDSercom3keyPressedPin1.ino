/*
 KeyPressed on PIN1
 by Mischianti Renzo <http://www.mischianti.org>

 https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/
*/

#include "Arduino.h"
#include "PCF8574.h"

TwoWire aWire(&sercom3, 20, 21);
// Set i2c address
PCF8574 pcf8574(&aWire, 0x38);

unsigned long timeElapsed;
void setup()
{
	Serial.begin(115200);
	delay(1000);

	pcf8574.pinMode(P0, OUTPUT);
	pcf8574.pinMode(P1, INPUT);

	Serial.print("Init pcf8574...");
	if (pcf8574.begin()){
		Serial.println("OK");
	}else{
		Serial.println("KO");
	}
}

void loop()
{
	uint8_t val = pcf8574.digitalRead(P1);
	if (val==HIGH) Serial.println("KEY PRESSED");
	delay(50);
}
