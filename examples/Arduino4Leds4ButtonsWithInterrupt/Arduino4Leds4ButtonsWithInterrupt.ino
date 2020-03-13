/*
 KeyPressed and leds with interrupt
 by Mischianti Renzo <http://www.mischianti.org>

 https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/
*/

#include "Arduino.h"
#include "PCF8574.h"

// For arduino uno only pin 1 and 2 are interrupted
#define ARDUINO_UNO_INTERRUPTED_PIN 2

// Function interrupt
void keyPressedOnPCF8574();

// Set i2c address
PCF8574 pcf8574(0x38, ARDUINO_UNO_INTERRUPTED_PIN, keyPressedOnPCF8574);
unsigned long timeElapsed;
void setup()
{
	Serial.begin(115200);

	pcf8574.pinMode(P0, INPUT_PULLUP);
	pcf8574.pinMode(P1, INPUT_PULLUP);
	pcf8574.pinMode(P2, INPUT);
	pcf8574.pinMode(P3, INPUT);

	pcf8574.pinMode(P7, OUTPUT);
	pcf8574.pinMode(P6, OUTPUT, HIGH);
	pcf8574.pinMode(P5, OUTPUT);
	pcf8574.pinMode(P4, OUTPUT, LOW);

	pcf8574.begin();

	timeElapsed = millis();
}
unsigned long lastSendTime = 0;        // last send time
unsigned long interval = 4000;          // interval between sends

bool startVal = HIGH;

bool keyPressed = false;
void loop()
{
	if (keyPressed){
		uint8_t val0 = pcf8574.digitalRead(P0);
		uint8_t val1 = pcf8574.digitalRead(P1);
		uint8_t val2 = pcf8574.digitalRead(P2);
		uint8_t val3 = pcf8574.digitalRead(P3);
		Serial.print("P0 ");
		Serial.print(val0);
		Serial.print(" P1 ");
		Serial.print(val1);
		Serial.print(" P2 ");
		Serial.print(val2);
		Serial.print(" P3 ");
		Serial.println(val3);
		keyPressed= false;
	}

	  if (millis() - lastSendTime > interval) {
			pcf8574.digitalWrite(P7, startVal);
			if (startVal==HIGH) {
				startVal = LOW;
			}else{
				startVal = HIGH;
			}
			lastSendTime = millis();

			Serial.print("P7 ");
			Serial.println(startVal);
	  }
}

void keyPressedOnPCF8574(){
	// Interrupt called (No Serial no read no wire in this function, and DEBUG disabled on PCF library)
	 keyPressed = true;

}
