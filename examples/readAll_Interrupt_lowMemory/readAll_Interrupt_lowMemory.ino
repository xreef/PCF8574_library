/*
 KeyPressed with interrupt in LOW_MEMORY mode
 by Mischianti Renzo <http://www.mischianti.org>

 https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/
*/

#include "Arduino.h"
#include "PCF8574.h"

// To use in low memory mode and prevent use of 7byte you must decomment the line
// #define PCF8574_LOW_MEMORY
// in the library

// For arduino uno only pin 1 and 2 are interrupted
#define ARDUINO_UNO_INTERRUPTED_PIN 2

// Function interrupt
void keyChangedOnPCF8574();

// Set i2c address
PCF8574 pcf8574(0x39, ARDUINO_UNO_INTERRUPTED_PIN, keyChangedOnPCF8574);
unsigned long timeElapsed;
void setup()
{
	Serial.begin(115200);
	delay(1000);

	pcf8574.pinMode(P0, INPUT);
	pcf8574.pinMode(P1, INPUT);
	pcf8574.pinMode(P2, INPUT);
	Serial.print("Init pcf8574...");
	if (pcf8574.begin()){
		Serial.println("OK");
	}else{
		Serial.println("KO");
	}

	Serial.println("START");

	timeElapsed = millis();
}

bool keyChanged = false;
void loop()
{
	if (keyChanged){
		byte di = pcf8574.digitalReadAll();
		Serial.print("READ VALUE FROM PCF: ");
		Serial.println(di, BIN);
//		delay(5);
		keyChanged= false;
	}
}

void keyChangedOnPCF8574(){
	// Interrupt called (No Serial no read no wire in this function, and DEBUG disabled on PCF library)
	 keyChanged = true;
}
