#include "Arduino.h"
#include "PCF8574.h"

// For arduino uno only pin 1 and 2 are interrupted
#define ARDUINO_UNO_INTERRUPTED_PIN 2

// Function interrupt
void keyPressedOnPCF8574();

// Set i2c address
PCF8574 pcf8574(0x39, ARDUINO_UNO_INTERRUPTED_PIN, keyPressedOnPCF8574);
unsigned long timeElapsed;
void setup()
{
	Serial.begin(115200);

	pcf8574.pinMode(P0, OUTPUT);
	pcf8574.pinMode(P1, INPUT);
	pcf8574.begin();


	timeElapsed = millis();
}

bool keyPressed = false;
void loop()
{
	if (keyPressed){
		uint8_t val = pcf8574.digitalRead(P1);
		Serial.print("READ VALUE FROM PCF ");
		Serial.println(val);
		keyPressed= false;
	}
}

void keyPressedOnPCF8574(){
	// Interrupt called (No Serial no read no wire in this function, and DEBUG disabled on PCF library)
	 keyPressed = true;

}
