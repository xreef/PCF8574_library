/*
 * Name: arduino_uno_interrupt_button_read
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates how to use an interrupt to read a button press on an Arduino Uno.
 * The PCF8574's INT pin is connected to an interrupt-capable pin on the Arduino.
 *
 * Connections:
 * PCF8574    ----- Arduino Uno
 * A0, A1, A2 ----- VCC (to set I2C address to 0x27) or GND (to set I2C address to 0x20)
 *                In this example the address is 0x39 (PCF8574A)
 * VSS        ----- GND
 * VDD        ----- 5V
 * SDA        ----- A4
 * SCL        ----- A5
 * INT        ----- D2 (interrupt pin)
 *
 * P1         ----- Connect to a button.
 * P0         ----- Unconnected (set as output).
 *
 */

#include "Arduino.h"
#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library

// Define the interrupt pin on the Arduino Uno (only pins 2 and 3 are interrupt-capable)
#define ARDUINO_UNO_INTERRUPTED_PIN 2

// Forward declaration of the interrupt service routine
void keyPressedOnPCF8574();

// Initialize the PCF8574 library with the I2C address, interrupt pin, and ISR
PCF8574 pcf8574(0x39, ARDUINO_UNO_INTERRUPTED_PIN, keyPressedOnPCF8574);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Set pin modes for the PCF8574
  pcf8574.pinMode(P0, OUTPUT);
  pcf8574.pinMode(P1, INPUT);

  Serial.print("Initializing PCF8574...");
  if (pcf8574.begin()){
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }
}

// Volatile flag to be set by the ISR
bool keyPressed = false;

void loop()
{
  // If the interrupt flag is set, read the button state
  if (keyPressed){
    uint8_t val = pcf8574.digitalRead(P1);
    Serial.print("READ VALUE FROM PCF: ");
    Serial.println(val);
    keyPressed = false;
  }
}

// Interrupt Service Routine (ISR)
void keyPressedOnPCF8574(){
  // Set the flag to indicate that a key has been pressed
  keyPressed = true;
}
