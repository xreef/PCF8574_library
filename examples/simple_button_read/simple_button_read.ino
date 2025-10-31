/*
 * Name: simple_button_read
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates how to read a digital input (a button) using the PCF8574 library.
 * It reads the state of pin P1 and prints a message to the Serial Monitor when the button is pressed.
 *
 * Connections:
 * PCF8574    ----- Arduino
 * A0, A1, A2 ----- VCC (to set I2C address to 0x27) or GND (to set I2C address to 0x20)
 *                In this example the address is 0x39 (PCF8574A)
 * VSS        ----- GND
 * VDD        ----- 5V or 3.3V
 * SDA        ----- A4 (on Arduino Uno)
 * SCL        ----- A5 (on Arduino Uno)
 *
 * P1         ----- Connect to one leg of a push-button.
 *                Connect the other leg of the button to VCC.
 *                A pull-down resistor (e.g., 10k Ohm) should be connected between P1 and GND.
 * P0         ----- Unconnected (set as output).
 *
 */

#include "Arduino.h"
#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library

// Set I2C address. 0x39 is the address for a PCF8574A with A0 connected to VCC.
PCF8574 pcf8574(0x39);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Set pin P0 to OUTPUT mode (not used in this example)
  pcf8574.pinMode(P0, OUTPUT);
  // Set pin P1 to INPUT mode to read the button state
  pcf8574.pinMode(P1, INPUT);

  Serial.print("Initializing PCF8574...");
  if (pcf8574.begin()){
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }
}

void loop()
{
  // Read the digital value from pin P1
  uint8_t val = pcf8574.digitalRead(P1);

  // If the button is pressed (the input is HIGH), print a message.
  if (val == HIGH) {
    Serial.println("KEY PRESSED");
  }

  delay(50); // Small delay to debounce the button
}
