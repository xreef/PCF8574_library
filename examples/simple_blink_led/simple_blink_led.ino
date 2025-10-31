/*
 * Name: simple_blink_led
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates the basic usage of the PCF8574 library to blink an LED.
 * It sets pin P0 as an output and toggles its state every second.
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
 * P0         ----- Connect to the anode of an LED.
 *                Connect the cathode of the LED to GND via a resistor (e.g., 220 Ohm).
 * P1         ----- Unconnected (set as input).
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

  // Set pin P0 to OUTPUT mode to control the LED
  pcf8574.pinMode(P0, OUTPUT);
  // Set pin P1 to INPUT mode (not used in this example)
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
  // Turn the LED on (set the pin HIGH)
  pcf8574.digitalWrite(P0, HIGH);
  delay(1000); // Wait for a second

  // Turn the LED off (set the pin LOW)
  pcf8574.digitalWrite(P0, LOW);
  delay(1000); // Wait for a second
}
