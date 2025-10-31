/*
 * Name: esp8266_sequential_leds
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example shows how to control a series of LEDs connected to a PCF8574 expander
 * on an ESP8266 (WeMos D1 Mini). The LEDs will turn on and off sequentially.
 *
 * Connections:
 * PCF8574    ----- WeMos D1 Mini
 * A0, A1, A2 ----- GND (to set I2C address to 0x20)
 * VSS        ----- GND
 * VDD        ----- 5V or 3.3V
 * SDA        ----- D2 (GPIO 4) with a 4.7k pull-up resistor
 * SCL        ----- D1 (GPIO 5) with a 4.7k pull-up resistor
 *
 * P0 to P7   ----- Connect to the anode of 8 LEDs.
 *                Connect the cathode of each LED to GND via a resistor (e.g., 220 Ohm).
 *
 */

#include "Arduino.h"
#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library

// Set i2c address (0x20 is the default for PCF8574 with A0, A1, A2 connected to GND)
PCF8574 pcf8574(0x20);

void setup()
{
  Serial.begin(9600);
  delay(1000);

  // Set all pins to OUTPUT
  for(int i=0; i<8; i++) {
    pcf8574.pinMode(i, OUTPUT);
  }

  Serial.print("Initializing PCF8574...");
  if (pcf8574.begin()){
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }
}

void loop()
{
  // This loop iterates through each pin (0 to 7)
  static int pin = 0;

  // Turn the current LED on
  pcf8574.digitalWrite(pin, HIGH);
  delay(1000);

  // Turn the current LED off
  pcf8574.digitalWrite(pin, LOW);
  delay(1000);

  // Move to the next pin
  pin++;
  if (pin > 7) {
    pin = 0; // Reset to the first pin
  }
}
