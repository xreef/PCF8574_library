/*
 * Name: arduino_samd_sercom_button_read
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates how to use a specific SERCOM interface for I2C communication
 * on an Arduino SAMD board (e.g., Arduino Nano 33 IoT) to read a button state.
 *
 * Connections:
 * PCF8574    ----- Arduino SAMD
 * A0, A1, A2 ----- VCC (to set I2C address to 0x27) or GND (to set I2C address to 0x20)
 *                In this example the address is 0x38 (PCF8574A)
 * VSS        ----- GND
 * VDD        ----- 3.3V
 * SDA        ----- Pin 20 (SERCOM3)
 * SCL        ----- Pin 21 (SERCOM3)
 *
 * P1         ----- Connect to one leg of a push-button.
 *                Connect the other leg of the button to VCC.
 *                A pull-down resistor (e.g., 10k Ohm) should be connected between P1 and GND.
 * P0         ----- Unconnected (set as output).
 *
 */

#include "Arduino.h"
#include "PCF8574.h"

// Initialize a TwoWire instance for SERCOM3 on pins 20 (SDA) and 21 (SCL)
TwoWire aWire(&sercom3, 20, 21);

// Initialize the PCF8574 library with the custom TwoWire instance and I2C address
PCF8574 pcf8574(&aWire, 0x38);

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

void loop()
{
  // Read the digital value from pin P1
  uint8_t val = pcf8574.digitalRead(P1);

  // If the button is pressed (the input is HIGH), print a message
  if (val == HIGH) {
    Serial.println("KEY PRESSED");
  }

  delay(50); // Small delay to debounce the button
}
