/*
 * Name: esp32_second_i2c_sequential_leds
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates how to use a second I2C interface on an ESP32 to control a PCF8574.
 * It blinks a series of LEDs connected to the expander.
 *
 * Connections:
 * PCF8574    ----- ESP32
 * A0, A1, A2 ----- GND (to set I2C address to 0x20)
 * VSS        ----- GND
 * VDD        ----- 5V or 3.3V
 * SDA        ----- GPIO 21 (for the second I2C interface)
 * SCL        ----- GPIO 22 (for the second I2C interface)
 *
 * P0 to P7   ----- Connect to the anode of 8 LEDs.
 *                Connect the cathode of each LED to GND via a resistor (e.g., 220 Ohm).
 *
 */

#include "Arduino.h"
#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library

// Instantiate a TwoWire object for the first I2C interface (not used in this example)
TwoWire I2Cone = TwoWire(0);
// Instantiate a TwoWire object for the second I2C interface
TwoWire I2Ctwo = TwoWire(1);

// Initialize the PCF8574 library with the second I2C interface and the I2C address
PCF8574 pcf8574(&I2Ctwo, 0x20);

void setup()
{
  Serial.begin(115200);

  // Initialize the second I2C interface with custom SDA and SCL pins and a frequency of 400kHz
  I2Ctwo.begin(21, 22, 400000U);
  delay(1000);

  // Set all PCF8574 pins to OUTPUT mode
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
  delay(400);

  // Turn the current LED off
  pcf8574.digitalWrite(pin, LOW);
  delay(400);

  // Move to the next pin
  pin++;
  if (pin > 7) {
    pin = 0; // Reset to the first pin
  }
}
