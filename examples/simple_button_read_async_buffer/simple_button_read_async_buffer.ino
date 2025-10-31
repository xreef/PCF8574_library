/*
 * Name: simple_button_read_async_buffer
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates the asynchronous reading of a button using the PCF8574 library's buffer.
 * The `readBuffer()` function is called frequently to update the internal buffer with the state of the input pins.
 * The button state is then read from the buffer at a different interval.
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

unsigned long timeElapsed;

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

  timeElapsed = millis();
}

void loop()
{
  // Continuously read the input pins and store their state in the internal buffer.
  // This allows for asynchronous reading of the inputs.
  pcf8574.readBuffer();

  // Every 2 seconds, check the state of the button from the buffer.
  if (millis() > timeElapsed + 2000){
    // Read the value of pin P1 from the buffer.
    // This read operation also resets the buffered value for that pin.
    uint8_t val = pcf8574.digitalRead(P1);

    // If the button was pressed, print a message.
    if (val == HIGH) {
      Serial.println("KEY PRESSED STORED ON BUFFER, NOW READ AND RESET.");
    }

    timeElapsed = millis();
  }
}
