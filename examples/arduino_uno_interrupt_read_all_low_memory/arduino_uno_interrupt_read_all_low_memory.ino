/*
 * Name: arduino_uno_interrupt_read_all_low_memory
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates how to use the low memory mode of the PCF8574 library
 * with interrupts on an Arduino Uno.
 *
 * To enable low memory mode, you must uncomment the following line in the PCF8574.h file:
 * #define PCF8574_LOW_MEMORY
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
 * P0-P2      ----- Connect to buttons or other input devices.
 *
 */

#include "Arduino.h"
#include "PCF8574.h"

// To use in low memory mode, you must uncomment the line:
// #define PCF8574_LOW_MEMORY
// in the PCF8574.h file of the library.

// Define the interrupt pin on the Arduino Uno
#define ARDUINO_UNO_INTERRUPTED_PIN 2

// Forward declaration of the interrupt service routine
void keyChangedOnPCF8574();

// Initialize the PCF8574 library with the I2C address, interrupt pin, and ISR
PCF8574 pcf8574(0x39, ARDUINO_UNO_INTERRUPTED_PIN, keyChangedOnPCF8574);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Set pin modes for the input pins
  pcf8574.pinMode(P0, INPUT);
  pcf8574.pinMode(P1, INPUT);
  pcf8574.pinMode(P2, INPUT);

  Serial.print("Initializing PCF8574...");
  if (pcf8574.begin()){
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }

  Serial.println("START");
}

// Volatile flag to be set by the ISR
bool keyChanged = false;

void loop()
{
  // If the interrupt flag is set, read the input states
  if (keyChanged){
    // In low memory mode, digitalReadAll() returns a byte with the state of all pins
    byte di = pcf8574.digitalReadAll();

    Serial.print("READ VALUE FROM PCF: ");
    Serial.println(di, BIN);

    keyChanged = false;
  }
}

// Interrupt Service Routine (ISR)
void keyChangedOnPCF8574(){
  // Set the flag to indicate that a key has been pressed
  keyChanged = true;
}
