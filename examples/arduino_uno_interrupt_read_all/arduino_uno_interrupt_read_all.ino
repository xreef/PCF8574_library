/*
 * Name: arduino_uno_interrupt_read_all
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates how to use interrupts with the PCF8574 library on an Arduino Uno.
 * It reads the state of all input pins when a change is detected on any of them.
 * The PCF8574's INT pin is connected to an interrupt-capable pin on the Arduino Uno.
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
 * P1, P2     ----- Connect to buttons or other input devices.
 * P0         ----- Unconnected (set as output).
 *
 */

#include "Arduino.h"
#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library

// Define the interrupt pin on the Arduino Uno (only pins 2 and 3 are interrupt-capable)
#define ARDUINO_UNO_INTERRUPTED_PIN 2

// Forward declaration of the interrupt service routine
void keyChangedOnPCF8574();

// Initialize the PCF8574 library with the I2C address, interrupt pin, and ISR
PCF8574 pcf8574(0x39, ARDUINO_UNO_INTERRUPTED_PIN, keyChangedOnPCF8574);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Set pin modes for the PCF8574
  pcf8574.pinMode(P0, OUTPUT);
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
  // If the interrupt flag is set
  if (keyChanged){
    // Read all digital inputs from the PCF8574
    PCF8574::DigitalInput di = pcf8574.digitalReadAll();

    // Print the values of the input pins
    Serial.print("READ VALUE FROM PCF: ");
    Serial.print("P0: "); Serial.print(di.p0);
    Serial.print(" - P1: "); Serial.print(di.p1);
    Serial.print(" - P2: "); Serial.print(di.p2);
    Serial.print(" - P3: "); Serial.println(di.p3);

    // Reset the flag
    keyChanged = false;
  }
}

// Interrupt Service Routine (ISR)
void keyChangedOnPCF8574(){
  // This function is called when the interrupt is triggered.
  // It sets a flag to indicate that a key has been pressed.
  // Avoid using Serial or other long-running code inside an ISR.
  keyChanged = true;
}
