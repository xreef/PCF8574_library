/*
 * Name: arduino_uno_interrupt_4leds_4buttons
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates how to use interrupts to read 4 buttons and control 4 LEDs
 * simultaneously on an Arduino Uno.
 *
 * Connections:
 * PCF8574    ----- Arduino Uno
 * A0, A1, A2 ----- VCC (to set I2C address to 0x27) or GND (to set I2C address to 0x20)
 *                In this example the address is 0x38 (PCF8574A)
 * VSS        ----- GND
 * VDD        ----- 5V
 * SDA        ----- A4
 * SCL        ----- A5
 * INT        ----- D2 (interrupt pin)
 *
 * P0-P3      ----- Connect to buttons.
 * P4-P7      ----- Connect to LEDs.
 *
 */

#include "Arduino.h"
#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library

// Define the interrupt pin on the Arduino Uno
#define ARDUINO_UNO_INTERRUPTED_PIN 2

// Forward declaration of the interrupt service routine
void keyPressedOnPCF8574();

// Initialize the PCF8574 library with the I2C address, interrupt pin, and ISR
PCF8574 pcf8574(0x38, ARDUINO_UNO_INTERRUPTED_PIN, keyPressedOnPCF8574);

void setup()
{
  Serial.begin(115200);

  // Set pin modes for the buttons
  pcf8574.pinMode(P0, INPUT_PULLUP);
  pcf8574.pinMode(P1, INPUT_PULLUP);
  pcf8574.pinMode(P2, INPUT);
  pcf8574.pinMode(P3, INPUT);

  // Set pin modes for the LEDs
  pcf8574.pinMode(P7, OUTPUT);
  pcf8574.pinMode(P6, OUTPUT, HIGH);
  pcf8574.pinMode(P5, OUTPUT);
  pcf8574.pinMode(P4, OUTPUT, LOW);

  Serial.print("Initializing PCF8574...");
  if (pcf8574.begin()){
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }
}

unsigned long lastSendTime = 0;
unsigned long interval = 4000;

bool startVal = HIGH;
bool keyPressed = false;

void loop()
{
  // If the interrupt flag is set, read the button states
  if (keyPressed){
    uint8_t val0 = pcf8574.digitalRead(P0);
    uint8_t val1 = pcf8574.digitalRead(P1);
    uint8_t val2 = pcf8574.digitalRead(P2);
    uint8_t val3 = pcf8574.digitalRead(P3);

    Serial.print("P0: "); Serial.print(val0);
    Serial.print(" P1: "); Serial.print(val1);
    Serial.print(" P2: "); Serial.print(val2);
    Serial.print(" P3: "); Serial.println(val3);

    keyPressed = false;
  }

  // Every `interval` milliseconds, toggle one of the LEDs
  if (millis() - lastSendTime > interval) {
    pcf8574.digitalWrite(P7, startVal);
    if (startVal == HIGH) {
      startVal = LOW;
    } else {
      startVal = HIGH;
    }
    lastSendTime = millis();

    Serial.print("P7: ");
    Serial.println(startVal);
  }
}

// Interrupt Service Routine (ISR)
void keyPressedOnPCF8574(){
  // Set the flag to indicate that a key has been pressed
  keyPressed = true;
}
