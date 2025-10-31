/*
 * Name: esp8266_interrupt_read_write_all
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates how to use interrupts to read button presses and simultaneously
 * write to multiple digital outputs on an ESP8266.
 *
 * Connections:
 * PCF8574    ----- ESP8266
 * A0, A1, A2 ----- VCC (to set I2C address to 0x27) or GND (to set I2C address to 0x20)
 *                In this example the address is 0x38 (PCF8574A)
 * VSS        ----- GND
 * VDD        ----- 5V or 3.3V
 * SDA        ----- D2 (GPIO 4) with a 4.7k pull-up resistor
 * SCL        ----- D1 (GPIO 5) with a 4.7k pull-up resistor
 * INT        ----- D3 (GPIO 0) with a 10k pull-up resistor
 *
 * P0-P3      ----- Connect to buttons or other input devices.
 * P4-P7      ----- Connect to LEDs or other output devices.
 *
 */

#include "Arduino.h"
#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library

// Define the interrupt pin on the ESP8266
#define ESP8266_INTERRUPTED_PIN D3

// Forward declaration of the interrupt service routine
void ICACHE_RAM_ATTR keyPressedOnPCF8574();

// Initialize the PCF8574 library with the I2C address, interrupt pin, and ISR
PCF8574 pcf8574(0x38, ESP8266_INTERRUPTED_PIN, keyPressedOnPCF8574);

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("INIT");

  // Set pin modes for the PCF8574
  pcf8574.pinMode(P0, INPUT);
  pcf8574.pinMode(P1, INPUT_PULLUP);
  pcf8574.pinMode(P2, INPUT);
  pcf8574.pinMode(P3, INPUT);

  pcf8574.pinMode(P7, OUTPUT);
  pcf8574.pinMode(P6, OUTPUT, HIGH);
  pcf8574.pinMode(P5, OUTPUT, LOW);
  pcf8574.pinMode(P4, OUTPUT, LOW);

  Serial.print("Initializing PCF8574...");
  if (pcf8574.begin()){
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }

  Serial.println("START");
}

unsigned long lastSendTime = 0;
unsigned long interval = 3000;

bool startVal = HIGH;
bool keyPressed = false;

void loop()
{
  // If the interrupt flag is set, read the input pins
  if (keyPressed){
    uint8_t val0 = pcf8574.digitalRead(P0);
    uint8_t val1 = pcf8574.digitalRead(P1);
    uint8_t val2 = pcf8574.digitalRead(P2);
    uint8_t val3 = pcf8574.digitalRead(P3);

    Serial.print("P0: "); Serial.print(val0);
    Serial.print(" P1: "); Serial.println(val1);
    Serial.print("P2: "); Serial.print(val2);
    Serial.print(" P3: "); Serial.println(val3);

    keyPressed = false;
  }

  // Every `interval` milliseconds, write to the output pins
  if (millis() - lastSendTime > interval) {
    Serial.print("WRITE ALL VALUE FROM P4 TO P7: ");
    Serial.println(startVal);

    bool startVal2 = LOW;
    if (startVal == HIGH) {
      startVal = LOW;
      startVal2 = HIGH;
    } else {
      startVal = HIGH;
      startVal2 = LOW;
    }

    // Create a DigitalInput struct to hold the new output values
    PCF8574::DigitalInput digitalInput;
    digitalInput.p4 = startVal2;
    digitalInput.p5 = startVal;
    digitalInput.p6 = startVal2;
    digitalInput.p7 = startVal;

    // Write all the values to the output pins at once
    pcf8574.digitalWriteAll(digitalInput);

    lastSendTime = millis();
  }
}

// Interrupt Service Routine (ISR)
void keyPressedOnPCF8574(){
  // Set the flag to indicate that a key has been pressed
  keyPressed = true;
}
