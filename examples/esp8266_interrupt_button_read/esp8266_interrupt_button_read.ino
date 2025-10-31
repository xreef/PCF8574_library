/*
 * Name: esp8266_interrupt_button_read
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates how to use interrupts with the PCF8574 library on an ESP8266.
 * It reads the state of 8 buttons connected to the PCF8574 when any of them are pressed.
 * The PCF8574's INT pin is connected to an interrupt-capable pin on the ESP8266.
 *
 * Connections:
 * PCF8574    ----- WeMos D1 Mini
 * A0, A1, A2 ----- GND (to set I2C address to 0x20)
 * VSS        ----- GND
 * VDD        ----- 5V or 3.3V
 * SDA        ----- D2 (GPIO 4) with a 4.7k pull-up resistor
 * SCL        ----- D1 (GPIO 5) with a 4.7k pull-up resistor
 * INT        ----- D7 (GPIO 13) with a 10k pull-up resistor
 *
 * P0 to P7   ----- Connect to one leg of 8 push-buttons.
 *                Connect the other leg of each button to VCC.
 *                A pull-down resistor (e.g., 10k Ohm) should be connected between each pin and GND.
 *
 */

#include "Arduino.h"
#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library

// Define the interrupt pin on the ESP8266
#define ESP8266_INTERRUPTED_PIN 13

// Set I2C address
PCF8574 pcf8574(0x20);

// Volatile flag to be set by the ISR
bool keyPressed = false;

// Interrupt Service Routine (ISR)
void ICACHE_RAM_ATTR keyPressedOnPCF8574(){
  // This function is called when the interrupt is triggered.
  // It sets a flag to indicate that a key has been pressed.
  // Avoid using Serial or other long-running code inside an ISR.
  keyPressed = true;
}

void setup()
{
  Serial.begin(9600);
  delay(1000);

  // Set the interrupt pin as an input with a pull-up resistor
  pinMode(ESP8266_INTERRUPTED_PIN, INPUT_PULLUP);
  // Attach the interrupt to the pin, calling the ISR on a FALLING edge
  attachInterrupt(digitalPinToInterrupt(ESP8266_INTERRUPTED_PIN), keyPressedOnPCF8574, FALLING);

  // Set all PCF8574 pins to INPUT mode
  for(int i=0; i<8; i++) {
    pcf8574.pinMode(i, INPUT);
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
  // If the interrupt flag is set
  if (keyPressed){
    // Read all digital inputs from the PCF8574
    PCF8574::DigitalInput val = pcf8574.digitalReadAll();

    // Check the state of each button and print a message if pressed
    if (val.p0 == HIGH) Serial.println("KEY0 PRESSED");
    if (val.p1 == HIGH) Serial.println("KEY1 PRESSED");
    if (val.p2 == HIGH) Serial.println("KEY2 PRESSED");
    if (val.p3 == HIGH) Serial.println("KEY3 PRESSED");
    if (val.p4 == HIGH) Serial.println("KEY4 PRESSED");
    if (val.p5 == HIGH) Serial.println("KEY5 PRESSED");
    if (val.p6 == HIGH) Serial.println("KEY6 PRESSED
*/