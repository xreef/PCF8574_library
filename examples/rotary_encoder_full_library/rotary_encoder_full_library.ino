/*
 * Name: rotary_encoder_full_library
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates how to use a rotary encoder with the PCF8574 library.
 * It uses the library's built-in encoder functions for simplicity and efficiency.
 * An interrupt is used to detect encoder and button events.
 *
 * Connections:
 * PCF8574    ----- WeMos D1 Mini
 * A0, A1, A2 ----- VCC (to set I2C address to 0x27) or GND (to set I2C address to 0x20)
 *                In this example the address is 0x38 (PCF8574A)
 * VSS        ----- GND
 * VDD        ----- 5V or 3.3V
 * SDA        ----- D1 (GPIO 5) with a 4.7k pull-up resistor
 * SCL        ----- D2 (GPIO 4) with a 4.7k pull-up resistor
 * INT        ----- D7 (GPIO 13) with a 10k pull-up resistor
 *
 * P0 (CLK)   ----- Rotary Encoder CLK pin
 * P1 (DT)    ----- Rotary Encoder DT pin
 * P2 (SW)    ----- Rotary Encoder SW (button) pin
 *
 */
#include "Arduino.h"
#include "PCF8574.h"

// Define the pins connected to the rotary encoder
int encoderPinA = P0;
int encoderPinB = P1;

// Define the interrupt pin on the microcontroller
#define INTERRUPTED_PIN D7

// Forward declaration of the interrupt service routine
void ICACHE_RAM_ATTR updateEncoder();

// Initialize the PCF8574 library with the I2C address, interrupt pin, and ISR
PCF8574 pcf8574(0x38, INTERRUPTED_PIN, updateEncoder);

// Volatile variables to store the encoder value and button state
volatile long encoderValue = 0;
uint8_t encoderButtonVal = HIGH;

void setup()
{
  Serial.begin (9600);
  delay(500);

  // Initialize the encoder pins using the library's built-in function
  pcf8574.encoder(encoderPinA, encoderPinB);
  // Set the encoder button pin as an input
  pcf8574.pinMode(P2, INPUT);

  // Initialize the PCF8574
  Serial.print("Initializing PCF8574...");
  if (pcf8574.begin()){
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }
}

// Flag to indicate that the encoder or button state has changed
bool changed = false;

void loop()
{
  // If a change has been detected by the ISR
  if (changed){
    // Print the new encoder value and button state
    Serial.print("ENCODER --> ");
    Serial.print(encoderValue);
    Serial.print(" - BUTTON --> ");
    Serial.println(encoderButtonVal ? "HIGH" : "LOW");
    // Reset the flag
    changed = false;
  }
}

// Previous state of the encoder button
bool valPrecEncoderButton = LOW;

// Interrupt Service Routine (ISR)
void updateEncoder(){
  // Read the encoder value using the library's helper function
  // This function returns true if the value has changed
  if (pcf8574.readEncoderValue(encoderPinA, encoderPinB, &encoderValue)) {
    changed = true;
  }

  // Read the state of the encoder button
  encoderButtonVal = pcf8574.digitalRead(P2);
  // If the button state has changed, set the flag
  if (encoderButtonVal != valPrecEncoderButton){
    changed = true;
    valPrecEncoderButton = encoderButtonVal;
  }
}
