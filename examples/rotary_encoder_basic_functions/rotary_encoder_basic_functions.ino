/*
 * Name: rotary_encoder_basic_functions
 *
 * Author: Renzo Mischianti
 * Website: https://www.mischianti.org
 *
 * This example demonstrates how to use a rotary encoder with the PCF8574 library
 * using basic digitalRead functions. This approach is more verbose but gives you
 * more control over the logic.
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

  // Set the encoder pins as inputs with pull-up resistors
  pcf8574.pinMode(encoderPinA, INPUT_PULLUP);
  pcf8574.pinMode(encoderPinB, INPUT_PULLUP);
  // Set the encoder button pin as an input with a pull-up resistor
  pcf8574.pinMode(P2, INPUT_PULLUP);

  // Set low latency for faster response, which is important for reading encoders
  pcf8574.setLatency(0);

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

// Variables to store the previous state of the encoder and button
uint8_t encoderPinALast = LOW;
uint8_t valPrecEncoderButton = LOW;

// Interrupt Service Routine (ISR)
void updateEncoder(){
  // Read the current state of the encoder's CLK pin
  uint8_t n = pcf8574.digitalRead(encoderPinA);

  // If the CLK pin has changed from LOW to HIGH, a rotation has occurred
  if ((encoderPinALast == LOW) && (n == HIGH)) {
    // Read the state of the DT pin to determine the direction of rotation
    if (pcf8574.digitalRead(encoderPinB) == LOW) {
      encoderValue--; // Counter-clockwise
    } else {
      encoderValue++; // Clockwise
    }
    changed = true; // Set the flag to indicate a change
  }
  encoderPinALast = n; // Store the current state of the CLK pin

  // Read the state of the encoder button
  encoderButtonVal = pcf8574.digitalRead(P2);
  // If the button state has changed, set the flag
  if (encoderButtonVal != valPrecEncoderButton){
    changed = true;
    valPrecEncoderButton = encoderButtonVal;
  }
}
