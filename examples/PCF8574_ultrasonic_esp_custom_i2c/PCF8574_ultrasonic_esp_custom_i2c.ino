/*
 * PCF8574 Ultrasonic with Custom I2C (ESP32/ESP8266)
 *
 * This example demonstrates using HC-SR04 with PCF8574 on ESP32/ESP8266
 * with custom I2C pins (SDA/SCL).
 *
 * ESP32/ESP8266 allow flexible I2C pin assignment, useful when default
 * pins are already in use or not conveniently located.
 *
 * Based on NewPing library example, adapted for PCF8574 with custom I2C.
 *
 * Hardware Connections (ESP32 example):
 * - PCF8574 SDA -> GPIO 21 (or custom pin)
 * - PCF8574 SCL -> GPIO 22 (or custom pin)
 * - PCF8574 P1 -> HC-SR04 TRIG
 * - PCF8574 P0 -> HC-SR04 ECHO
 * - PCF8574 & HC-SR04 VCC -> 5V (or 3.3V)
 * - PCF8574 & HC-SR04 GND -> GND
 *
 * Hardware Connections (ESP8266 example):
 * - PCF8574 SDA -> GPIO 4 (D2) (or custom pin)
 * - PCF8574 SCL -> GPIO 5 (D1) (or custom pin)
 * - Rest same as above
 *
 * NOTE: When using 3.3V logic, use a level shifter or ensure HC-SR04
 * is 3.3V compatible.
 *
 * Author: Renzo Mischianti
 * Website: www.mischianti.org
 */

#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>

// Uncomment your board type
#define ESP32_BOARD
// #define ESP8266_BOARD

// PCF8574 I2C address
#define PCF8574_ADDRESS 0x20

// HC-SR04 pins on PCF8574
#define TRIGGER_PIN P1
#define ECHO_PIN    P0

// Maximum distance
#define MAX_DISTANCE 200

// Custom I2C pins
#ifdef ESP32_BOARD
  #define CUSTOM_SDA 21  // ESP32 default is 21, change if needed
  #define CUSTOM_SCL 22  // ESP32 default is 22, change if needed
#endif

#ifdef ESP8266_BOARD
  #define CUSTOM_SDA 4   // ESP8266 D2
  #define CUSTOM_SCL 5   // ESP8266 D1
#endif

// Create PCF8574 object with custom I2C pins
#if defined(ESP32_BOARD) || defined(ESP8266_BOARD)
  PCF8574 pcf(PCF8574_ADDRESS, CUSTOM_SDA, CUSTOM_SCL);
#else
  PCF8574 pcf(PCF8574_ADDRESS);
#endif

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n========================================"));
  Serial.println(F("PCF8574 Ultrasonic with Custom I2C"));

  #ifdef ESP32_BOARD
    Serial.println(F("Board: ESP32"));
  #endif
  #ifdef ESP8266_BOARD
    Serial.println(F("Board: ESP8266"));
  #endif

  Serial.println(F("========================================\n"));

  // Configure pins
  pcf.pinMode(TRIGGER_PIN, OUTPUT);
  pcf.pinMode(ECHO_PIN, INPUT);
  pcf.digitalWrite(TRIGGER_PIN, LOW);

  // Initialize PCF8574 (I2C pins already set in constructor)
  if (!pcf.begin()) {
    Serial.println(F("ERROR: Could not initialize PCF8574! Check wiring, I2C address, SDA/SCL connections and power."));
    while (1) delay(100);
  }

  Serial.println(F("PCF8574 initialized successfully\n"));

  Serial.println(F("Configuration:"));
  Serial.print(F("  I2C SDA: GPIO "));
  Serial.println(CUSTOM_SDA);
  Serial.print(F("  I2C SCL: GPIO "));
  Serial.println(CUSTOM_SCL);
  Serial.print(F("  PCF8574 Address: 0x"));
  Serial.println(PCF8574_ADDRESS, HEX);
  Serial.print(F("  Trigger Pin: P"));
  Serial.println(TRIGGER_PIN);
  Serial.print(F("  Echo Pin: P"));
  Serial.println(ECHO_PIN);
  Serial.print(F("  Max Distance: "));
  Serial.print(MAX_DISTANCE);
  Serial.println(F(" cm\n"));

  Serial.println(F("Starting measurements...\n"));
  delay(1000);
}

void loop() {
  // Measure distance using polling method (recommended)
  unsigned int distance = pcf.ping_cm_poll(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE, 100);

  // Display result
  Serial.print(F("Distance: "));
  if (distance > 0) {
    Serial.print(distance);
    Serial.print(F(" cm"));

    // Visual bar graph
    Serial.print(F(" ["));
    int bars = distance / 10;
    for (int i = 0; i < bars && i < 20; i++) {
      Serial.print(F("="));
    }
    Serial.println(F("]"));
  } else {
    Serial.println(F("Out of range or no echo"));
  }

  delay(500); // Measure twice per second
}
