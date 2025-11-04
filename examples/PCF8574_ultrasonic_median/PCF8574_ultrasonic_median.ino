/*
 * PCF8574 Ultrasonic Median Example
 *
 * This example demonstrates stable distance measurement using median filtering.
 * Taking the median of multiple readings filters out erratic readings
 * and provides more stable results.
 *
 * Based on NewPing library's median example, adapted for PCF8574.
 *
 * IMPORTANT NOTES:
 * - Median filtering takes longer (multiple samples needed)
 * - Results are more stable and reliable
 * - Use polling methods to reduce I2C traffic
 * - Best for applications requiring stable readings
 *
 * Hardware Connections:
 * - PCF8574 P1 -> HC-SR04 TRIG
 * - PCF8574 P0 -> HC-SR04 ECHO
 * - PCF8574 SDA -> Arduino SDA
 * - PCF8574 SCL -> Arduino SCL
 * - PCF8574 & HC-SR04 VCC -> 5V
 * - PCF8574 & HC-SR04 GND -> GND
 *
 * Author: Renzo Mischianti
 * Website: www.mischianti.org
 */

#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>

// PCF8574 I2C address
#define PCF8574_ADDRESS 0x20

// HC-SR04 pins on PCF8574
#define TRIGGER_PIN P1  // PCF8574 pin connected to trigger pin on the ultrasonic sensor
#define ECHO_PIN    P0  // PCF8574 pin connected to echo pin on the ultrasonic sensor

// Maximum distance we want to measure (in centimeters)
#define MAX_DISTANCE 200

// Number of iterations for median calculation
#define ITERATIONS 5

// Create PCF8574 object
PCF8574 pcf(PCF8574_ADDRESS);

void setup() {
  Serial.begin(115200);

  // Wait for serial port to connect
  while (!Serial && millis() < 3000);

  Serial.println(F("\n=== PCF8574 Ultrasonic Median Example ==="));
  Serial.println(F("Using median filtering for stable readings"));

  // Configure pins
  pcf.pinMode(TRIGGER_PIN, OUTPUT);
  pcf.pinMode(ECHO_PIN, INPUT);

  // Initialize trigger pin to LOW
  pcf.digitalWrite(TRIGGER_PIN, LOW);

  // Initialize PCF8574 after pins are configured
  if (!pcf.begin()) {
    Serial.println(F("ERROR: Could not initialize PCF8574! Check wiring, I2C address, SDA/SCL connections and power."));
    while (1) delay(100);
  }

  Serial.println(F("PCF8574 initialized successfully"));

  Serial.println(F("\nConfiguration:"));
  Serial.print(F("  Trigger Pin: P"));
  Serial.println(TRIGGER_PIN);
  Serial.print(F("  Echo Pin: P"));
  Serial.println(ECHO_PIN);
  Serial.print(F("  Max Distance: "));
  Serial.print(MAX_DISTANCE);
  Serial.println(F(" cm"));
  Serial.print(F("  Median Iterations: "));
  Serial.println(ITERATIONS);
  Serial.println(F("\nStarting measurements...\n"));

  delay(1000);
}

void loop() {
  delay(50); // Wait between measurements

  unsigned long startTime = millis();

  // Get median distance from multiple samples (more stable than single reading)
  // Using polling variant for better I2C efficiency
  unsigned int distance = pcf.ping_median_poll(TRIGGER_PIN, ECHO_PIN, ITERATIONS, MAX_DISTANCE, 100);

  unsigned long measurementTime = millis() - startTime;

  Serial.print(F("Ping: "));
  Serial.print(distance);
  Serial.print(F(" cm"));
  Serial.print(F(" (median of "));
  Serial.print(ITERATIONS);
  Serial.print(F(" samples, took "));
  Serial.print(measurementTime);
  Serial.println(F(" ms)"));
}
