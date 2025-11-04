/*
 * PCF8574 Multiple Ultrasonic Sensors Example
 *
 * This example shows how to use multiple HC-SR04 sensors with PCF8574.
 * Each sensor uses 2 pins (TRIG and ECHO), so 8 pins can handle up to 4 sensors.
 * This example demonstrates 3 sensors.
 *
 * Based on NewPing library's multiple sensors example, adapted for PCF8574.
 *
 * IMPORTANT NOTES:
 * - Each sensor needs time between readings (~60ms cycle)
 * - With I2C latency, sequential reading is slower
 * - Use polling methods for better efficiency
 * - Consider using one PCF8574 per sensor for better performance
 *
 * Hardware Connections:
 * Sensor 1:
 *   - PCF8574 P0 -> HC-SR04 #1 TRIG
 *   - PCF8574 P1 -> HC-SR04 #1 ECHO
 * Sensor 2:
 *   - PCF8574 P2 -> HC-SR04 #2 TRIG
 *   - PCF8574 P3 -> HC-SR04 #2 ECHO
 * Sensor 3:
 *   - PCF8574 P4 -> HC-SR04 #3 TRIG
 *   - PCF8574 P5 -> HC-SR04 #3 ECHO
 *
 * Common:
 *   - PCF8574 SDA -> Arduino SDA
 *   - PCF8574 SCL -> Arduino SCL
 *   - All VCC -> 5V, All GND -> GND
 *
 * Author: Renzo Mischianti
 * Website: www.mischianti.org
 */

#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>

// PCF8574 I2C address
#define PCF8574_ADDRESS 0x20

// Number of sensors
#define SONAR_NUM 3

// Maximum distance (in cm) to measure
#define MAX_DISTANCE 200

// Sensor pins on PCF8574 (trigger pin, echo pin)
const uint8_t sensorPins[SONAR_NUM][2] = {
  {P0, P1},  // Sensor 1: TRIG on P0, ECHO on P1
  {P2, P3},  // Sensor 2: TRIG on P2, ECHO on P3
  {P4, P5}   // Sensor 3: TRIG on P4, ECHO on P5
};

// Create PCF8574 object
PCF8574 pcf(PCF8574_ADDRESS);

void setup() {
  Serial.begin(115200);

  // Wait for serial port to connect
  while (!Serial && millis() < 3000);

  Serial.println(F("\n=== PCF8574 Multiple Ultrasonic Sensors Example ==="));
  Serial.print(F("Managing "));
  Serial.print(SONAR_NUM);
  Serial.println(F(" HC-SR04 sensors via PCF8574"));

  // Configure pins for all sensors
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    pcf.pinMode(sensorPins[i][0], OUTPUT); // TRIG pin
    pcf.pinMode(sensorPins[i][1], INPUT);  // ECHO pin
    pcf.digitalWrite(sensorPins[i][0], LOW); // Initialize TRIG to LOW

    Serial.print(F("  Sensor "));
    Serial.print(i + 1);
    Serial.print(F(": TRIG=P"));
    Serial.print(sensorPins[i][0]);
    Serial.print(F(", ECHO=P"));
    Serial.println(sensorPins[i][1]);
  }

  // Initialize PCF8574 after pins are configured
  if (!pcf.begin()) {
    Serial.println(F("ERROR: Could not initialize PCF8574! Check wiring, I2C address, SDA/SCL connections and power."));
    while (1) delay(100);
  }

  Serial.println(F("PCF8574 initialized successfully"));

  Serial.println(F("\nStarting measurements...\n"));
  delay(1000);
}

void loop() {
  // Read each sensor sequentially
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    delay(33); // Wait 33ms between pings (about 29ms needed minimum)

    // Get distance using polling method for efficiency
    unsigned int distance = pcf.ping_cm_poll(
      sensorPins[i][0],  // TRIG pin
      sensorPins[i][1],  // ECHO pin
      MAX_DISTANCE,      // Max distance
      100                // Polling interval
    );

    Serial.print(F("Sensor "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(distance);
    Serial.print(F(" cm"));

    // Add visual indicator
    if (distance == 0) {
      Serial.println(F(" [OUT OF RANGE]"));
    } else if (distance < 10) {
      Serial.println(F(" [VERY CLOSE!]"));
    } else if (distance < 50) {
      Serial.println(F(" [CLOSE]"));
    } else {
      Serial.println();
    }
  }

  Serial.println(); // Empty line between cycles
  delay(200); // Wait before next cycle
}
