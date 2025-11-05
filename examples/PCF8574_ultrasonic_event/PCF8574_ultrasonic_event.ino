/*
 * PCF8574 Ultrasonic Event-Based Example
 *
 * This example demonstrates event-based distance measurement with threshold detection.
 * It triggers actions when objects enter/leave specific distance zones.
 * Useful for proximity detection, parking sensors, obstacle avoidance, etc.
 *
 * Events:
 * - Object detected (< threshold)
 * - Object in warning zone (< warning threshold)
 * - Object cleared (> threshold)
 *
 * Hardware Connections:
 * - PCF8574 P1 -> HC-SR04 TRIG
 * - PCF8574 P0 -> HC-SR04 ECHO
 * - PCF8574 P7 -> LED (optional, for visual feedback)
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
#define TRIGGER_PIN P1  // Trigger pin
#define ECHO_PIN    P0  // Echo pin
#define LED_PIN     P7  // Optional LED for visual feedback

// Distance thresholds (in centimeters)
#define DANGER_THRESHOLD  20  // Very close - danger zone
#define WARNING_THRESHOLD 50  // Warning zone
#define CLEAR_THRESHOLD   100 // Clear - no object detected

// Maximum distance
#define MAX_DISTANCE 200

// Create PCF8574 object
PCF8574 pcf(PCF8574_ADDRESS);

// State tracking
enum State {
  STATE_CLEAR,
  STATE_WARNING,
  STATE_DANGER
};

State currentState = STATE_CLEAR;
unsigned long lastChangeTime = 0;

void setup() {
  Serial.begin(115200);

  // Wait for serial port to connect
  while (!Serial && millis() < 3000);

  Serial.println(F("\n=== PCF8574 Ultrasonic Event-Based Example ==="));
  Serial.println(F("Proximity detection with threshold zones"));

  // Configure pins
  pcf.pinMode(TRIGGER_PIN, OUTPUT);
  pcf.pinMode(ECHO_PIN, INPUT);
  pcf.pinMode(LED_PIN, OUTPUT);

  // Initialize pins
  pcf.digitalWrite(TRIGGER_PIN, LOW);
  pcf.digitalWrite(LED_PIN, LOW);

  // Initialize PCF8574 after pins are configured
  if (!pcf.begin()) {
    Serial.println(F("ERROR: Could not initialize PCF8574! Check wiring, I2C address, SDA/SCL connections and power."));
    while (1) delay(100);
  }

  Serial.println(F("PCF8574 initialized successfully"));

  Serial.println(F("\nThresholds:"));
  Serial.print(F("  DANGER: < "));
  Serial.print(DANGER_THRESHOLD);
  Serial.println(F(" cm"));
  Serial.print(F("  WARNING: < "));
  Serial.print(WARNING_THRESHOLD);
  Serial.println(F(" cm"));
  Serial.print(F("  CLEAR: > "));
  Serial.print(CLEAR_THRESHOLD);
  Serial.println(F(" cm"));
  Serial.println(F("\nMonitoring for objects...\n"));

  delay(1000);
}

void loop() {
  delay(100); // Check every 100ms

  // Get distance with median filtering for stability
  unsigned int distance = pcf.ping_median_poll(TRIGGER_PIN, ECHO_PIN, 3, MAX_DISTANCE, 100);

  // Determine new state based on distance
  State newState = currentState;

  if (distance > 0) { // Valid reading
    if (distance < DANGER_THRESHOLD) {
      newState = STATE_DANGER;
    } else if (distance < WARNING_THRESHOLD) {
      newState = STATE_WARNING;
    } else if (distance > CLEAR_THRESHOLD) {
      newState = STATE_CLEAR;
    }
    // If between WARNING and CLEAR, maintain current state (hysteresis)
  } else {
    // No valid reading - assume clear
    newState = STATE_CLEAR;
  }

  // Check for state change
  if (newState != currentState) {
    currentState = newState;
    lastChangeTime = millis();

    // Handle state change events
    switch (currentState) {
      case STATE_DANGER:
        Serial.println(F("\n*** DANGER! Object too close! ***"));
        Serial.print(F("Distance: "));
        Serial.print(distance);
        Serial.println(F(" cm"));
        pcf.digitalWrite(LED_PIN, HIGH); // Turn on LED
        break;

      case STATE_WARNING:
        Serial.println(F("\n** WARNING: Object detected **"));
        Serial.print(F("Distance: "));
        Serial.print(distance);
        Serial.println(F(" cm"));
        // Blink LED
        for (int i = 0; i < 3; i++) {
          pcf.digitalWrite(LED_PIN, HIGH);
          delay(100);
          pcf.digitalWrite(LED_PIN, LOW);
          delay(100);
        }
        break;

      case STATE_CLEAR:
        Serial.println(F("\n* CLEAR: Path is clear *"));
        pcf.digitalWrite(LED_PIN, LOW); // Turn off LED
        break;
    }
  }

  // Periodic status update (every 2 seconds)
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 2000) {
    lastStatusTime = millis();

    Serial.print(F("Status: "));
    switch (currentState) {
      case STATE_DANGER:  Serial.print(F("DANGER ")); break;
      case STATE_WARNING: Serial.print(F("WARNING")); break;
      case STATE_CLEAR:   Serial.print(F("CLEAR  ")); break;
    }
    Serial.print(F(" | Distance: "));
    if (distance > 0) {
      Serial.print(distance);
      Serial.print(F(" cm"));
    } else {
      Serial.print(F("OUT OF RANGE"));
    }
    Serial.print(F(" | State duration: "));
    Serial.print((millis() - lastChangeTime) / 1000);
    Serial.println(F(" s"));
  }
}