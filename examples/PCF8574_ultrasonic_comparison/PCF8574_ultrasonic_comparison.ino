/*
 * PCF8574 Ultrasonic Methods Comparison
 *
 * This example compares different measurement methods available in the PCF8574 library:
 * 1. pulseIn - High precision, high I2C traffic
 * 2. pulseInPoll - Reduced I2C traffic with polling
 * 3. ping_cm - Simple distance in cm
 * 4. ping_cm_poll - Distance with polling (recommended)
 * 5. ping_median - Most stable with median filtering
 * 6. ping_median_poll - Stable + efficient
 *
 * This helps you understand the trade-offs and choose the best method
 * for your application.
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
#define TRIGGER_PIN P1
#define ECHO_PIN    P0

// Maximum distance
#define MAX_DISTANCE 200

// Create PCF8574 object
PCF8574 pcf(PCF8574_ADDRESS);

void setup() {
  Serial.begin(115200);

  while (!Serial && millis() < 3000);

  Serial.println(F("\n========================================"));
  Serial.println(F("PCF8574 Ultrasonic Methods Comparison"));
  Serial.println(F("========================================\n"));

  // Configure pins
  pcf.pinMode(TRIGGER_PIN, OUTPUT);
  pcf.pinMode(ECHO_PIN, INPUT);
  pcf.digitalWrite(TRIGGER_PIN, LOW);

  // Initialize PCF8574 after pins are configured
  if (!pcf.begin()) {
    Serial.println(F("ERROR: Could not initialize PCF8574! Check wiring, I2C address, SDA/SCL connections and power."));
    while (1) delay(100);
  }

  Serial.println(F("PCF8574 initialized successfully\n"));

  Serial.println(F("Methods to be compared:"));
  Serial.println(F("  1. pulseIn          - Manual calculation, high precision"));
  Serial.println(F("  2. pulseInPoll      - Polling 100us, reduced I2C"));
  Serial.println(F("  3. ping_cm          - Simple distance measurement"));
  Serial.println(F("  4. ping_cm_poll     - Distance with 100us polling"));
  Serial.println(F("  5. ping_median      - Median of 5 samples"));
  Serial.println(F("  6. ping_median_poll - Median with polling\n"));

  delay(2000);
}

void loop() {
  Serial.println(F("\n--- New Measurement Cycle ---\n"));

  unsigned long duration, distance;
  unsigned long startTime, elapsedTime;

  // Method 1: pulseIn (manual calculation)
  Serial.println(F("1. Using pulseIn (high precision, more I2C):"));
  startTime = millis();
  pcf.digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  pcf.digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  pcf.digitalWrite(TRIGGER_PIN, LOW);
  duration = pcf.pulseIn(ECHO_PIN, HIGH, 30000UL);
  distance = duration / 29 / 2;
  elapsedTime = millis() - startTime;

  Serial.print(F("   Result: "));
  Serial.print(distance);
  Serial.print(F(" cm (took "));
  Serial.print(elapsedTime);
  Serial.println(F(" ms)"));
  delay(100);

  // Method 2: pulseInPoll with 100us polling
  Serial.println(F("\n2. Using pulseInPoll (100us polling):"));
  startTime = millis();
  pcf.digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  pcf.digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  pcf.digitalWrite(TRIGGER_PIN, LOW);
  duration = pcf.pulseInPoll(ECHO_PIN, HIGH, 30000UL, 100);
  distance = duration / 29 / 2;
  elapsedTime = millis() - startTime;

  Serial.print(F("   Result: "));
  Serial.print(distance);
  Serial.print(F(" cm (took "));
  Serial.print(elapsedTime);
  Serial.println(F(" ms)"));
  delay(100);

  // Method 3: ping_cm (simple and clear)
  Serial.println(F("\n3. Using ping_cm (simple):"));
  startTime = millis();
  distance = pcf.ping_cm(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
  elapsedTime = millis() - startTime;

  Serial.print(F("   Result: "));
  Serial.print(distance);
  Serial.print(F(" cm (took "));
  Serial.print(elapsedTime);
  Serial.println(F(" ms)"));
  delay(100);

  // Method 4: ping_cm_poll (recommended for most cases)
  Serial.println(F("\n4. Using ping_cm_poll (RECOMMENDED):"));
  startTime = millis();
  distance = pcf.ping_cm_poll(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE, 100);
  elapsedTime = millis() - startTime;

  Serial.print(F("   Result: "));
  Serial.print(distance);
  Serial.print(F(" cm (took "));
  Serial.print(elapsedTime);
  Serial.println(F(" ms)"));
  delay(100);

  // Method 5: ping_median (most stable)
  Serial.println(F("\n5. Using ping_median (5 samples, most stable):"));
  startTime = millis();
  distance = pcf.ping_median(TRIGGER_PIN, ECHO_PIN, 5, MAX_DISTANCE);
  elapsedTime = millis() - startTime;

  Serial.print(F("   Result: "));
  Serial.print(distance);
  Serial.print(F(" cm (took "));
  Serial.print(elapsedTime);
  Serial.println(F(" ms)"));
  delay(100);

  // Method 6: ping_median_poll (stable + efficient)
  Serial.println(F("\n6. Using ping_median_poll (BEST for stability):"));
  startTime = millis();
  distance = pcf.ping_median_poll(TRIGGER_PIN, ECHO_PIN, 5, MAX_DISTANCE, 100);
  elapsedTime = millis() - startTime;

  Serial.print(F("   Result: "));
  Serial.print(distance);
  Serial.print(F(" cm (took "));
  Serial.print(elapsedTime);
  Serial.println(F(" ms)"));

  // Summary
  Serial.println(F("\n--- Summary ---"));
  Serial.println(F("  pulseIn/ping_cm:      Fast but more I2C traffic"));
  Serial.println(F("  pulseInPoll/ping_cm_poll: Good balance (recommended)"));
  Serial.println(F("  ping_median*:         Slowest but most stable"));

  Serial.println(F("\n--- End of Cycle ---"));

  // Wait before next cycle
  delay(3000);
}
