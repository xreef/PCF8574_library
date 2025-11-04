/*
 * HC-SR04 Ultrasonic Sensor via PCF8574 - Extended Example
 *
 * This example demonstrates various methods for reading HC-SR04 ultrasonic sensor
 * connected to a PCF8574 I2C expander, including NewPing-style methods.
 *
 * Methods available:
 * - pulseIn/pulseInPoll: Low-level pulse measurement (existing methods)
 * - ping/pingPoll: Send trigger and measure echo (returns microseconds)
 * - ping_cm/ping_cm_poll: Get distance in centimeters
 * - ping_in/ping_in_poll: Get distance in inches
 * - ping_median/ping_median_poll: Get median of multiple readings for stability
 *
 * The "poll" variants reduce I2C traffic by polling at intervals instead of
 * continuously reading. This trades some precision for better I2C bus efficiency.
 *
 * IMPORTANT: Reading ultrasonic sensors via I2C expander has limitations:
 * - Lower precision compared to direct GPIO connection
 * - I2C communication adds latency
 * - Use polling methods to reduce I2C traffic
 * - Best for applications not requiring millimeter precision
 *
 * Connections:
 * - PCF8574 TRIG: P1
 * - PCF8574 ECHO: P0
 * - PCF8574 SDA: Arduino SDA
 * - PCF8574 SCL: Arduino SCL
 * - PCF8574 VCC: 5V
 * - PCF8574 GND: GND
 *
 * Author: Renzo Mischianti
 * Website: www.mischianti.org
 */

#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>

// PCF8574 I2C address (change if needed)
PCF8574 pcf(0x20);

// HC-SR04 pins on PCF8574
const uint8_t trigPinPCF = P1; // TRIG on P1
const uint8_t echoPinPCF = P0; // ECHO on P0

// Maximum distance to measure (in cm)
const unsigned long MAX_DISTANCE = 400; // 4 meters

void setup(){
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for serial (or 3s timeout)

  Serial.println(F("\n\n=== HC-SR04 via PCF8574 - Extended Example ==="));

  // Initialize PCF8574
  if (!pcf.begin()){
    Serial.println(F("ERROR: PCF8574 not found!"));
    while(1) delay(100);
  }

  Serial.println(F("PCF8574 initialized successfully"));

  // Configure pins
  pcf.pinMode(trigPinPCF, OUTPUT);
  pcf.pinMode(echoPinPCF, INPUT);

  // Initialize TRIG pin to LOW
  pcf.digitalWrite(trigPinPCF, LOW);

  Serial.println(F("\nConnections:"));
  Serial.println(F("  TRIG -> PCF8574 P1"));
  Serial.println(F("  ECHO -> PCF8574 P0"));
  Serial.println(F("\nMethods demonstrated:"));
  Serial.println(F("  1. Basic pulseIn (high precision, more I2C traffic)"));
  Serial.println(F("  2. pulseInPoll (reduced I2C traffic)"));
  Serial.println(F("  3. ping_cm (simple distance in cm)"));
  Serial.println(F("  4. ping_cm_poll (distance with polling)"));
  Serial.println(F("  5. ping_in (distance in inches)"));
  Serial.println(F("  6. ping_median (stable median reading)"));
  Serial.println(F("\n===========================================\n"));

  delay(2000);
}

void loop(){
  Serial.println(F("--- New Measurement Cycle ---"));

  // Method 1: Using pulseIn (original method - high precision, high I2C traffic)
  Serial.println(F("\n1. Using pulseIn (manual calculation):"));
  pcf.digitalWrite(trigPinPCF, LOW);
  delayMicroseconds(2);
  pcf.digitalWrite(trigPinPCF, HIGH);
  delayMicroseconds(10);
  pcf.digitalWrite(trigPinPCF, LOW);

  unsigned long duration = pcf.pulseIn(echoPinPCF, HIGH, 30000UL);
  unsigned long distance_cm = duration / 29 / 2;

  Serial.print(F("   Duration: "));
  Serial.print(duration);
  Serial.print(F(" us, Distance: "));
  Serial.print(distance_cm);
  Serial.println(F(" cm"));

  delay(100); // Wait between measurements

  // Method 2: Using pulseInPoll (reduced I2C traffic)
  Serial.println(F("\n2. Using pulseInPoll (100us polling):"));
  pcf.digitalWrite(trigPinPCF, LOW);
  delayMicroseconds(2);
  pcf.digitalWrite(trigPinPCF, HIGH);
  delayMicroseconds(10);
  pcf.digitalWrite(trigPinPCF, LOW);

  duration = pcf.pulseInPoll(echoPinPCF, HIGH, 30000UL, 100);
  distance_cm = duration / 29 / 2;

  Serial.print(F("   Duration: "));
  Serial.print(duration);
  Serial.print(F(" us, Distance: "));
  Serial.print(distance_cm);
  Serial.println(F(" cm"));

  delay(100);

  // Method 3: Using ping_cm (NewPing-style, simple and clear)
  Serial.println(F("\n3. Using ping_cm (NewPing-style):"));
  distance_cm = pcf.ping_cm(trigPinPCF, echoPinPCF, MAX_DISTANCE);

  Serial.print(F("   Distance: "));
  Serial.print(distance_cm);
  Serial.println(F(" cm"));

  delay(100);

  // Method 4: Using ping_cm_poll (best for I2C efficiency)
  Serial.println(F("\n4. Using ping_cm_poll (efficient):"));
  distance_cm = pcf.ping_cm_poll(trigPinPCF, echoPinPCF, MAX_DISTANCE, 100);

  Serial.print(F("   Distance: "));
  Serial.print(distance_cm);
  Serial.println(F(" cm"));

  delay(100);

  // Method 5: Using ping_in (distance in inches)
  Serial.println(F("\n5. Using ping_in (inches):"));
  unsigned long distance_in = pcf.ping_in(trigPinPCF, echoPinPCF, MAX_DISTANCE);

  Serial.print(F("   Distance: "));
  Serial.print(distance_in);
  Serial.println(F(" inches"));

  delay(100);

  // Method 6: Using ping_median (most stable, takes 5 samples)
  Serial.println(F("\n6. Using ping_median (5 samples, most stable):"));
  Serial.println(F("   Taking 5 measurements... please wait"));

  unsigned long startTime = millis();
  distance_cm = pcf.ping_median(trigPinPCF, echoPinPCF, 5, MAX_DISTANCE);
  unsigned long elapsed = millis() - startTime;

  Serial.print(F("   Median distance: "));
  Serial.print(distance_cm);
  Serial.print(F(" cm (took "));
  Serial.print(elapsed);
  Serial.println(F(" ms)"));

  // Show a simple distance bar
  Serial.println(F("\nDistance visualization:"));
  Serial.print(F("   ["));
  int bars = distance_cm / 10; // 1 bar = 10 cm
  if (bars > 40) bars = 40; // Limit to 40 bars
  for (int i = 0; i < bars; i++){
    Serial.print(F("="));
  }
  Serial.println(F("]"));

  Serial.println(F("\n--- End of Cycle ---\n"));

  // Wait before next cycle
  delay(2000);
}
