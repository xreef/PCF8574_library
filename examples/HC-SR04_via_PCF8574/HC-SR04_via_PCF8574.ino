/*
 * HC-SR04 Ultrasonic Sensor via PCF8574 - Basic Example
 *
 * This example demonstrates how to use an HC-SR04 ultrasonic sensor
 * connected to a PCF8574 I2C expander.
 *
 * Both TRIG and ECHO pins are connected to the PCF8574.
 *
 * Two methods are shown:
 * - pulseIn: High precision but generates more I2C requests
 * - pulseInPoll: Uses polling to reduce I2C traffic (recommended)
 *
 * IMPORTANT: Reading ultrasonic sensors via I2C has limitations
 * compared to direct GPIO connection due to I2C latency.
 * Use polling methods for better efficiency.
 *
 * Connections:
 * - PCF8574 TRIG: P1
 * - PCF8574 ECHO: P0
 *
 * Author: Renzo Mischianti
 * Website: www.mischianti.org
 */

#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>

// PCF8574 address (change if necessary)
PCF8574 pcf(0x20);

// Both pins are on PCF8574
const uint8_t trigPinPCF = P1; // PCF8574 pin for TRIG
const uint8_t echoPinPCF = P0; // PCF8574 pin for ECHO

void setup(){
  Serial.begin(115200);

  // Configure PCF8574 pins
  pcf.pinMode(trigPinPCF, OUTPUT);
  pcf.pinMode(echoPinPCF, INPUT);

  // Initialize TRIG pin to LOW
  pcf.digitalWrite(trigPinPCF, LOW);

  // Initialize PCF8574 after pins are configured
  if (!pcf.begin()){
    Serial.println(F("ERROR: Could not initialize PCF8574! Check wiring, I2C address, SDA/SCL connections and power."));
    while(1) delay(100);
  }

  Serial.println(F("Ultrasonic sensor fully controlled by PCF8574"));
  Serial.println(F("TRIG: P1, ECHO: P0"));
}

// Measure using pulseIn (high resolution but many I2C requests)
unsigned long measureDistancePulseIn(){
  // Generate trigger pulse using PCF8574
  pcf.digitalWrite(trigPinPCF, LOW);
  delayMicroseconds(2);
  pcf.digitalWrite(trigPinPCF, HIGH);
  delayMicroseconds(10);
  pcf.digitalWrite(trigPinPCF, LOW);

  // Measure ECHO pulse duration
  unsigned long duration = pcf.pulseIn(echoPinPCF, HIGH, 30000UL); // 30ms timeout
  unsigned long distanceCm = duration / 29 / 2;
  return distanceCm;
}

// Measure using pulseInPoll (fewer I2C reads; use pollIntervalMicros to balance)
unsigned long measureDistancePulseInPoll(unsigned int pollIntervalMicros = 100){
  // Generate trigger pulse using PCF8574
  pcf.digitalWrite(trigPinPCF, LOW);
  delayMicroseconds(2);
  pcf.digitalWrite(trigPinPCF, HIGH);
  delayMicroseconds(10);
  pcf.digitalWrite(trigPinPCF, LOW);

  // Measure ECHO pulse duration with polling
  unsigned long duration = pcf.pulseInPoll(echoPinPCF, HIGH, 30000UL, pollIntervalMicros);
  unsigned long distanceCm = duration / 29 / 2;
  return distanceCm;
}

void loop(){
  unsigned long d1 = measureDistancePulseIn();
  Serial.print("Distance pulseIn: ");
  Serial.print(d1);
  Serial.println(" cm");

  unsigned long d2 = measureDistancePulseInPoll(100); // poll every 100us
  Serial.print("Distance pulseInPoll(100us): ");
  Serial.print(d2);
  Serial.println(" cm");

  delay(2000);
}
