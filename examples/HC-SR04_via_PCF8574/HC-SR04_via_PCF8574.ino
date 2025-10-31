#include <Wire.h>
#include <PCF8574.h>

// PCF8574 address (change if necessary)
PCF8574 pcf(0x39);

const int trigPin = 9; // Arduino pin for TRIG
const uint8_t echoPinPCF = P0; // PCF8574 pin connected to ECHO

void setup(){
  Serial.begin(115200);
  pcf.begin();
  pcf.pinMode(echoPinPCF, INPUT);
  pinMode(trigPin, OUTPUT);
}

// Measure using pulseIn (forces immediate read; higher resolution but many I2C requests)
unsigned long measureDistancePulseIn(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pcf.pulseIn(echoPinPCF, HIGH, 30000UL); // 30ms timeout
  unsigned long distanceCm = duration / 29 / 2;
  return distanceCm;
}

// Measure using pulseInPoll (fewer I2C reads; use pollIntervalMicros to tune the tradeoff)
unsigned long measureDistancePulseInPoll(unsigned int pollIntervalMicros = 50){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pcf.pulseInPoll(echoPinPCF, HIGH, 30000UL, pollIntervalMicros);
  unsigned long distanceCm = duration / 29 / 2;
  return distanceCm;
}

void loop(){
  unsigned long d1 = measureDistancePulseIn();
  Serial.print("Distance pulseIn: "); Serial.print(d1); Serial.println(" cm");

  unsigned long d2 = measureDistancePulseInPoll(100); // polling 100us
  Serial.print("Distance pulseInPoll(100us): "); Serial.print(d2); Serial.println(" cm");

  delay(500);
}
