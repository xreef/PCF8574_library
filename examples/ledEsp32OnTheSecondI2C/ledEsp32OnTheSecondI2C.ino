/*
 * 	PCF8574 GPIO Port Expand
 *  Blink all led
 *  by Mischianti Renzo <http://www.mischianti.org>
 *
 *  https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/
 *
 *
 * PCF8574    ----- Esp32
 * A0         ----- GRD
 * A1         ----- GRD
 * A2         ----- GRD
 * VSS        ----- GRD
 * VDD        ----- 5V/3.3V
 * SDA        ----- 21
 * SCL        ----- 22
 *
 * P0     ----------------- LED0
 * P1     ----------------- LED1
 * P2     ----------------- LED2
 * P3     ----------------- LED3
 * P4     ----------------- LED4
 * P5     ----------------- LED5
 * P6     ----------------- LED6
 * P7     ----------------- LED7
 *
 */

#include "Arduino.h"
#include "PCF8574.h"  // https://github.com/xreef/PCF8574_library

// Instantiate Wire for generic use at 400kHz
TwoWire I2Cone = TwoWire(0);
// Instantiate Wire for generic use at 100kHz
TwoWire I2Ctwo = TwoWire(1);

// Set i2c address
PCF8574 pcf8574(&I2Ctwo, 0x20);
// PCF8574 pcf8574(&I2Ctwo, 0x20, 21, 22);
// PCF8574(TwoWire *pWire, uint8_t address, uint8_t interruptPin,  void (*interruptFunction)() );
// PCF8574(TwoWire *pWire, uint8_t address, uint8_t sda, uint8_t scl, uint8_t interruptPin,  void (*interruptFunction)());

void setup()
{
  Serial.begin(112560);

  I2Cone.begin(16,17,400000U); // SDA pin 16, SCL pin 17, 400kHz frequency
  delay(1000);

  // Set pinMode to OUTPUT
  for(int i=0;i<8;i++) {
    pcf8574.pinMode(i, OUTPUT);
  }

  Serial.print("Init pcf8574...");
  if (pcf8574.begin()){
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }
}

void loop()
{
  static int pin = 0;
  pcf8574.digitalWrite(pin, HIGH);
  delay(400);
  pcf8574.digitalWrite(pin, LOW);
  delay(400);
  pin++;
  if (pin > 7) pin = 0;
}
