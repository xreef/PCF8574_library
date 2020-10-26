/*
 * PCF8574 GPIO Port Expand
 * http://nopnop2002.webcrow.jp/WeMos/WeMos-25.html
 *
 * PCF8574    ----- WeMos
 * A0         ----- GRD
 * A1         ----- GRD
 * A2         ----- GRD
 * VSS        ----- GRD
 * VDD        ----- 5V/3.3V
 * SDA        ----- GPIO_4(PullUp)
 * SCL        ----- GPIO_5(PullUp)
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

// Set i2c address
PCF8574 pcf8574(0x20);

void setup()
{
  Serial.begin(9600);
  delay(1000);

  // Set pinMode to OUTPUT
  for(int i=0;i<8;i++) {
    pcf8574.pinMode(i, OUTPUT);
  }
	Serial.print("Init pcf8574...");
	if (pcf8574.begin()){
		Serial.println("OK");
	}else{
		Serial.println("KO");
	}
}

void loop()
{
  static int pin = 0;
  pcf8574.digitalWrite(pin, HIGH);
  delay(1000);
  pcf8574.digitalWrite(pin, LOW);
  delay(1000);
  pin++;
  if (pin > 7) pin = 0;
}
