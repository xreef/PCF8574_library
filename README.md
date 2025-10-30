<div>
<a href="https://www.mischianti.org/forums/forum/mischiantis-libraries/pcf8574-i2c-digital-i-o-expander/"><img
  src="https://github.com/xreef/LoRa_E32_Series_Library/raw/master/resources/buttonSupportForumEnglish.png" alt="Support forum pcf8574 English"
   align="right"></a>
</div>
<div>
<a href="https://www.mischianti.org/it/forums/forum/le-librerie-di-mischianti/pcf8574-expander-digitale-i-o-i2c/"><img
  src="https://github.com/xreef/LoRa_E32_Series_Library/raw/master/resources/buttonSupportForumItaliano.png" alt="Forum supporto pcf8574 italiano"
  align="right"></a>
</div>

#


# PCF8574 PCF8574AP digital input and output expander with i2c bus.
#### Complete documentation on my site: [pcf8574 Article](https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/).

#### If you need more pins [here](https://www.mischianti.org/2019/07/22/pcf8575-i2c-16-bit-digital-i-o-expander/) you can find the pcf8575 16bit version of the IC.

## Changelog
 - 30/10/2025: v2.3.8 Add beginResult() to help users find their wrong code
 - 01/02/2024: v2.3.7 Add the possibility to insert address at begin()
 - 10/07/2023: v2.3.6 Support for Arduino UNO R4
 - 08/02/2023: v2.3.5 Fix STM32 support and add support for Raspberry Pi Pico and other rp2040 boards
 - 10/08/2022: v2.3.4 Add support for custom SERCOM interface of Arduino SAMD devices. Force SDA SCL to use GPIO numeration for STM32 bug (https://www.mischianti.org/forums/topic/compatible-with-stm32duino/). 
 - 28/07/2022: v2.3.3 Force SDA SCL to use GPIO numeration (https://www.mischianti.org/forums/topic/cannot-set-sda-clk-on-esp8266/).
 - 28/07/2022: v2.3.2 Fix the SDA SCL type #58 and add basic support for SAMD device.
 - 26/04/2022: v2.3.1 Fix example for esp32 and double begin issue #56.
 - 06/04/2022: v2.3.0 Fix package size
 - 30/12/2021: v2.2.4 Minor fix and remove deprecated declaration
 - 23/11/2020: v2.2.2 Add multiple implementation for encoder management (you can enable by uncomment relative define)

I try to simplify the use of this IC, with a minimal set of operations.

Tested with esp8266, esp32, Arduino, Arduino SAMD (Nano 33 IoT, MKR etc.), STM32 and rp2040 (Raspberry Pi Pico and similar)

PCF8574P address map 0x20-0x27 
PCF8574AP address map 0x38-0x3f 

**Constructor:**
Pass the address of I2C (to check the address use this guide [I2cScanner](https://playground.arduino.cc/Main/I2cScanner)) 
```cpp
	PCF8574(uint8_t address);
```
For ESP8266 if you want to specify SDA and SCL pins use this:

```cpp
	PCF8574(uint8_t address, uint8_t sda, uint8_t scl);
```
You must set input/output mode:
```cpp
	pcf8574.pinMode(P0, OUTPUT);
	pcf8574.pinMode(P1, INPUT);
	pcf8574.pinMode(P2, INPUT);
```

then IC as you can see in the image has 8 digital input/output ports:

![PCF8574 schema](https://github.com/xreef/PCF8574_library/blob/master/resources/PCF8574-pins.gif)

To read all analog input in one trasmission you can do (even if I use a 10millis debounce time to prevent too much read from i2c):
```cpp
	PCF8574::DigitalInput di = PCF8574.digitalReadAll();
	Serial.print(di.p0);
	Serial.print(" - ");
	Serial.print(di.p1);
	Serial.print(" - ");
	Serial.print(di.p2);
	Serial.print(" - ");
	Serial.println(di.p3);
```

To follow a request (you can see It on [issue #5](https://github.com/xreef/PCF8574_library/issues/5)) I create a define variable to work with low memory devices, if you uncomment this line in the .h file of the library:

```cpp
// #define PCF8574_LOW_MEMORY
```

Enable low memory props and gain about 7 bytes of memory, and you must use the method to read all like so:

 ```cpp
	byte di = pcf8574.digitalReadAll();
	Serial.print("READ VALUE FROM PCF: ");
	Serial.println(di, BIN);
```

where `di` is a byte like 1110001, so you must do a bitwise operation to get the data, operation that I already do in the "normal" mode. For example:

 ```cpp
	p0 = ((di & bit(0))>0)?HIGH:LOW;
	p1 = ((di & bit(1))>0)?HIGH:LOW;
	p2 = ((di & bit(2))>0)?HIGH:LOW;
	p3 = ((di & bit(3))>0)?HIGH:LOW;
	p4 = ((di & bit(4))>0)?HIGH:LOW;
	p5 = ((di & bit(5))>0)?HIGH:LOW;
	p6 = ((di & bit(6))>0)?HIGH:LOW;
	p7 = ((di & bit(7))>0)?HIGH:LOW;
 ```
 

if you want to read a single input:

```cpp
	int p1Digital = PCF8574.digitalRead(P1); // read P1
```

If you want to write a digital value:
```cpp
	PCF8574.digitalWrite(P1, HIGH);
```
or:
```cpp
	PCF8574.digitalWrite(P1, LOW);
```

You can also use an interrupt pin:
You must initialize the pin and the function to call when interrupt raised from PCF8574
```cpp
// Function interrupt
void keyPressedOnPCF8574();

// Set i2c address
PCF8574 pcf8574(0x39, ARDUINO_UNO_INTERRUPT_PIN, keyPressedOnPCF8574);
```
Remember you can't use Serial or Wire on an interrupt function.

It's better to only set a variable to read on loop:
```cpp
void keyPressedOnPCF8574(){
	// Interrupt called (No Serial no read no wire in this function, and DEBUG disabled on PCF library)
	 keyPressed = true;
}
```

For the examples I use this wire schema on breadboard:
![Breadboard](https://github.com/xreef/PCF8574_library/raw/master/resources/testReadWriteLedButton_bb.png)

https://downloads.arduino.cc/libraries/logs/github.com/xreef/PCF8574_library/

## Example: Using an HC‑SR04 Ultrasonic Sensor

Note: The HC‑SR04 normally requires a microcontroller digital pin with precise timing (native `pulseIn`). If you connect the `ECHO` pin to a PCF8574 input, measurements will be affected by I2C latency and polling; accuracy will therefore be reduced. Below are two approaches: "higher resolution" (uses `pulseIn`) and "low I2C traffic" (uses `pulseInPoll`).

Wiring: connect TRIG to an Arduino digital pin, and ECHO to a PCF8574 pin configured as INPUT.

Schematic example sketch:

```cpp
// Arduino side
#include <Wire.h>
#include <PCF8574.h>

PCF8574 pcf(0x39); // example address

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

  unsigned long duration = pcf.pulseIn(echoPinPCF, HIGH, 30000UL); // 30 ms timeout
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

  unsigned long d2 = measureDistancePulseInPoll(100); // polling 100 us
  Serial.print("Distance pulseInPoll(100us): "); Serial.print(d2); Serial.println(" cm");

  delay(500);
}
```

Accuracy warnings
- The `PCF8574` is an I2C I/O expander; each read requires an I2C transaction which introduces latency and timing jitter.
- `pulseIn` forces many immediate reads and gives better time resolution than `pulseInPoll`, but it cannot match the precision of a native microcontroller pin measurement.
- `pulseInPoll` reduces the number of I2C reads by sampling at configurable intervals (`pollIntervalMicros`), lowering bus load while reducing time resolution — choose a tradeoff that fits your application.
- For accurate distance measurements (especially for short ranges where the echo pulse is brief) connect `ECHO` directly to a microcontroller digital pin and use the native `pulseIn`.

Practical tips
- Try `pollIntervalMicros` between 20 and 200 microseconds and test on your hardware to find a good compromise.
- If `pulseIn` via PCF8574 produces inconsistent readings, try disabling other I2C activity during the measurement or use `pulseInPoll` with a shorter polling interval.
