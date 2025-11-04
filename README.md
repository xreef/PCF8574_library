# PCF8574 I2C Digital I/O Expander Library

![PCF8574 Library Logo](resources/pcf8574_library_logo.png)

[![arduino-library-badge](https://www.ardu-badge.com/badge/PCF8574%20library.svg?)](https://www.ardu-badge.com/PCF8574%20library) 
[![](https://img.shields.io/badge/Platform-Arduino%20%7C%20SAMD%20%7C%20ESP32%20%7C%20ESP8266%20%7C%20RP2040%20%7C%20STM32-green.svg)]()  
[![](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)

A simple and efficient library to use the PCF8574 I2C 8-bit digital I/O expander with Arduino, ESP8266, ESP32, and other platforms.

**Author:** Renzo Mischianti  
**Website:** [www.mischianti.org](https://www.mischianti.org/category/my-libraries/pcf8574-i2c-digital-i-o-expander/)  
**GitHub:** [xreef/PCF8574_library](https://github.com/xreef/PCF8574_library)

---

## 📚 Documentation & Articles
Complete documentation, tutorials, and examples are available on mischianti.org.

- 🌐 **[PCF8574 Main Article](https://mischianti.org/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/)**: The primary guide for this library with wiring diagrams and detailed explanations.
- 📖 **[PCF8574 and rotary encoder](https://www.mischianti.org/pcf8574-i2c-digital-i-o-expander-rotary-encoder-part-2/)**: A guide to using a rotary encoder with the PCF8574.
- 📖 **[PCF8575 16-bit I/O Expander](https://mischianti.org/pcf8575-i2c-16-bit-digital-i-o-expander/)**: A guide for the 16-bit version of this IC.
- 🛠️ **[I2C Scanner](https://playground.arduino.cc/Main/I2cScanner)**: A utility to find the I2C address of your device.
- 💬 **[Support Forum (English)](https://www.mischianti.org/forums/forum/mischiantis-libraries/pcf8574-i2c-digital-i-o-expander/)**: Get help and discuss the library.
- 💬 **[Forum di Supporto (Italiano)](https://www.mischianti.org/it/forums/forum/le-librerie-di-mischianti/pcf8574-expander-digitale-i-o-i2c/)**: Ottieni supporto e discuti della libreria.

---

## 📋 Table of Contents
- [Features](#-features)
- [Supported Platforms](#-supported-platforms)
- [Installation](#-installation)
- [API Overview](#-api-overview)
- [Basic Usage](#-basic-usage)
- [Interrupts](#-interrupts)
- [Rotary Encoder Support](#-rotary-encoder-support)
- [Low Memory Mode](#-low-memory-mode)
- [HC-SR04 Ultrasonic Sensor Example](#example-using-an-hc-sr04-ultrasonic-sensor)
- [Changelog](#-changelog)
- [License](#-license)
- [Contributing](#-contributing)
- [Support & Contact](#-support--contact)

## ✨ Features

- **8 Digital I/O Pins**: Expand your microcontroller's I/O capabilities over I2C.
- **Input & Output Modes**: Each pin can be individually configured as an `INPUT` or `OUTPUT`.
- **Interrupt Support**: Use the PCF8574's interrupt pin to detect input changes without polling.
- **Rotary Encoder Support**: Built-in functions to simplify reading rotary encoders.
- **Flexible I2C Configuration**:
    - Supports all PCF8574 and PCF8574A I2C addresses (0x20-0x27 and 0x38-0x3F).
    - Custom I2C pins (SDA, SCL) can be specified for platforms like ESP8266/ESP32.
- **Efficient Reading**: Read all 8 pin states in a single I2C transaction with `digitalReadAll()`.
- **Low Memory Mode**: An optional mode to reduce RAM footprint on memory-constrained devices.
- **Specialized Functions**: Includes `pulseIn()` and `pulseInPoll()` for reading pulse widths, plus NewPing-style ultrasonic sensor methods (`ping()`, `ping_cm()`, `ping_median()`, etc.) for easy HC-SR04 integration.

## 🎯 Supported Platforms

| Platform | Support | Notes |
|---|---|---|
| ESP32 | ✅ | Full support |
| ESP8266 | ✅ | Full support |
| Raspberry Pi Pico (RP2040) | ✅ | Full support |
| Arduino (AVR, Uno, Mega) | ✅ | Full support |
| Arduino SAMD (Nano 33 IoT, etc.) | ✅ | Full support |
| STM32 | ✅ | Full support |

## 📦 Installation

### Arduino IDE (Library Manager)
1. Open Arduino IDE.
2. Go to `Sketch` > `Include Library` > `Manage Libraries...`.
3. Search for "**PCF8574_library**" by Renzo Mischianti.
4. Click `Install`.

### PlatformIO
Add the library to your `platformio.ini` file:
```ini
lib_deps = xreef/PCF8574 library
```

### Manual Installation
1. Download the latest release from [GitHub](https://github.com/xreef/PCF8574_library/releases).
2. Unzip the folder and rename it to `PCF8574_library`.
3. Move it to your Arduino `libraries` folder.
4. Restart the Arduino IDE.

---

## 🚀 API Overview

### Constructor
```cpp
// Basic constructor with I2C address
PCF8574(uint8_t address);

// For ESP8266/ESP32 with custom SDA/SCL pins
PCF8574(uint8_t address, uint8_t sda, uint8_t scl);

// Constructor with interrupt pin
PCF8574(uint8_t address, uint8_t interruptPin, void (*interruptFunction)());
```

### Core Functions
```cpp
// Initialize the library (must be called in setup())
bool begin(uint8_t defaultVal = 0xFF);

// Set a pin as INPUT or OUTPUT
void pinMode(uint8_t pin, uint8_t mode);

// Write a value (HIGH/LOW) to a pin
void digitalWrite(uint8_t pin, uint8_t value);

// Read a value (HIGH/LOW) from a pin
uint8_t digitalRead(uint8_t pin);

// Read all 8 pins at once
PCF8574::DigitalInput digitalReadAll();

// Pulse measurement (low-level)
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);
unsigned long pulseInPoll(uint8_t pin, uint8_t state, unsigned long timeout, unsigned int pollIntervalMicros);

// Ultrasonic sensor methods (HC-SR04 compatible, NewPing-style)
unsigned long ping(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm = 500);
unsigned long pingPoll(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm = 500, unsigned int pollIntervalMicros = 100);
unsigned long ping_cm(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm = 500);
unsigned long ping_cm_poll(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm = 500, unsigned int pollIntervalMicros = 100);
unsigned long ping_in(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm = 500);
unsigned long ping_in_poll(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm = 500, unsigned int pollIntervalMicros = 100);
unsigned long ping_median(uint8_t trigPin, uint8_t echoPin, uint8_t iterations = 5, unsigned long maxDistance_cm = 500);
unsigned long ping_median_poll(uint8_t trigPin, uint8_t echoPin, uint8_t iterations = 5, unsigned long maxDistance_cm = 500, unsigned int pollIntervalMicros = 100);
static unsigned long microsecondsToDistance_cm(unsigned long microseconds);
static unsigned long microsecondsToDistance_in(unsigned long microseconds);
```

## 💡 Basic Usage

Here is a simple example of how to use the library to control an LED and read a button.

```cpp
#include <Wire.h>
#include <PCF8574.h>

// Set I2C address (e.g., 0x20)
PCF8574 pcf8574(0x20);

const uint8_t LED_PIN = P0;
const uint8_t BUTTON_PIN = P1;

void setup() {
  Serial.begin(115200);

  // Initialize the PCF8574
  if (!pcf8574.begin()) {
    Serial.println("Couldn't find PCF8574");
    while (1);
  }

  // Set pin modes
  pcf8574.pinMode(LED_PIN, OUTPUT);
  pcf8574.pinMode(BUTTON_PIN, INPUT);

  Serial.println("PCF8574 initialized!");
}

void loop() {
  // Read the button state
  uint8_t buttonState = pcf8574.digitalRead(BUTTON_PIN);

  // If button is pressed (assuming active-low), turn on the LED
  if (buttonState == LOW) {
    pcf8574.digitalWrite(LED_PIN, HIGH);
  } else {
    pcf8574.digitalWrite(LED_PIN, LOW);
  }

  delay(100);
}

/** Important:** Always configure PCF8574 pins with `pinMode()` (and set initial output levels with `digitalWrite()` if needed) before calling `begin()`. Some platforms or usage patterns rely on the initial pin configuration and calling `begin()` before `pinMode()` can lead to unexpected behavior. Example:

```cpp
pcf.pinMode(P0, OUTPUT);
pcf.pinMode(P1, INPUT);
// optional: pcf.digitalWrite(P0, LOW);
if (!pcf.begin()) {
  Serial.println(F("ERROR: Could not initialize PCF8574! Check wiring, I2C address, SDA/SCL and power."));
  while (1) delay(100);
}
``` **/

## ⚡ Interrupts

You can use an interrupt to detect input changes without constantly polling the device.

1.  Connect the `INT` pin of the PCF8574 to an interrupt-capable pin on your microcontroller.
2.  Define an interrupt handler function (ISR).
3.  Initialize the library with the interrupt pin and handler.

```cpp
// Global flag to be set by the ISR
volatile bool keyPressed = false;

// Interrupt Service Routine (ISR)
// NOTE: Keep this function as short and fast as possible.
void ICACHE_RAM_ATTR keyPressedOnPCF8574() {
  keyPressed = true;
}

// Initialize with interrupt pin and ISR
PCF8574 pcf8574(0x20, D3, keyPressedOnPCF8574); // Address, Interrupt Pin, ISR

void setup() {
  // ... setup code ...
  pcf8574.begin();
}

void loop() {
  if (keyPressed) {
    // Reset the flag
    keyPressed = false;

    // An input on the PCF8574 has changed, read all values
    PCF8574::DigitalInput values = pcf8574.digitalReadAll();
    Serial.print("Pin P0 is: ");
    Serial.println(values.p0);
    // ... check other pins ...
  }
}
```

## 🔄 Rotary Encoder Support

The library offers built-in support for rotary encoders. To use it, enable one of the encoder implementations in the `PCF8574.h` file by uncommenting the corresponding `#define`.

Example implementation:
```cpp
// In PCF8574.h, uncomment one of these:
// #define PCF8574_ENCODER_SUPPORT
// #define PCF8574_ENCODER_SUPPORT_OPTIMIZED

#include <Wire.h>
#include <PCF8574.h>

PCF8574 pcf8574(0x20);

// Define encoder pins
const uint8_t ENCODER_A = P0;
const uint8_t ENCODER_B = P1;

volatile long encoderValue = 0;

void setup() {
  Serial.begin(115200);
  pcf8574.begin();

  // Initialize the encoder
  pcf8574.encoder(ENCODER_A, ENCODER_B);
  pcf8574.setEncoderValue(0);
}

void loop() {
  long newEncoderValue = pcf8574.getEncoderValue();

  if (newEncoderValue != encoderValue) {
    Serial.print("Encoder value: ");
    Serial.println(newEncoderValue);
    encoderValue = newEncoderValue;
  }

  delay(10);
}
```

## 🧠 Low Memory Mode

For devices with very limited RAM, you can enable a low-memory mode.
1.  Uncomment `#define PCF8574_LOW_MEMORY` in `PCF8574.h`.
2.  This saves ~7 bytes of RAM by changing how `digitalReadAll()` works.

Instead of returning a struct, `digitalReadAll()` will return a single `byte`. You must then use bitwise operations to get the value of each pin.

```cpp
// In your sketch, after enabling low memory mode:
byte pinValues = pcf8574.digitalReadAll();

bool p0 = (pinValues & bit(0)) > 0;
bool p1 = (pinValues & bit(1)) > 0;
// ... and so on for all 8 pins
```

## Example: Using an HC-SR04 Ultrasonic Sensor

The library includes specialized methods for working with HC-SR04 ultrasonic sensors, similar to the NewPing library. These methods simplify distance measurement and reduce I2C traffic.

> **⚠️ Accuracy Warning:** Measuring pulses via an I2C expander is less precise than using a direct microcontroller pin. I2C communication adds latency and jitter. The polling methods (`_poll` variants) trade some precision for much better I2C bus efficiency by reading at intervals instead of continuously. For high-accuracy needs, connect the sensor's `ECHO` pin directly to your microcontroller.

### Available Ultrasonic Methods

| Method | Description | Returns |
|--------|-------------|---------|
| `ping(trigPin, echoPin, maxDistance_cm)` | Send trigger and measure echo pulse | Microseconds |
| `pingPoll(trigPin, echoPin, maxDistance_cm, pollInterval)` | Ping with polling to reduce I2C traffic | Microseconds |
| `ping_cm(trigPin, echoPin, maxDistance_cm)` | Get distance in centimeters | Centimeters |
| `ping_cm_poll(trigPin, echoPin, maxDistance_cm, pollInterval)` | Distance in cm with polling (recommended) | Centimeters |
| `ping_in(trigPin, echoPin, maxDistance_cm)` | Get distance in inches | Inches |
| `ping_in_poll(trigPin, echoPin, maxDistance_cm, pollInterval)` | Distance in inches with polling | Inches |
| `ping_median(trigPin, echoPin, iterations, maxDistance_cm)` | Median of multiple readings (most stable) | Centimeters |
| `ping_median_poll(trigPin, echoPin, iterations, maxDistance_cm, pollInterval)` | Median with polling | Centimeters |

### Basic Example (Original Methods)

```cpp
#include <Wire.h>
#include <PCF8574.h>

PCF8574 pcf(0x20);

const int trigPin = 9;          // A standard digital pin on your Arduino/ESP
const uint8_t echoPinPCF = P0;  // A pin on the PCF8574

void setup() {
  Serial.begin(115200);
  pcf.begin();
  pcf.pinMode(echoPinPCF, INPUT);
  pinMode(trigPin, OUTPUT);
}

void loop() {
  // Trigger the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the echo pulse duration on the PCF8574 pin
  // Timeout is 30000 microseconds (30ms)
  unsigned long duration = pcf.pulseIn(echoPinPCF, HIGH, 30000UL);

  // Calculate distance
  unsigned long distanceCm = duration / 29 / 2;

  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  delay(500);
}
```

### NewPing-Style Example (Recommended)

For easier use and better I2C efficiency, use the new ultrasonic methods:

```cpp
#include <Wire.h>
#include <PCF8574.h>

PCF8574 pcf(0x20);

const uint8_t trigPinPCF = P1;  // TRIG on PCF8574 P1
const uint8_t echoPinPCF = P0;  // ECHO on PCF8574 P0
const unsigned long MAX_DISTANCE = 400; // Maximum distance in cm

void setup() {
  Serial.begin(115200);
  pcf.begin();
  pcf.pinMode(trigPinPCF, OUTPUT);
  pcf.pinMode(echoPinPCF, INPUT);
  pcf.digitalWrite(trigPinPCF, LOW);
}

void loop() {
  // Simple distance measurement in cm (with polling for efficiency)
  unsigned long distance = pcf.ping_cm_poll(trigPinPCF, echoPinPCF, MAX_DISTANCE, 100);
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // For more stable readings, use median of 5 samples
  // unsigned long distance = pcf.ping_median_poll(trigPinPCF, echoPinPCF, 5, MAX_DISTANCE, 100);
  
  delay(500);
}
```

### Understanding Polling Interval

The `pollInterval` parameter (in microseconds) controls how often the PCF8574 is read:
- **Smaller values (50-100µs)**: More precise timing, but more I2C traffic
- **Larger values (200-500µs)**: Less I2C traffic, but reduced precision
- **Recommended**: 100µs provides a good balance for most applications

Example: `ping_cm_poll(trigPin, echoPin, 400, 100)` reads every 100µs, reducing I2C requests by ~10x compared to continuous reading.

See the `HC-SR04_via_PCF8574` and `HC-SR04_via_PCF8574_extended` examples in the library for complete demonstrations.

## 📝 Changelog
- 04/11/2025: v2.4.0 Add NewPing-style ultrasonic sensor methods (ping, ping_cm, ping_in, ping_median) for HC-SR04 support
- 30/10/2025: v2.3.8 Add beginResult() to help users find their wrong code
- 01/02/2024: v2.3.7 Add the possibility to insert address at begin()
- 10/07/2023: v2.3.6 Support for Arduino UNO R4
- 08/02/2023: v2.3.5 Fix STM32 support and add support for Raspberry Pi Pico and other rp2040 boards
- 10/08/2022: v2.3.4 Add support for custom SERCOM interface of Arduino SAMD devices. Force SDA SCL to use GPIO numeration for STM32 bug.
- 28/07/2022: v2.3.3 Force SDA SCL to use GPIO numeration.
- 28/07/2022: v2.3.2 Fix the SDA SCL type #58 and add basic support for SAMD device.
- 26/04/2022: v2.3.1 Fix example for esp32 and double begin issue #56.
- 06/04/2022: v2.3.0 Fix package size
- 30/12/2021: v2.2.4 Minor fix and remove deprecated declaration
- 23/11/2020: v2.2.2 Add multiple implementation for encoder management

## 📄 License
This library is released under the MIT License. See the `LICENSE` file for more details.

Copyright (c) 2017-2025 Renzo Mischianti

## 🤝 Contributing
Contributions are welcome! Please fork the repository, create a feature branch, and submit a pull request.

## 📞 Support & Contact
- **Documentation:** [https://www.mischianti.org/category/my-libraries/pcf8574-i2c-digital-i-o-expander/](https://www.mischianti.org/category/my-libraries/pcf8574-i2c-digital-i-o-expander/)
- **GitHub Issues:** [https://github.com/xreef/PCF8574_library/issues](https://github.com/xreef/PCF8574_library/issues)
- **Author:** Renzo Mischianti ([@xreef](https://github.com/xreef))

---

⭐ If this library helped your project, please give it a star on GitHub!
