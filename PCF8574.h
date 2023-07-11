/*
 * PCF8574 GPIO Port Expand
 *
 * AUTHOR:  Renzo Mischianti
 * VERSION: 2.3.6
 *
 * https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Renzo Mischianti www.mischianti.org All right reserved.
 *
 * You may copy, alter and reuse this code in any way you like, but please leave
 * reference to www.mischianti.org in your comments if you redistribute this code.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef PCF8574_h
#define PCF8574_h

#include "Wire.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DEFAULT_SDA SDA;
#define DEFAULT_SCL SCL;

// Uncomment to enable printing out nice debug messages.
//  #define PCF8574_DEBUG

// Uncomment for low memory usage this prevent use of complex DigitalInput structure and free 7byte of memory
// #define PCF8574_LOW_MEMORY

// Uncomment for low latency to get realtime data every time.
// #define PCF8574_LOW_LATENCY

//#define PCF8574_SOFT_INITIALIZATION

// Select an algorithm to manage encoder progression
 #define BASIC_ENCODER_ALGORITHM
// #define MISCHIANTI_ENCODER_ALGORITHM
// #define SEQUENCE_ENCODER_ALGORITHM_REDUCED
// #define SEQUENCE_ENCODER_ALGORITHM
// #define POKI_ENCODER_ALGORITHM

// Define where debug output will be printed.
#define DEBUG_PRINTER Serial

// Setup debug printing macros.
#ifdef PCF8574_DEBUG
	#define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
	#define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
	#define DEBUG_PRINT(...) {}
	#define DEBUG_PRINTLN(...) {}
#endif

#ifdef PCF8574_LOW_LATENCY
	#define READ_ELAPSED_TIME 0
#else
	#define READ_ELAPSED_TIME 10
#endif

//#define P0  	B00000001
//#define P1  	B00000010
//#define P2  	B00000100
//#define P3  	B00001000
//#define P4  	B00010000
//#define P5  	B00100000
//#define P6  	B01000000
//#define P7  	B10000000
//
#define P0  	0
#define P1  	1
#define P2  	2
#define P3  	3
#define P4  	4
#define P5  	5
#define P6  	6
#define P7  	7

#include <math.h>


class PCF8574 {
public:

	PCF8574(uint8_t address);
	PCF8574(uint8_t address, uint8_t interruptPin,  void (*interruptFunction)() );

#if !defined(__AVR) && !defined(ARDUINO_ARCH_SAMD) && !defined(TEENSYDUINO) && !defined(ARDUINO_ARCH_RENESAS)
	PCF8574(uint8_t address, int sda, int scl);
	PCF8574(uint8_t address, int sda, int scl, uint8_t interruptPin,  void (*interruptFunction)());
#endif

#if defined(ESP32) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_RENESAS)
	///// changes for second i2c bus
	PCF8574(TwoWire *pWire, uint8_t address);
	PCF8574(TwoWire *pWire, uint8_t address, uint8_t interruptPin,  void (*interruptFunction)() );
#endif
#if defined(ESP32)
	PCF8574(TwoWire *pWire, uint8_t address, int sda, int scl);
	PCF8574(TwoWire *pWire, uint8_t address, int sda, int scl, uint8_t interruptPin,  void (*interruptFunction)());
#endif

	bool begin();
	void pinMode(uint8_t pin, uint8_t mode, uint8_t output_start = HIGH);

	void encoder(uint8_t pinA, uint8_t pinB);

	void attachInterrupt();
	void detachInterrupt();

	void readBuffer(bool force = true);
	uint8_t digitalRead(uint8_t pin, bool forceReadNow = false);
	#ifndef PCF8574_LOW_MEMORY
		struct DigitalInput {
			uint8_t p0;
			uint8_t p1;
			uint8_t p2;
			uint8_t p3;
			uint8_t p4;
			uint8_t p5;
			uint8_t p6;
			uint8_t p7;
		} digitalInput;


		DigitalInput digitalReadAll(void);

		bool digitalWriteAll(PCF8574::DigitalInput digitalInput);
	#else
		byte digitalReadAll(void);
		bool digitalWriteAll(byte digitalInput);
	#endif
	bool digitalWrite(uint8_t pin, uint8_t value);

#ifdef MISCHIANTI_ENCODER_ALGORITHM
	bool readEncoderValueMischianti(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation = false);
	int8_t readEncoderValueMischianti(uint8_t pinA, uint8_t pinB);
#endif
#ifdef POKI_ENCODER_ALGORITHM
	bool readEncoderValuePoki(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation = false);
	int8_t readEncoderValuePoki(uint8_t pinA, uint8_t pinB);
#endif

//	bool readEncoderValueEvolved(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation = false);
//	int8_t readEncoderValueEvolved(uint8_t pinA, uint8_t pinB);

#ifdef SEQUENCE_ENCODER_ALGORITHM
	bool readEncoderValueSequence(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation = false);
	int8_t readEncoderValueSequence(uint8_t pinA, uint8_t pinB);
#endif
#ifdef SEQUENCE_ENCODER_ALGORITHM_REDUCED
	bool readEncoderValueSequenceReduced(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation = false);
	int8_t readEncoderValueSequenceReduced(uint8_t pinA, uint8_t pinB);
#endif
#ifdef BASIC_ENCODER_ALGORITHM
	bool readEncoderValue(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation = false);
	int8_t readEncoderValue(uint8_t pinA, uint8_t pinB);
#endif

	int getLatency() const {
		return latency;
	}

	void setLatency(int latency = READ_ELAPSED_TIME) {
		this->latency = latency;
	}

	uint8_t getTransmissionStatusCode() const {
		return transmissionStatus;
	}

	bool isLastTransmissionSuccess(){
		DEBUG_PRINT(F("STATUS --> "));
		DEBUG_PRINTLN(transmissionStatus);
		return transmissionStatus==0;
	}
private:
	uint8_t _address;

	#if !defined(DEFAULT_SDA)
	#  if defined(ARDUINO_ARCH_STM32)
	#    define DEFAULT_SDA PB7
	#  elif defined(ESP8266)
	#    define DEFAULT_SDA 4
	#  elif defined(SDA)
	#    define DEFAULT_SDA SDA
	#  else
	#    error "Error define DEFAULT_SDA, SDA not declared, if you have this error contact the mantainer"
	#  endif
	#endif
	#if !defined(DEFAULT_SCL)
	#  if defined(ARDUINO_ARCH_STM32)
	#    define DEFAULT_SCL PB6
	#  elif defined(ESP8266)
	#    define DEFAULT_SCL 5
	#  elif defined(SDA)
	#    define DEFAULT_SCL SCL
	#  else
	#    error "Error define DEFAULT_SCL, SCL not declared, if you have this error contact the mantainer"
	#  endif
	#endif

	int _sda = DEFAULT_SDA;
	int _scl = DEFAULT_SCL;

	TwoWire *_wire;

	bool _usingInterrupt = false;
	uint8_t _interruptPin = 2;
	void (*_interruptFunction)(){};

	byte writeMode 			= 	0b00000000;
	byte writeModeUp		= 	0b00000000;
	byte readMode 			= 	0b00000000;
	byte readModePullUp 	= 	0b00000000;
	byte readModePullDown 	= 	0b00000000;
	byte byteBuffered 		= 	0b00000000;
	byte resetInitial		= 	0b00000000;
	byte initialBuffer		= 	0b00000000;
	unsigned long lastReadMillis = 0;

	byte writeByteBuffered = 0b00000000;

	volatile byte encoderValues = 0b00000000;

	uint8_t prevNextCode = 0;
	uint16_t store=0;

	int latency = READ_ELAPSED_TIME;

	bool checkProgression(byte oldValA, byte newValA, byte oldValB, byte newValB, byte validProgression);

//	byte validCW = B11100001;
//	byte validCCW = B01001011;
	byte validCW = 0b01001011;
	byte validCCW = 0b11100001;

	uint8_t transmissionStatus = 0;

	void setVal(uint8_t pin, uint8_t value);
	bool digitalWriteAllBytes(byte allpins);
};

#endif

