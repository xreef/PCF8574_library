/** \mainpage Thermistor library
 *
 * MIT license
 * written by Renzo Mischianti
 */

#ifndef Thermistor_h
#define Thermistor_h

#include "Wire.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Uncomment to enable printing out nice debug messages.
// #define PCF8574_DEBUG

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

#define READ_ELAPSED_TIME 10

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

	PCF8574(uint8_t address);
	PCF8574(uint8_t address, uint8_t sda, uint8_t scl);

	PCF8574(uint8_t address, uint8_t interruptPin,  void (*interruptFunction)() );
	PCF8574(uint8_t address, uint8_t sda, uint8_t scl, uint8_t interruptPin,  void (*interruptFunction)());

	void begin();
	void pinMode(uint8_t pin, uint8_t mode);

	void readBuffer(bool force = true);
	uint8_t digitalRead(uint8_t pin);
	DigitalInput digitalReadAll(void);
	void digitalWrite(uint8_t pin, uint8_t value);

private:
	uint8_t _address;
	uint8_t _sda = SDA;
	uint8_t _scl = SCL;

	bool _usingInterrupt = false;
	uint8_t _interruptPin = 2;
	void (*_interruptFunction)(){};

	byte writeMode 	= 	B00000000;
	byte readMode 	= 	B00000000;
	byte byteBuffered = B00000000;
	unsigned long lastReadMillis = 0;

	byte writeByteBuffered = B00000000;

};

#endif
