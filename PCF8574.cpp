/*
 * PCF8574 GPIO Port Expand
 * https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Renzo Mischianti www.mischianti.org All right reserved.
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

#include "PCF8574.h"
#include "Wire.h"

#ifdef PCF8574_BEGIN_ENUM_RESULT
static bool probeI2CDevice(TwoWire *wire, uint8_t address){
    wire->beginTransmission(address);
    const uint8_t err = wire->endTransmission();
    return (err == 0);
}

BeginResult PCF8574::beginResult(){
    // If there are no pins configured (both writeMode and readMode are zero)
    if (writeMode == 0 && readMode == 0){
        // initialize Wire with custom pins if needed (existing begin() does this)
        #if !defined(__AVR)  && !defined(ARDUINO_ARCH_SAMD)  && !defined(TEENSYDUINO) && !defined(ARDUINO_ARCH_RENESAS)
            #ifdef ARDUINO_ARCH_STM32
                _wire->begin((uint32_t)_sda, (uint32_t)_scl);
            #elif defined(ARDUINO_ARCH_RP2040)
                _wire->setSCL(_scl);
                _wire->setSDA(_sda);
                _wire->begin();
            #else
                _wire->begin((int)_sda, (int)_scl);
            #endif
        #else
            _wire->begin();
        #endif

        // Still set internal state but do not write anything to the device
        lastReadMillis = millis();
        PCF8574::attachInterrupt();
        return BeginResult::NO_PINS_CONFIGURED;
    }

    // If pins are configured, delegate to existing begin() to keep current behavior
    bool ok = PCF8574::begin();

    return ok ? BeginResult::OK : BeginResult::I2C_ERROR;
}

BeginResult PCF8574::beginResult(uint8_t address){
    _address = address;
    return PCF8574::beginResult();
}

// Static helper that returns a human-readable C string for a BeginResult value.
const char* PCF8574::beginResultToString(BeginResult result){
	switch(result){
		case BeginResult::OK:
			return "OK";
		case BeginResult::I2C_ERROR:
			return "I2C_ERROR: no ACK from device (check wiring/I2C address/SDA-SCL/power).";
		case BeginResult::NO_PINS_CONFIGURED:
			return "NO_PINS_CONFIGURED: no pcf.pinMode() calls before begin().";
		case BeginResult::INVALID_ADDRESS:
			return "INVALID_ADDRESS: address out of supported ranges (check A0/A1/A2).";
		default:
			return "UNKNOWN_BEGIN_RESULT";
	}
}

// Convenience wrappers: call beginResult(), optionally print diagnostics, and return true on OK.
bool PCF8574::beginWithResultPrint(bool showDiagnostics){
	BeginResult r = beginResult();
	if (showDiagnostics) printBeginResult(r, true);
	return (r == BeginResult::OK);
}

#endif

// New lightweight probe for users: isOnline
bool PCF8574::isOnline(){
    // Use TwoWire instance to probe for ACK. This mirrors the probe logic used when
    // PCF8574_BEGIN_ENUM_RESULT is enabled.
    _wire->beginTransmission(_address);
    uint8_t err = _wire->endTransmission();
    return (err == 0);
}

/**
 * Constructor
 * @param address: i2c address
 */
PCF8574::PCF8574(uint8_t address){
	_wire = &Wire;

	_address = address;
};

/**
 * Construcor
 * @param address: i2c address
 * @param interruptPin: pin to set interrupt
 * @param interruptFunction: function to call when interrupt raised
 */
PCF8574::PCF8574(uint8_t address, uint8_t interruptPin,  void (*interruptFunction)() ){
	_wire = &Wire;

	_address = address;
	_interruptPin = interruptPin;
	_interruptFunction = interruptFunction;
	_usingInterrupt = true;
};

#if !defined(__AVR) && !defined(ARDUINO_ARCH_SAMD) && !defined(TEENSYDUINO) && !defined(ARDUINO_ARCH_RENESAS)
	/**
	 * Constructor
	 * @param address: i2c address
	 * @param sda: sda pin
	 * @param scl: scl pin
	 */
	PCF8574::PCF8574(uint8_t address, int sda, int scl){
		_wire = &Wire;

		_address = address;
		_sda = sda;
		_scl = scl;
	};

	/**
	 * Constructor
	 * @param address: i2c address
	 * @param sda: sda pin
	 * @param scl: scl pin
	 * @param interruptPin: pin to set interrupt
 	 * @param interruptFunction: function to call when interrupt raised
	 */
	PCF8574::PCF8574(uint8_t address, int sda, int scl, uint8_t interruptPin,  void (*interruptFunction)() ){
		_wire = &Wire;

		_address = address;
		_sda = sda;
		_scl = scl;

		_interruptPin = interruptPin;
		_interruptFunction = interruptFunction;

		_usingInterrupt = true;
	};
#endif

#if defined(ESP32) || defined(ARDUINO_ARCH_SAMD)|| defined(ARDUINO_ARCH_RP2040)  || defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_RENESAS)
	/**
	 * Constructor
	 * @param address: i2c address
	 */
	PCF8574::PCF8574(TwoWire *pWire, uint8_t address){
		_wire = pWire;

		_address = address;
	};

	/**
	 * Construcor
	 * @param address: i2c address
	 * @param interruptPin: pin to set interrupt
	 * @param interruptFunction: function to call when interrupt raised
	 */
	PCF8574::PCF8574(TwoWire *pWire, uint8_t address, uint8_t interruptPin,  void (*interruptFunction)() ){
		_wire = pWire;

		_address = address;
		_interruptPin = interruptPin;
		_interruptFunction = interruptFunction;
		_usingInterrupt = true;
	};
#endif
#if defined(ESP32)
	/**
	 * Constructor
	 * @param address: i2c address
	 * @param sda: sda pin
	 * @param scl: scl pin
	 */
	PCF8574::PCF8574(TwoWire *pWire, uint8_t address, int sda, int scl){
		_wire = pWire;

		_address = address;
		_sda = sda;
		_scl = scl;
	};

	/**
	 * Constructor
	 * @param address: i2c address
	 * @param sda: sda pin
	 * @param scl: scl pin
	 * @param interruptPin: pin to set interrupt
	 * @param interruptFunction: function to call when interrupt raised
	 */
	PCF8574::PCF8574(TwoWire *pWire, uint8_t address, int sda, int scl, uint8_t interruptPin,  void (*interruptFunction)() ){
		_wire = pWire;

		_address = address;
		_sda = sda;
		_scl = scl;

		_interruptPin = interruptPin;
		_interruptFunction = interruptFunction;

		_usingInterrupt = true;
	};
#endif
	bool encoderPins[8];

	void PCF8574::attachInterrupt(){
		// If using interrupt set interrupt value to pin
		if (_usingInterrupt){
			for (int i = 0; i < 8;i++){
				if (encoderPins[i]) PCF8574::digitalRead(i);
			}
//			PCF8574::digitalReadAll();
//			(*_interruptFunction)();

	//		PCF8574_DEBUG_PRINTLN("Using interrupt pin (not all pin is interrupted)");
	//		::pinMode(_interruptPin, INPUT_PULLUP);
	//		attachInterrupt(digitalPinToInterrupt(_interruptPin), (*_interruptFunction), FALLING );
			PCF8574_DEBUG_PRINTLN("Using interrupt pin (not all pin is interrupted)");
			::pinMode(_interruptPin, INPUT_PULLUP);
			::attachInterrupt(digitalPinToInterrupt(_interruptPin), (*_interruptFunction), FALLING );
		}

	}
	void PCF8574::detachInterrupt(){
		// If using interrupt set interrupt value to pin
		if (_usingInterrupt){
			::detachInterrupt(digitalPinToInterrupt(_interruptPin));
			PCF8574_DEBUG_PRINTLN("Detach interrupt pin");
		}

	}

bool PCF8574::begin(uint8_t address){
	_address = address;
	return PCF8574::begin();
}


/**
 * wake up i2c controller
 */
bool PCF8574::begin(){
	this->transmissionStatus = 4;
	#if !defined(__AVR)  && !defined(ARDUINO_ARCH_SAMD)  && !defined(TEENSYDUINO) && !defined(ARDUINO_ARCH_RENESAS)
		PCF8574_DEBUG_PRINT(F("begin(sda, scl) -> "));PCF8574_DEBUG_PRINT(_sda);PCF8574_DEBUG_PRINT(F(" "));PCF8574_DEBUG_PRINTLN(_scl);
//		_wire->begin(_sda, _scl);
#ifdef ARDUINO_ARCH_STM32
		_wire->begin((uint32_t)_sda, (uint32_t)_scl);
#elif defined(ARDUINO_ARCH_RP2040)
		_wire->setSCL(_scl);
		_wire->setSDA(_sda);
		_wire->begin();
#else
		_wire->begin((int)_sda, (int)_scl);
#endif
	#else
	//			Default pin for AVR some problem on software emulation
	//			#define SCL_PIN _scl
	// 			#define SDA_PIN _sda
		_wire->begin();
	#endif

	// Check if there are pins to set low
	if (writeMode>0 || readMode>0){
		PCF8574_DEBUG_PRINTLN("Set write mode");
		_wire->beginTransmission(_address);


		PCF8574_DEBUG_PRINT("resetInitial pin ");
#ifdef PCF8574_SOFT_INITIALIZATION
		resetInitial = writeModeUp | readModePullUp;
#else
		resetInitial = writeModeUp | readMode;
#endif
		PCF8574_DEBUG_PRINTLN( resetInitial, BIN);

		_wire->write(resetInitial);

		initialBuffer = writeModeUp | readModePullUp;
		byteBuffered = initialBuffer;
		writeByteBuffered = writeModeUp;

		PCF8574_DEBUG_PRINTLN("Start end trasmission if stop here check pullup resistor.");
		this->transmissionStatus = _wire->endTransmission();
	}

//	// If using interrupt set interrupt value to pin
//	if (_usingInterrupt){
////		PCF8574_DEBUG_PRINTLN("Using interrupt pin (not all pin is interrupted)");
////		::pinMode(_interruptPin, INPUT_PULLUP);
////		attachInterrupt(digitalPinToInterrupt(_interruptPin), (*_interruptFunction), FALLING );
//		PCF8574_DEBUG_PRINTLN("Using interrupt pin (not all pin is interrupted)");
//		::pinMode(_interruptPin, INPUT_PULLUP);
//		::attachInterrupt(digitalPinToInterrupt(_interruptPin), (*_interruptFunction), FALLING );
//	}

	PCF8574::attachInterrupt();

	// inizialize last read
	lastReadMillis = millis();

	return this->isLastTransmissionSuccess();
}

/**
 * Set if fin is OUTPUT or INPUT
 * @param pin: pin to set
 * @param mode: mode, supported only INPUT or OUTPUT (to simplify)
 * @param output_start: output_start, for OUTPUT we can set initial value
 */
void PCF8574::pinMode(uint8_t pin, uint8_t mode, uint8_t output_start){
	PCF8574_DEBUG_PRINT("Set pin ");
	PCF8574_DEBUG_PRINT(pin);
	PCF8574_DEBUG_PRINT(" as ");
	PCF8574_DEBUG_PRINTLN(mode);

	if (mode == OUTPUT){
		writeMode = writeMode | bit(pin);
		if (output_start==HIGH) {
			writeModeUp = writeModeUp | bit(pin);
		}

		readMode =  readMode & ~bit(pin);
		readModePullDown 	=	readModePullDown 	& 	~bit(pin);
		readModePullUp 		=	readModePullUp 		& 	~bit(pin);

		PCF8574_DEBUG_PRINT("W: ");
		PCF8574_DEBUG_PRINT(writeMode, BIN);
		PCF8574_DEBUG_PRINT(" R ALL: ");
		PCF8574_DEBUG_PRINT(readMode, BIN);

		PCF8574_DEBUG_PRINT(" R Down: ");
		PCF8574_DEBUG_PRINT(readModePullDown, BIN);
		PCF8574_DEBUG_PRINT("R Up: ");
		PCF8574_DEBUG_PRINTLN(readModePullUp, BIN);

	}else if (mode == INPUT){
		writeMode = writeMode & ~bit(pin);

		readMode 			=   readMode 			| bit(pin);
		readModePullDown 	=	readModePullDown 	| bit(pin);
		readModePullUp 		=	readModePullUp 		& ~bit(pin);

		PCF8574_DEBUG_PRINT("W: ");
		PCF8574_DEBUG_PRINT(writeMode, BIN);
		PCF8574_DEBUG_PRINT(" R ALL: ");
		PCF8574_DEBUG_PRINT(readMode, BIN);

		PCF8574_DEBUG_PRINT(" R Down: ");
		PCF8574_DEBUG_PRINT(readModePullDown, BIN);
		PCF8574_DEBUG_PRINT("R Up: ");
		PCF8574_DEBUG_PRINTLN(readModePullUp, BIN);
	}else if (mode == INPUT_PULLUP){
		writeMode = writeMode & ~bit(pin);

		readMode 			=   readMode 			| bit(pin);
		readModePullDown 	=	readModePullDown 	& ~bit(pin);
		readModePullUp 		=	readModePullUp 		| bit(pin);

		PCF8574_DEBUG_PRINT("W: ");
		PCF8574_DEBUG_PRINT(writeMode, BIN);
		PCF8574_DEBUG_PRINT(" R ALL: ");
		PCF8574_DEBUG_PRINT(readMode, BIN);

		PCF8574_DEBUG_PRINT(" R Down: ");
		PCF8574_DEBUG_PRINT(readModePullDown, BIN);
		PCF8574_DEBUG_PRINT("R Up: ");
		PCF8574_DEBUG_PRINTLN(readModePullUp, BIN);
	}
	else{
		PCF8574_DEBUG_PRINTLN("Mode non supported by PCF8574")
	}
};


void PCF8574::encoder(uint8_t pinA, uint8_t pinB){
	PCF8574::pinMode(pinA, INPUT_PULLUP);
	PCF8574::pinMode(pinB, INPUT_PULLUP);

	encoderPins[pinA] = true;
	encoderPins[pinB] = true;
}

byte getBit(byte n, byte position)
{
   return (n >> position) & 1;
}

//int8_t PCF8574::readEncoderValue(uint8_t pinA, uint8_t pinB){
//	  bool changed = false;
//
//	  byte offset = 0;
//
//	  byte na = PCF8574::digitalRead(pinA);
//	  byte nb = PCF8574::digitalRead(pinB);
//
//	  byte encoderPinALast = (encoderValues & bit(pinA));
//	  byte encoderPinBLast = (encoderValues & bit(pinB));
//
//	  if ((encoderPinALast!=na || encoderPinBLast!=nb) && (encoderPinALast == LOW) && (na == HIGH)) {
//		if (nb == LOW) {
//			offset = - 1;
//			changed = true;
//		} else {
//			offset = + 1;
//			changed = true;
//		}
//	  }
//
//	  encoderValues = (encoderPinALast!=na)?encoderValues ^ bit(pinA):encoderValues;
//	  encoderValues = (encoderPinBLast!=nb)?encoderValues ^ bit(pinB):encoderValues;
//
//	  return offset;
//}

bool PCF8574::checkProgression(byte oldValA, byte oldValB, byte newValA, byte newValB, byte validProgression){
	bool findOldVal = false;
	int posFinded = 0;
	for (int pos = 0; pos<8; pos = pos + 2){
		if ((oldValB == ((validProgression & bit(pos+1))>0?HIGH:LOW)) && (oldValA == ((validProgression & bit(pos+0))>0?HIGH:LOW)) ){
			findOldVal = true;
			posFinded = pos;
		}
	}
	if (!findOldVal) return false;

	posFinded = posFinded + 2;
	if (posFinded>8) posFinded = 0;

	return ((newValB == ((validProgression & bit(posFinded+1))>0?HIGH:LOW)) && (newValA == ((validProgression & bit(posFinded+0))>0?HIGH:LOW)) );
}

#ifdef BASIC_ENCODER_ALGORITHM
	bool PCF8574::readEncoderValue(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation){
		PCF8574::detachInterrupt();

		  bool changed = false;

		  byte na = PCF8574::digitalRead(pinA, true);
		  byte nb = PCF8574::digitalRead(pinB, true);

		  byte encoderPinALast = (this->encoderValues & bit(pinA))>0?HIGH:LOW;
		  byte encoderPinBLast = (this->encoderValues & bit(pinB))>0?HIGH:LOW;

		  PCF8574_DEBUG_PRINT(pinA);
		  PCF8574_DEBUG_PRINT(" TO --> ");
		  PCF8574_DEBUG_PRINT(encoderPinALast);
		  PCF8574_DEBUG_PRINT(encoderPinBLast);
		  PCF8574_DEBUG_PRINT(" - ");
		  PCF8574_DEBUG_PRINT(na);
		  PCF8574_DEBUG_PRINT(nb);
		  PCF8574_DEBUG_PRINTLN();

		  if ((encoderPinALast!=na || encoderPinBLast!=nb) && (encoderPinALast == LOW) && (na == HIGH)) {
//			  bool vCW = checkProgression(encoderPinALast, encoderPinBLast, na, nb, validCW);
//			  bool vCCW = checkProgression(encoderPinALast, encoderPinBLast, na, nb, validCCW);

				if (nb == LOW) {
					*encoderValue = *encoderValue + (!reverseRotation?+1:-1);
					changed = true;
				} else {
					*encoderValue = *encoderValue + (!reverseRotation?-1:+1);
					changed = true;
				}

	//			if (nb == LOW && vCW) {
	//	//			checkCW(encoderPinALast, encoderPinBLast, na, nb);
	//				*encoderValue = *encoderValue - 1;
	//				changed = true;
	//			} else if (vCCW) {
	//				*encoderValue = *encoderValue + 1;
	//				changed = true;
	//			}

		  }

		  this->encoderValues = (encoderPinALast!=na)?this->encoderValues ^ bit(pinA):this->encoderValues;
		  this->encoderValues = (encoderPinBLast!=nb)?this->encoderValues ^ bit(pinB):this->encoderValues;
		  PCF8574::attachInterrupt();

			return changed;
	}
	int8_t PCF8574::readEncoderValue(uint8_t pinA, uint8_t pinB) {
		volatile long encoderValue = 0;
		PCF8574::readEncoderValue(pinA, pinB, &encoderValue);
		return encoderValue;
	}

#endif

#ifdef SEQUENCE_ENCODER_ALGORITHM
	bool PCF8574::readEncoderValueSequence(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation){

		PCF8574::detachInterrupt();
		  bool changed = false;

		  delay(100);
		  byte na = PCF8574::digitalRead(pinA, true);
		  byte nb = PCF8574::digitalRead(pinB, true);

		  byte encoderPinALast = (this->encoderValues & bit(pinA))>0?HIGH:LOW;
		  byte encoderPinBLast = (this->encoderValues & bit(pinB))>0?HIGH:LOW;

		  PCF8574_DEBUG_PRINT(pinA);
		  PCF8574_DEBUG_PRINT(" TO --> ");
		  PCF8574_DEBUG_PRINT(encoderPinALast);
		  PCF8574_DEBUG_PRINT(encoderPinBLast);
		  PCF8574_DEBUG_PRINT(" - ");
		  PCF8574_DEBUG_PRINT(na);
		  PCF8574_DEBUG_PRINT(nb);
		  PCF8574_DEBUG_PRINT(" -- ");

		  int encoded = (na << 1) | nb; //converting the 2 pin value to single number
		  int lastEncoded = (encoderPinALast << 1) | encoderPinBLast;
		  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

		  PCF8574_DEBUG_PRINT("sum - ");
		  PCF8574_DEBUG_PRINT(sum, BIN);

		  PCF8574_DEBUG_PRINT(" enc - ");
		  PCF8574_DEBUG_PRINT( *encoderValue);

			  if(
					  sum == 0b1101
					   || sum == 0b0100
					  || sum == 0b0010
					   || sum == 0b1011
					  ){
	//			  encoderValue ++;
				  *encoderValue = *encoderValue + (!reverseRotation?+1:-1);
				  changed = true;
			  }
			  if(
					  sum == 0b1110
					   || sum == 0b0111
					  || sum == 0b0001
					   || sum == 0b1000
					) {
				  *encoderValue = *encoderValue + (!reverseRotation?-1:+1);
				  changed = true;
	//			  encoderValue --;
			  }

		  PCF8574_DEBUG_PRINT(" enc next - ");
		  PCF8574_DEBUG_PRINTLN( *encoderValue);

		  this->encoderValues = (encoderPinALast!=na)?this->encoderValues ^ bit(pinA):this->encoderValues;
		  this->encoderValues = (encoderPinBLast!=nb)?this->encoderValues ^ bit(pinB):this->encoderValues;
		  PCF8574::attachInterrupt();
			return changed;
	}
	int8_t PCF8574::readEncoderValueSequence(uint8_t pinA, uint8_t pinB) {
		volatile long encoderValue = 0;
		PCF8574::readEncoderValueSequence(pinA, pinB, &encoderValue);
		return encoderValue;
	}

#endif
#ifdef SEQUENCE_ENCODER_ALGORITHM_REDUCED
	bool PCF8574::readEncoderValueSequenceReduced(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation){

		PCF8574::detachInterrupt();
		  bool changed = false;

		  delay(100);
		  byte na = PCF8574::digitalRead(pinA, true);
		  byte nb = PCF8574::digitalRead(pinB, true);

		  byte encoderPinALast = (this->encoderValues & bit(pinA))>0?HIGH:LOW;
		  byte encoderPinBLast = (this->encoderValues & bit(pinB))>0?HIGH:LOW;

		  PCF8574_DEBUG_PRINT(pinA);
		  PCF8574_DEBUG_PRINT(" TO --> ");
		  PCF8574_DEBUG_PRINT(encoderPinALast);
		  PCF8574_DEBUG_PRINT(encoderPinBLast);
		  PCF8574_DEBUG_PRINT(" - ");
		  PCF8574_DEBUG_PRINT(na);
		  PCF8574_DEBUG_PRINT(nb);
		  PCF8574_DEBUG_PRINT(" -- ");

		  int encoded = (na << 1) | nb; //converting the 2 pin value to single number
		  int lastEncoded = (encoderPinALast << 1) | encoderPinBLast;
		  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

		  PCF8574_DEBUG_PRINT("sum - ");
		  PCF8574_DEBUG_PRINT(sum, BIN);

		  PCF8574_DEBUG_PRINT(" enc - ");
		  PCF8574_DEBUG_PRINT( *encoderValue);

			  if(
					  sum == 0b1101
					  // || sum == 0b0100
					  || sum == 0b0010
					  // || sum == 0b1011
					  ){
	//			  encoderValue ++;
				  *encoderValue = *encoderValue + (!reverseRotation?+1:-1);
				  changed = true;
			  }
			  if(
					  sum == 0b1110
					  // || sum == 0b0111
					  || sum == 0b0001
					  // || sum == 0b1000
					) {
				  *encoderValue = *encoderValue + (!reverseRotation?-1:+1);
				  changed = true;
	//			  encoderValue --;
			  }

		  PCF8574_DEBUG_PRINT(" enc next - ");
		  PCF8574_DEBUG_PRINTLN( *encoderValue);

		  this->encoderValues = (encoderPinALast!=na)?this->encoderValues ^ bit(pinA):this->encoderValues;
		  this->encoderValues = (encoderPinBLast!=nb)?this->encoderValues ^ bit(pinB):this->encoderValues;
		  PCF8574::attachInterrupt();
			return changed;
	}
	int8_t PCF8574::readEncoderValueSequenceReduced(uint8_t pinA, uint8_t pinB) {
		volatile long encoderValue = 0;
		PCF8574::readEncoderValueSequenceReduced(pinA, pinB, &encoderValue);
		return encoderValue;
	}

#endif
#ifdef MISCHIANTI_ENCODER_ALGORITHM
	bool PCF8574::readEncoderValueMischianti(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation){
		PCF8574::detachInterrupt();
		  bool changed = false;

		  byte na = PCF8574::digitalRead(pinA, true);
		  byte nb = PCF8574::digitalRead(pinB, true);

		  byte encoderPinALast = (this->encoderValues & bit(pinA))>0?HIGH:LOW;
		  byte encoderPinBLast = (this->encoderValues & bit(pinB))>0?HIGH:LOW;

		  if ((encoderPinALast!=na || encoderPinBLast!=nb) && ((encoderPinALast == LOW) || encoderPinALast==encoderPinBLast) && (na == HIGH)) {
			  PCF8574_DEBUG_PRINT("TO --> ");
			  PCF8574_DEBUG_PRINT(encoderPinALast);
			  PCF8574_DEBUG_PRINT(encoderPinBLast);
			  PCF8574_DEBUG_PRINT(" - ");
			  PCF8574_DEBUG_PRINT(na);
			  PCF8574_DEBUG_PRINT(nb);
			  PCF8574_DEBUG_PRINTLN();

				if (nb == LOW && nb!=na) {
					*encoderValue = *encoderValue + (!reverseRotation?+1:-1);
					changed = true;
				} else if (nb==na && encoderPinALast==encoderPinBLast) {
					*encoderValue = *encoderValue + (!reverseRotation?-1:+1);
					changed = true;
				}
		  }
//		  encoderValues = encoderValues & (~(bit(pinA) | bit(pinB)));
//		  if (na == HIGH){
//			  encoderValues = encoderValues | bit(pinA);
//		  }
//		  if (nb == HIGH){
//			  encoderValues = encoderValues | bit(pinA);
//		  }

		  if (encoderPinALast!=na || encoderPinBLast!=nb){
		  this->encoderValues = (encoderPinALast!=na)?this->encoderValues ^ bit(pinA):this->encoderValues;
		  this->encoderValues = (encoderPinBLast!=nb)?this->encoderValues ^ bit(pinB):this->encoderValues;
		  }

		  PCF8574::attachInterrupt();
			return changed;
	}
	int8_t PCF8574::readEncoderValueMischianti(uint8_t pinA, uint8_t pinB) {
		volatile long encoderValue = 0;
		PCF8574::readEncoderValueMischianti(pinA, pinB, &encoderValue);
		return encoderValue;
	}

#endif
//#ifdef MISCHIANTI_ENCODER_ALGORITHM_EVOLVED
//	bool PCF8574::readEncoderValueEvolved(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation){
//		PCF8574::detachInterrupt();
//		  bool changed = false;
//
//		  byte na = PCF8574::digitalRead(pinA, true);
//		  byte nb = PCF8574::digitalRead(pinB, true);
//
//		  byte encoderPinALast = (this->encoderValues & bit(pinA))>0?HIGH:LOW;
//		  byte encoderPinBLast = (this->encoderValues & bit(pinB))>0?HIGH:LOW;
//
////		  PCF8574_DEBUG_PRINT(pinA);
////		  PCF8574_DEBUG_PRINT(" TO --> ");
////		  PCF8574_DEBUG_PRINT(encoderPinALast);
////		  PCF8574_DEBUG_PRINT(encoderPinBLast);
////		  PCF8574_DEBUG_PRINT(" - ");
////		  PCF8574_DEBUG_PRINT(na);
////		  PCF8574_DEBUG_PRINT(nb);
//
//		  if (
//
//		  ((encoderPinALast!=na || encoderPinBLast!=nb) && ((encoderPinALast == LOW) || encoderPinALast==encoderPinBLast) && (na == HIGH))
//		  || ((encoderPinALast!=na || encoderPinBLast!=nb) && ((encoderPinALast == HIGH) || encoderPinALast==encoderPinBLast) && (na == LOW))
//			){
//			  PCF8574_DEBUG_PRINT("TO --> ");
//			  PCF8574_DEBUG_PRINT(encoderPinALast);
//			  PCF8574_DEBUG_PRINT(encoderPinBLast);
//			  PCF8574_DEBUG_PRINT(" - ");
//			  PCF8574_DEBUG_PRINT(na);
//			  PCF8574_DEBUG_PRINT(nb);
//			  PCF8574_DEBUG_PRINTLN();
////
////			  PCF8574_DEBUG_PRINT (" <------ ");
//
//				if (nb == LOW && nb!=na) {
//					*encoderValue = *encoderValue + (!reverseRotation?+1:-1);
//					changed = true;
//				} else if (nb==na && encoderPinALast==encoderPinBLast) {
//					*encoderValue = *encoderValue + (!reverseRotation?-1:+1);
//					changed = true;
//				}
//		  }
////		  PCF8574_DEBUG_PRINTLN();
////		  encoderValues = encoderValues & (~(bit(pinA) | bit(pinB)));
////		  if (na == HIGH){
////			  encoderValues = encoderValues | bit(pinA);
////		  }
////		  if (nb == HIGH){
////			  encoderValues = encoderValues | bit(pinA);
////		  }
//
//		  if (encoderPinALast!=na || encoderPinBLast!=nb){
//		  this->encoderValues = (encoderPinALast!=na)?this->encoderValues ^ bit(pinA):this->encoderValues;
//		  this->encoderValues = (encoderPinBLast!=nb)?this->encoderValues ^ bit(pinB):this->encoderValues;
//		  }
//
//		  PCF8574::attachInterrupt();
//			return changed;
//	}
//	int8_t PCF8574::readEncoderValueEvolved(uint8_t pinA, uint8_t pinB) {
//		volatile long encoderValue = 0;
//		PCF8574::readEncoderValueEvolved(pinA, pinB, &encoderValue);
//		return encoderValue;
//	}
//
//#endif

#ifdef POKI_ENCODER_ALGORITHM
	bool PCF8574::readEncoderValuePoki(uint8_t pinA, uint8_t pinB, volatile long *encoderValue, bool reverseRotation){
		  PCF8574::detachInterrupt();

		  bool changed = false;

		  byte na = PCF8574::digitalRead(pinA, true);
		  byte nb = PCF8574::digitalRead(pinB, true);

		  byte encoderPinALast = (this->encoderValues & bit(pinA))>0?HIGH:LOW;
		  byte encoderPinBLast = (this->encoderValues & bit(pinB))>0?HIGH:LOW;

		  PCF8574_DEBUG_PRINT("TO --> ");
		  PCF8574_DEBUG_PRINT(encoderPinALast);
		  PCF8574_DEBUG_PRINT(encoderPinBLast);
		  PCF8574_DEBUG_PRINT(" - ");
		  PCF8574_DEBUG_PRINT(na);
		  PCF8574_DEBUG_PRINT(nb);
		  PCF8574_DEBUG_PRINTLN();

		  if ((encoderPinALast!=na || encoderPinBLast!=nb) && ((encoderPinALast == LOW) || encoderPinALast==encoderPinBLast) && (na == HIGH)) {
			  PCF8574_DEBUG_PRINT("TO --> ");
			  PCF8574_DEBUG_PRINT(encoderPinALast);
			  PCF8574_DEBUG_PRINT(encoderPinBLast);
			  PCF8574_DEBUG_PRINT(" - ");
			  PCF8574_DEBUG_PRINT(na);
			  PCF8574_DEBUG_PRINT(nb);
			  PCF8574_DEBUG_PRINTLN();

			  if (na && !nb) {
					if (encoderPinBLast) {
						*encoderValue = *encoderValue + (!reverseRotation?+1:-1);
					} else {
						*encoderValue = *encoderValue  + (!reverseRotation?-1:+1);
					}
					changed = true;
				}
		  }

		  this->encoderValues = (encoderPinALast!=na)?this->encoderValues ^ bit(pinA):encoderValues;
		  this->encoderValues = (encoderPinBLast!=nb)?this->encoderValues ^ bit(pinB):encoderValues;
		  PCF8574::attachInterrupt();

			return changed;
	}
	int8_t PCF8574::readEncoderValuePoki(uint8_t pinA, uint8_t pinB) {
		volatile long encoderValue = 0;
		PCF8574::readEncoderValue(pinA, pinB, &encoderValue);
		return encoderValue;
	}

#endif


/**
 * Read value from i2c and bufferize it
 * @param force
 */
void PCF8574::readBuffer(bool force){
//	if (millis() > PCF8574::lastReadMillis+latency || _usingInterrupt || force){
	if (millis() - PCF8574::lastReadMillis > latency || _usingInterrupt || force){
		_wire->requestFrom(_address,(uint8_t)1);// Begin transmission to PCF8574 with the buttons
		lastReadMillis = millis();
		if(_wire->available())   // If bytes are available to be recieved
		{
			byte iInput = _wire->read();// Read a byte
			  if ((iInput & readModePullDown)>0 and (~iInput & readModePullUp)>0){
//			  if ((iInput & readMode)>0){
				byteBuffered = (byteBuffered & ~readMode) | (byte)iInput;
			}
		}
	}
}

#ifndef PCF8574_LOW_MEMORY
	/**
	 * Read value of all INPUT pin
	 * Debounce read more fast than 10millis, non managed for interrupt mode
	 * @return
	 */
	PCF8574::DigitalInput PCF8574::digitalReadAll(void){
		PCF8574_DEBUG_PRINTLN("Read from buffer");
		_wire->requestFrom(_address,(uint8_t)1);// Begin transmission to PCF8574 with the buttons
		lastReadMillis = millis();
		if(_wire->available())   // If bytes are available to be recieved
		{
			  PCF8574_DEBUG_PRINTLN("Data ready");
			  byte iInput = _wire->read();// Read a byte

			  if ((readModePullDown & iInput)>0 or (readModePullUp & ~iInput)>0){
				  PCF8574_DEBUG_PRINT(" -------- CHANGE --------- ");
				  byteBuffered = (byteBuffered & ~readMode) | (byte)iInput;
			  }
		}

		PCF8574_DEBUG_PRINT("Buffer value ");
		PCF8574_DEBUG_PRINTLN(byteBuffered, BIN);

		if ((bit(0) & readMode)>0) digitalInput.p0 = ((byteBuffered & bit(0))>0)?HIGH:LOW;
		if ((bit(1) & readMode)>0) digitalInput.p1 = ((byteBuffered & bit(1))>0)?HIGH:LOW;
		if ((bit(2) & readMode)>0) digitalInput.p2 = ((byteBuffered & bit(2))>0)?HIGH:LOW;
		if ((bit(3) & readMode)>0) digitalInput.p3 = ((byteBuffered & bit(3))>0)?HIGH:LOW;
		if ((bit(4) & readMode)>0) digitalInput.p4 = ((byteBuffered & bit(4))>0)?HIGH:LOW;
		if ((bit(5) & readMode)>0) digitalInput.p5 = ((byteBuffered & bit(5))>0)?HIGH:LOW;
		if ((bit(6) & readMode)>0) digitalInput.p6 = ((byteBuffered & bit(6))>0)?HIGH:LOW;
		if ((bit(7) & readMode)>0) digitalInput.p7 = ((byteBuffered & bit(7))>0)?HIGH:LOW;

		if ((bit(0) & writeMode)>0) digitalInput.p0 = ((writeByteBuffered & bit(0))>0)?HIGH:LOW;
		if ((bit(1) & writeMode)>0) digitalInput.p1 = ((writeByteBuffered & bit(1))>0)?HIGH:LOW;
		if ((bit(2) & writeMode)>0) digitalInput.p2 = ((writeByteBuffered & bit(2))>0)?HIGH:LOW;
		if ((bit(3) & writeMode)>0) digitalInput.p3 = ((writeByteBuffered & bit(3))>0)?HIGH:LOW;
		if ((bit(4) & writeMode)>0) digitalInput.p4 = ((writeByteBuffered & bit(4))>0)?HIGH:LOW;
		if ((bit(5) & writeMode)>0) digitalInput.p5 = ((writeByteBuffered & bit(5))>0)?HIGH:LOW;
		if ((bit(6) & writeMode)>0) digitalInput.p6 = ((writeByteBuffered & bit(6))>0)?HIGH:LOW;
		if ((bit(7) & writeMode)>0) digitalInput.p7 = ((writeByteBuffered & bit(7))>0)?HIGH:LOW;

		//if ((byteBuffered & readModePullDown)>0 and (~byteBuffered & readModePullUp)>0){

//		byteBuffered = (resetInitial & readMode) | (byteBuffered  & ~readMode); //~readMode & byteBuffered;

		byteBuffered = (initialBuffer & readMode) | (byteBuffered  & ~readMode); //~readMode & byteBuffered;

			PCF8574_DEBUG_PRINT("Buffer hight value readed set readed ");
			PCF8574_DEBUG_PRINTLN(byteBuffered, BIN);
		//}
		PCF8574_DEBUG_PRINT("Return value ");
		return digitalInput;
	};
#else
	/**
	 * Read value of all INPUT pin in byte format for low memory usage
	 * Debounce read more fast than 10millis, non managed for interrupt mode
	 * @return
	 */
	byte PCF8574::digitalReadAll(void){
		PCF8574_DEBUG_PRINTLN("Read from buffer");
		_wire->requestFrom(_address,(uint8_t)1);// Begin transmission to PCF8574 with the buttons
		lastReadMillis = millis();
		if(_wire->available())   // If bytes are available to be recieved
		{
			  PCF8574_DEBUG_PRINTLN("Data ready");
			  byte iInput = _wire->read();// Read a byte

			  if ((readModePullDown & iInput)>0 or (readModePullUp & ~iInput)>0){
				  PCF8574_DEBUG_PRINT(" -------- CHANGE --------- ");
				  byteBuffered = (byteBuffered & ~readMode) | (byte)iInput;

			  }
		}

		PCF8574_DEBUG_PRINT("Buffer value ");
		PCF8574_DEBUG_PRINTLN(byteBuffered, BIN);

		byte byteRead = byteBuffered | writeByteBuffered;

		//if ((byteBuffered & readModePullDown)>0 and (~byteBuffered & readModePullUp)>0){
//			byteBuffered = (resetInitial & readMode) | (byteBuffered  & ~readMode); //~readMode & byteBuffered;
		byteBuffered = (initialBuffer & readMode) | (byteBuffered  & ~readMode); //~readMode & byteBuffered;
			PCF8574_DEBUG_PRINT("Buffer hight value readed set readed ");
			PCF8574_DEBUG_PRINTLN(byteBuffered, BIN);
		//}
		PCF8574_DEBUG_PRINT("Return value ");
		return byteRead;
	};
#endif

/**
 * Read value of specified pin
 * Debounce read more fast than 10millis, non managed for interrupt mode
 * @param pin
 * @return
 */
uint8_t PCF8574::digitalRead(uint8_t pin, bool forceReadNow){
	uint8_t value = (bit(pin) & readModePullUp)?HIGH:LOW;
	PCF8574_DEBUG_PRINT("Read pin ");
	PCF8574_DEBUG_PRINT (pin);
	// Check if pin already HIGH than read and prevent reread of i2c
//	PCF8574_DEBUG_PRINTLN("----------------------------------")
//	PCF8574_DEBUG_PRINT("readModePullUp   ");
//	PCF8574_DEBUG_PRINTLN(readModePullUp, BIN);
//	PCF8574_DEBUG_PRINT("readModePullDown ");
//	PCF8574_DEBUG_PRINTLN(readModePullDown, BIN);
//	PCF8574_DEBUG_PRINT("byteBuffered     ");
//	PCF8574_DEBUG_PRINTLN(byteBuffered, BIN);


	if ((((bit(pin) & (readModePullDown & byteBuffered))>0) or (bit(pin) & (readModePullUp & ~byteBuffered))>0 )){
		PCF8574_DEBUG_PRINTLN(" ...Pin already set");
		  if ((bit(pin) & byteBuffered)>0){
			  value = HIGH;
		  }else{
			  value = LOW;
		  }
//	 }else if (forceReadNow || (millis() > PCF8574::lastReadMillis+latency)){
	 }else if (forceReadNow || (millis() - PCF8574::lastReadMillis > latency)){
		 PCF8574_DEBUG_PRINT(" ...Read from buffer... ");
		  _wire->requestFrom(_address,(uint8_t)1);// Begin transmission to PCF8574 with the buttons
		  lastReadMillis = millis();
		  if(_wire->available())   // If bytes are available to be recieved
		  {
			  PCF8574_DEBUG_PRINTLN(" Data ready");
			  byte iInput = _wire->read();// Read a byte
			  PCF8574_DEBUG_PRINT("Input ");
			  PCF8574_DEBUG_PRINT((byte)iInput, BIN);

			  if ((readModePullDown & iInput)>0 or (readModePullUp & ~iInput)>0){
				  PCF8574_DEBUG_PRINT(" -------- CHANGE --------- ");
				  byteBuffered = (byteBuffered & ~readMode) | (byte)iInput;
				  if ((bit(pin) & byteBuffered)>0){
					  value = HIGH;
				  }else{
					  value = LOW;
				  }
//				  value = (bit(pin) & byteBuffered);
			  }
		  }
	}
	PCF8574_DEBUG_PRINT(" ..Buffer value ");
	PCF8574_DEBUG_PRINT(byteBuffered, BIN);
	// If HIGH set to low to read buffer only one time
	if ((bit(pin) & readModePullDown) and value==HIGH){
		byteBuffered = bit(pin) ^ byteBuffered;
		PCF8574_DEBUG_PRINT(" ...Buffer hight value readed set readed ");
		PCF8574_DEBUG_PRINT (byteBuffered, BIN);
	}else if ((bit(pin) & readModePullUp) and value==LOW){
		byteBuffered = bit(pin) ^ byteBuffered;
		PCF8574_DEBUG_PRINT(" ...Buffer low value readed set readed ");
		PCF8574_DEBUG_PRINT(byteBuffered, BIN);
	}else if(bit(pin) & writeByteBuffered){
		value = HIGH;
	}
	PCF8574_DEBUG_PRINT(" ...Return value ");
	PCF8574_DEBUG_PRINTLN(value);
	return value;
};

/**
 * Write on pin
 * @param pin
 * @param value
 */
bool PCF8574::digitalWrite(uint8_t pin, uint8_t value){
	PCF8574_DEBUG_PRINTLN("Begin trasmission");
	_wire->beginTransmission(_address);     //Begin the transmission to PCF8574
	PCF8574_DEBUG_PRINT("Value ");
	PCF8574_DEBUG_PRINT(value);
	PCF8574_DEBUG_PRINT(" Write data pre ");
	PCF8574_DEBUG_PRINT(writeByteBuffered, BIN);

	if (value==HIGH){
		writeByteBuffered = writeByteBuffered | bit(pin);
		byteBuffered  = writeByteBuffered | bit(pin);
	}else{
		writeByteBuffered = writeByteBuffered & ~bit(pin);
		byteBuffered  = writeByteBuffered & ~bit(pin);
	}
	PCF8574_DEBUG_PRINT("Write data ");
	PCF8574_DEBUG_PRINT(writeByteBuffered, BIN);
	PCF8574_DEBUG_PRINT(" for pin ");
	PCF8574_DEBUG_PRINT(pin);
	PCF8574_DEBUG_PRINT(" bin value ");
	PCF8574_DEBUG_PRINT(bit(pin), BIN);
	PCF8574_DEBUG_PRINT(" value ");
	PCF8574_DEBUG_PRINT(value);

	// writeByteBuffered = writeByteBuffered & (~writeMode & byteBuffered);
	byteBuffered = (writeByteBuffered & writeMode) | (resetInitial & readMode);

	// byteBuffered = (writeByteBuffered & writeMode) | (byteBuffered & readMode);
	PCF8574_DEBUG_PRINT(" byteBuffered ");
	PCF8574_DEBUG_PRINTLN(byteBuffered, BIN);

	PCF8574_DEBUG_PRINT("Going to write data ");
	PCF8574_DEBUG_PRINTLN(writeByteBuffered, BIN);

	_wire->write(byteBuffered);

	byteBuffered = (writeByteBuffered & writeMode) | (initialBuffer & readMode);

//	byteBuffered = (writeByteBuffered & writeMode) & (byteBuffered & readMode);
	PCF8574_DEBUG_PRINTLN("Start end trasmission if stop here check pullup resistor.");

	this->transmissionStatus = _wire->endTransmission();

	return this->isLastTransmissionSuccess();
};

#ifndef PCF8574_LOW_MEMORY
	/**
	 * Read value of all INPUT pin
	 * Debounce read more fast than 10millis, non managed for interrupt mode
	 * @return
	 */
	void PCF8574::setVal(uint8_t pin, uint8_t value){
		if (value==HIGH){
			writeByteBuffered = writeByteBuffered | bit(pin);
			byteBuffered  = writeByteBuffered | bit(pin);
		}else{
			writeByteBuffered = writeByteBuffered & ~bit(pin);
			byteBuffered  = writeByteBuffered & ~bit(pin);
		}

	}
	bool PCF8574::digitalWriteAll(PCF8574::DigitalInput digitalInput){

		setVal(P0, digitalInput.p0);
		setVal(P1, digitalInput.p1);
		setVal(P2, digitalInput.p2);
		setVal(P3, digitalInput.p3);
		setVal(P4, digitalInput.p4);
		setVal(P5, digitalInput.p5);
		setVal(P6, digitalInput.p6);
		setVal(P7, digitalInput.p7);

		return digitalWriteAllBytes(writeByteBuffered);
	}
#else
	bool PCF8574::digitalWriteAll(byte digitalInput){
		return digitalWriteAllBytes(digitalInput);
	}
#endif


bool PCF8574::digitalWriteAllBytes(byte allpins){
	_wire->beginTransmission(_address);     //Begin the transmission to PCF8574

	// writeByteBuffered = writeByteBuffered & (~writeMode & byteBuffered);
	writeByteBuffered = allpins;
	byteBuffered = (writeByteBuffered & writeMode) | (resetInitial & readMode);

	// byteBuffered = (writeByteBuffered & writeMode) | (byteBuffered & readMode);
	PCF8574_DEBUG_PRINT(" byteBuffered ");
	PCF8574_DEBUG_PRINTLN(byteBuffered, BIN);

	PCF8574_DEBUG_PRINT("Going to write data ");
	PCF8574_DEBUG_PRINTLN(writeByteBuffered, BIN);

	_wire->write(byteBuffered);

	byteBuffered = (writeByteBuffered & writeMode) | (initialBuffer & readMode);

//	byteBuffered = (writeByteBuffered & writeMode) & (byteBuffered & readMode);
	PCF8574_DEBUG_PRINTLN("Start end trasmission if stop here check pullup resistor.");

	this->transmissionStatus = _wire->endTransmission();

	return this->isLastTransmissionSuccess();

}

/**
 * Measure length (in microseconds) of a pulse on the specified pin.
 * Similar semantics to Arduino's pulseIn, but uses PCF8574 digitalRead that can force an immediate read.
 * @param pin: PCF8574 pin (0..7)
 * @param state: HIGH or LOW - the state to measure the duration of
 * @param timeout: timeout in microseconds
 * @return pulse length in microseconds, 0 on timeout
 */
unsigned long PCF8574::pulseIn(uint8_t pin, uint8_t state, unsigned long timeout){
    unsigned long startMicros = micros();
    unsigned long timeoutTime = startMicros + timeout;

    // Wait for any previous pulse to end
    while (digitalRead(pin, true) == state) {
        if (micros() > timeoutTime) return 0;
    }

    // Wait for the pulse to start
    while (digitalRead(pin, true) != state) {
        if (micros() > timeoutTime) return 0;
    }
    unsigned long pulseStart = micros();

    // Wait for the pulse to end
    while (digitalRead(pin, true) == state) {
        if (micros() > timeoutTime) return 0;
    }
    unsigned long pulseEnd = micros();

    return pulseEnd - pulseStart;
}

/**
 * pulseInPoll: like pulseIn but polls at a configurable interval to reduce I2C traffic.
 * pollIntervalMicros: time between actual reads from PCF8574. Larger -> fewer I2C reads, less precision.
 */
unsigned long PCF8574::pulseInPoll(uint8_t pin, uint8_t state, unsigned long timeout, unsigned int pollIntervalMicros){
    unsigned long startMicros = micros();
    unsigned long timeoutTime = startMicros + timeout;

    // Wait for any previous pulse to end
    while (true){
        if (digitalRead(pin, true) != state) break;
        if (micros() > timeoutTime) return 0;
        if (pollIntervalMicros > 0) delayMicroseconds(pollIntervalMicros);
    }

    // Wait for the pulse to start
    while (true){
        if (digitalRead(pin, true) == state) break;
        if (micros() > timeoutTime) return 0;
        if (pollIntervalMicros > 0) delayMicroseconds(pollIntervalMicros);
    }
    unsigned long pulseStart = micros();

    // Wait for the pulse to end
    while (true){
        if (digitalRead(pin, true) != state) break;
        if (micros() > timeoutTime) return 0;
        if (pollIntervalMicros > 0) delayMicroseconds(pollIntervalMicros);
    }
    unsigned long pulseEnd = micros();

    return pulseEnd - pulseStart;
}

/**
 * Ultrasonic sensor support (HC-SR04 compatible)
 * Send a trigger pulse and measure echo response duration
 * @param trigPin: PCF8574 pin connected to HC-SR04 TRIG
 * @param echoPin: PCF8574 pin connected to HC-SR04 ECHO
 * @param maxDistance_cm: maximum distance in cm (default 500cm)
 * @return pulse duration in microseconds, 0 on timeout
 */
unsigned long PCF8574::ping(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm){
    // Calculate timeout based on max distance
    // Speed of sound: ~343 m/s or 29 microseconds per cm (round trip)
    unsigned long timeout = maxDistance_cm * 29UL * 2UL + 1000UL; // Add 1ms safety margin

    // Send trigger pulse: LOW for 2us, HIGH for 10us, then LOW
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure echo pulse duration
    return pulseIn(echoPin, HIGH, timeout);
}

/**
 * Ultrasonic ping with polling to reduce I2C traffic
 * @param trigPin: PCF8574 pin connected to HC-SR04 TRIG
 * @param echoPin: PCF8574 pin connected to HC-SR04 ECHO
 * @param maxDistance_cm: maximum distance in cm (default 500cm)
 * @param pollIntervalMicros: polling interval in microseconds (default 100us)
 * @return pulse duration in microseconds, 0 on timeout
 */
unsigned long PCF8574::pingPoll(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm, unsigned int pollIntervalMicros){
    // Calculate timeout based on max distance
    unsigned long timeout = maxDistance_cm * 29UL * 2UL + 1000UL;

    // Send trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure echo pulse duration with polling
    return pulseInPoll(echoPin, HIGH, timeout, pollIntervalMicros);
}

/**
 * Get distance in centimeters
 * @param trigPin: PCF8574 pin connected to HC-SR04 TRIG
 * @param echoPin: PCF8574 pin connected to HC-SR04 ECHO
 * @param maxDistance_cm: maximum distance in cm (default 500cm)
 * @return distance in centimeters, 0 on timeout
 */
unsigned long PCF8574::ping_cm(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm){
    unsigned long duration = ping(trigPin, echoPin, maxDistance_cm);
    return microsecondsToDistance_cm(duration);
}

/**
 * Get distance in centimeters with polling
 * @param trigPin: PCF8574 pin connected to HC-SR04 TRIG
 * @param echoPin: PCF8574 pin connected to HC-SR04 ECHO
 * @param maxDistance_cm: maximum distance in cm (default 500cm)
 * @param pollIntervalMicros: polling interval in microseconds (default 100us)
 * @return distance in centimeters, 0 on timeout
 */
unsigned long PCF8574::ping_cm_poll(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm, unsigned int pollIntervalMicros){
    unsigned long duration = pingPoll(trigPin, echoPin, maxDistance_cm, pollIntervalMicros);
    return microsecondsToDistance_cm(duration);
}

/**
 * Get distance in inches
 * @param trigPin: PCF8574 pin connected to HC-SR04 TRIG
 * @param echoPin: PCF8574 pin connected to HC-SR04 ECHO
 * @param maxDistance_cm: maximum distance in cm (default 500cm)
 * @return distance in inches, 0 on timeout
 */
unsigned long PCF8574::ping_in(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm){
    unsigned long duration = ping(trigPin, echoPin, maxDistance_cm);
    return microsecondsToDistance_in(duration);
}

/**
 * Get distance in inches with polling
 * @param trigPin: PCF8574 pin connected to HC-SR04 TRIG
 * @param echoPin: PCF8574 pin connected to HC-SR04 ECHO
 * @param maxDistance_cm: maximum distance in cm (default 500cm)
 * @param pollIntervalMicros: polling interval in microseconds (default 100us)
 * @return distance in inches, 0 on timeout
 */
unsigned long PCF8574::ping_in_poll(uint8_t trigPin, uint8_t echoPin, unsigned long maxDistance_cm, unsigned int pollIntervalMicros){
    unsigned long duration = pingPoll(trigPin, echoPin, maxDistance_cm, pollIntervalMicros);
    return microsecondsToDistance_in(duration);
}

/**
 * Get median distance from multiple samples for more stable readings
 * @param trigPin: PCF8574 pin connected to HC-SR04 TRIG
 * @param echoPin: PCF8574 pin connected to HC-SR04 ECHO
 * @param iterations: number of samples to take (default 5)
 * @param maxDistance_cm: maximum distance in cm (default 500cm)
 * @return median distance in centimeters, 0 on timeout
 */
unsigned long PCF8574::ping_median(uint8_t trigPin, uint8_t echoPin, uint8_t iterations, unsigned long maxDistance_cm){
    if (iterations == 0) iterations = 1;
    if (iterations > 10) iterations = 10; // Limit to reasonable number

    unsigned long distances[10];

    // Collect samples
    for (uint8_t i = 0; i < iterations; i++){
        distances[i] = ping_cm(trigPin, echoPin, maxDistance_cm);
        if (i < iterations - 1) delay(30); // Wait between samples (HC-SR04 needs ~60ms cycle)
    }

    // Simple bubble sort
    for (uint8_t i = 0; i < iterations - 1; i++){
        for (uint8_t j = 0; j < iterations - i - 1; j++){
            if (distances[j] > distances[j + 1]){
                unsigned long temp = distances[j];
                distances[j] = distances[j + 1];
                distances[j + 1] = temp;
            }
        }
    }

    // Return median
    return distances[iterations / 2];
}

/**
 * Get median distance with polling to reduce I2C traffic
 * @param trigPin: PCF8574 pin connected to HC-SR04 TRIG
 * @param echoPin: PCF8574 pin connected to HC-SR04 ECHO
 * @param iterations: number of samples to take (default 5)
 * @param maxDistance_cm: maximum distance in cm (default 500cm)
 * @param pollIntervalMicros: polling interval in microseconds (default 100us)
 * @return median distance in centimeters, 0 on timeout
 */
unsigned long PCF8574::ping_median_poll(uint8_t trigPin, uint8_t echoPin, uint8_t iterations, unsigned long maxDistance_cm, unsigned int pollIntervalMicros){
    if (iterations == 0) iterations = 1;
    if (iterations > 10) iterations = 10;

    unsigned long distances[10];

    // Collect samples
    for (uint8_t i = 0; i < iterations; i++){
        distances[i] = ping_cm_poll(trigPin, echoPin, maxDistance_cm, pollIntervalMicros);
        if (i < iterations - 1) delay(30);
    }

    // Simple bubble sort
    for (uint8_t i = 0; i < iterations - 1; i++){
        for (uint8_t j = 0; j < iterations - i - 1; j++){
            if (distances[j] > distances[j + 1]){
                unsigned long temp = distances[j];
                distances[j] = distances[j + 1];
                distances[j + 1] = temp;
            }
        }
    }

    // Return median
    return distances[iterations / 2];
}

/**
 * Convert microseconds to centimeters
 * Sound travels at ~343 m/s or 29 microseconds per cm (round trip)
 * @param microseconds: pulse duration in microseconds
 * @return distance in centimeters
 */
unsigned long PCF8574::microsecondsToDistance_cm(unsigned long microseconds){
    if (microseconds == 0) return 0;
    return microseconds / 29 / 2;
}

/**
 * Convert microseconds to inches
 * Sound travels at ~343 m/s or 74 microseconds per inch (round trip)
 * @param microseconds: pulse duration in microseconds
 * @return distance in inches
 */
unsigned long PCF8574::microsecondsToDistance_in(unsigned long microseconds){
    if (microseconds == 0) return 0;
    return microseconds / 74 / 2;
}

#ifdef PCF8574_DEBUG
// Print a human-readable description for BeginResult values.
void PCF8574::printBeginResult(BeginResult result, bool verbose){
    switch(result){
        case BeginResult::OK:
            PCF8574_DEBUG_PRINTLN(F("PCF8574 initialization: OK"));
            break;
        case BeginResult::I2C_ERROR:
            PCF8574_DEBUG_PRINTLN(F("ERROR: I2C communication failed (no ACK)."));
            if (verbose){
                PCF8574_DEBUG_PRINTLN(F("  Check: wiring, I2C address (A0/A1/A2), SDA/SCL connections and power supply."));
            }
            break;
        case BeginResult::NO_PINS_CONFIGURED:
            PCF8574_DEBUG_PRINTLN(F("WARNING: begin() called but no PCF8574 pins were configured with pinMode()."));
            if (verbose){
                PCF8574_DEBUG_PRINTLN(F("  Call pcf.pinMode(pin, mode) for needed pins before calling begin()."));
            }
            break;
        case BeginResult::INVALID_ADDRESS:
            PCF8574_DEBUG_PRINTLN(F("ERROR: Invalid I2C address specified for PCF8574."));
            if (verbose){
                PCF8574_DEBUG_PRINTLN(F("  Ensure the address matches hardware (A0,A1,A2 strapping) and use 0x20-0x27 or 0x38-0x3F."));
            }
            break;
        default:
            PCF8574_DEBUG_PRINTLN(F("ERROR: Unknown beginResult status."));
            break;
    }
}
#endif

