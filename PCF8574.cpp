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
    uint8_t err = wire->endTransmission();
    return (err == 0);
}

BeginResult PCF8574::beginResult(){
    // Probe I2C first
    if (!probeI2CDevice(_wire, _address)){
        return BeginResult::I2C_ERROR;
    }

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
#endif

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

	//		DEBUG_PRINTLN("Using interrupt pin (not all pin is interrupted)");
	//		::pinMode(_interruptPin, INPUT_PULLUP);
	//		attachInterrupt(digitalPinToInterrupt(_interruptPin), (*_interruptFunction), FALLING );
			DEBUG_PRINTLN("Using interrupt pin (not all pin is interrupted)");
			::pinMode(_interruptPin, INPUT_PULLUP);
			::attachInterrupt(digitalPinToInterrupt(_interruptPin), (*_interruptFunction), FALLING );
		}

	}
	void PCF8574::detachInterrupt(){
		// If using interrupt set interrupt value to pin
		if (_usingInterrupt){
			::detachInterrupt(digitalPinToInterrupt(_interruptPin));
			DEBUG_PRINTLN("Detach interrupt pin");
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
		DEBUG_PRINT(F("begin(sda, scl) -> "));DEBUG_PRINT(_sda);DEBUG_PRINT(F(" "));DEBUG_PRINTLN(_scl);
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
		DEBUG_PRINTLN("Set write mode");
		_wire->beginTransmission(_address);


		DEBUG_PRINT("resetInitial pin ");
#ifdef PCF8574_SOFT_INITIALIZATION
		resetInitial = writeModeUp | readModePullUp;
#else
		resetInitial = writeModeUp | readMode;
#endif
		DEBUG_PRINTLN( resetInitial, BIN);

		_wire->write(resetInitial);

		initialBuffer = writeModeUp | readModePullUp;
		byteBuffered = initialBuffer;
		writeByteBuffered = writeModeUp;

		DEBUG_PRINTLN("Start end trasmission if stop here check pullup resistor.");
		this->transmissionStatus = _wire->endTransmission();
	}

//	// If using interrupt set interrupt value to pin
//	if (_usingInterrupt){
////		DEBUG_PRINTLN("Using interrupt pin (not all pin is interrupted)");
////		::pinMode(_interruptPin, INPUT_PULLUP);
////		attachInterrupt(digitalPinToInterrupt(_interruptPin), (*_interruptFunction), FALLING );
//		DEBUG_PRINTLN("Using interrupt pin (not all pin is interrupted)");
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
	DEBUG_PRINT("Set pin ");
	DEBUG_PRINT(pin);
	DEBUG_PRINT(" as ");
	DEBUG_PRINTLN(mode);

	if (mode == OUTPUT){
		writeMode = writeMode | bit(pin);
		if (output_start==HIGH) {
			writeModeUp = writeModeUp | bit(pin);
		}

		readMode =  readMode & ~bit(pin);
		readModePullDown 	=	readModePullDown 	& 	~bit(pin);
		readModePullUp 		=	readModePullUp 		& 	~bit(pin);

		DEBUG_PRINT("W: ");
		DEBUG_PRINT(writeMode, BIN);
		DEBUG_PRINT(" R ALL: ");
		DEBUG_PRINT(readMode, BIN);

		DEBUG_PRINT(" R Down: ");
		DEBUG_PRINT(readModePullDown, BIN);
		DEBUG_PRINT("R Up: ");
		DEBUG_PRINTLN(readModePullUp, BIN);

	}else if (mode == INPUT){
		writeMode = writeMode & ~bit(pin);

		readMode 			=   readMode 			| bit(pin);
		readModePullDown 	=	readModePullDown 	| bit(pin);
		readModePullUp 		=	readModePullUp 		& ~bit(pin);

		DEBUG_PRINT("W: ");
		DEBUG_PRINT(writeMode, BIN);
		DEBUG_PRINT(" R ALL: ");
		DEBUG_PRINT(readMode, BIN);

		DEBUG_PRINT(" R Down: ");
		DEBUG_PRINT(readModePullDown, BIN);
		DEBUG_PRINT("R Up: ");
		DEBUG_PRINTLN(readModePullUp, BIN);
	}else if (mode == INPUT_PULLUP){
		writeMode = writeMode & ~bit(pin);

		readMode 			=   readMode 			| bit(pin);
		readModePullDown 	=	readModePullDown 	& ~bit(pin);
		readModePullUp 		=	readModePullUp 		| bit(pin);

		DEBUG_PRINT("W: ");
		DEBUG_PRINT(writeMode, BIN);
		DEBUG_PRINT(" R ALL: ");
		DEBUG_PRINT(readMode, BIN);

		DEBUG_PRINT(" R Down: ");
		DEBUG_PRINT(readModePullDown, BIN);
		DEBUG_PRINT("R Up: ");
		DEBUG_PRINTLN(readModePullUp, BIN);
	}
	else{
		DEBUG_PRINTLN("Mode non supported by PCF8574")
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

		  DEBUG_PRINT(pinA);
		  DEBUG_PRINT(" TO --> ");
		  DEBUG_PRINT(encoderPinALast);
		  DEBUG_PRINT(encoderPinBLast);
		  DEBUG_PRINT(" - ");
		  DEBUG_PRINT(na);
		  DEBUG_PRINT(nb);
		  DEBUG_PRINTLN();

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

		  DEBUG_PRINT(pinA);
		  DEBUG_PRINT(" TO --> ");
		  DEBUG_PRINT(encoderPinALast);
		  DEBUG_PRINT(encoderPinBLast);
		  DEBUG_PRINT(" - ");
		  DEBUG_PRINT(na);
		  DEBUG_PRINT(nb);
		  DEBUG_PRINT(" -- ");

		  int encoded = (na << 1) | nb; //converting the 2 pin value to single number
		  int lastEncoded = (encoderPinALast << 1) | encoderPinBLast;
		  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

		  DEBUG_PRINT("sum - ");
		  DEBUG_PRINT(sum, BIN);

		  DEBUG_PRINT(" enc - ");
		  DEBUG_PRINT( *encoderValue);

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

		  DEBUG_PRINT(" enc next - ");
		  DEBUG_PRINTLN( *encoderValue);

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

		  DEBUG_PRINT(pinA);
		  DEBUG_PRINT(" TO --> ");
		  DEBUG_PRINT(encoderPinALast);
		  DEBUG_PRINT(encoderPinBLast);
		  DEBUG_PRINT(" - ");
		  DEBUG_PRINT(na);
		  DEBUG_PRINT(nb);
		  DEBUG_PRINT(" -- ");

		  int encoded = (na << 1) | nb; //converting the 2 pin value to single number
		  int lastEncoded = (encoderPinALast << 1) | encoderPinBLast;
		  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

		  DEBUG_PRINT("sum - ");
		  DEBUG_PRINT(sum, BIN);

		  DEBUG_PRINT(" enc - ");
		  DEBUG_PRINT( *encoderValue);

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

		  DEBUG_PRINT(" enc next - ");
		  DEBUG_PRINTLN( *encoderValue);

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
			  DEBUG_PRINT("TO --> ");
			  DEBUG_PRINT(encoderPinALast);
			  DEBUG_PRINT(encoderPinBLast);
			  DEBUG_PRINT(" - ");
			  DEBUG_PRINT(na);
			  DEBUG_PRINT(nb);
			  DEBUG_PRINTLN();

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
//		  if ((encoderPinALast!=na || encoderPinBLast!=nb) && ((encoderPinALast == LOW) || encoderPinALast==encoderPinBLast) && (na == HIGH)) {
//			  DEBUG_PRINT("TO --> ");
//			  DEBUG_PRINT(encoderPinALast);
//			  DEBUG_PRINT(encoderPinBLast);
//			  DEBUG_PRINT(" - ");
//			  DEBUG_PRINT(na);
//			  DEBUG_PRINT(nb);
//			  DEBUG_PRINTLN();
//
//				if (nb == LOW && nb!=na) {
//					*encoderValue = *encoderValue + (!reverseRotation?+1:-1);
//					changed = true;
//				} else if (nb==na && encoderPinALast==encoderPinBLast) {
//					*encoderValue = *encoderValue + (!reverseRotation?-1:+1);
//					changed = true;
//				}
//		  }
//

