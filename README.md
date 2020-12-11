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
#
#
#
#
#

### Additional information and documentation on my site: [pcf8574 Article](https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/).

### If you need more pins [here](https://www.mischianti.org/2019/07/22/pcf8575-i2c-16-bit-digital-i-o-expander/) you can find the pcf8575 16bit version of the IC.

### Version 2.2

Library to use I2C analog IC with arduino and esp8266. Can read and write digital value with only 2 wires (perfect for ESP-01).

Tutorial: 

To download. click the DOWNLOADS button in the top right corner, rename the uncompressed folder PCF8574. Check that the PCF8574 folder contains `PCF8574\\.cpp` and `PCF8574.h`. Place the DHT library folder your `<arduinosketchfolder>/libraries/` folder. You may need to create the libraries subfolder if its your first library. Restart the IDE.

## Changelog
23/11/2020: v2.2.2 Add multiple implementation for encoder management (you can enable by uncomment relative define)

# Reef complete PCF8574 PCF8574AP digital input and output expander with i2c bus.
I try to simplify the use of this IC, with a minimal set of operations.

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
![Breadboard](https://github.com/xreef/PCF8574_library/blob/master/resources/testReadWriteLedButton_bb.png)

