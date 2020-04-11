### Additional information and document update here on my site: [pcf8574 Article](https://www.mischianti.org/2019/01/02/pcf8574-i2c-digital-i-o-expander-fast-easy-usage/).

### If you need more pins [here](https://www.mischianti.org/2019/07/22/pcf8575-i2c-16-bit-digital-i-o-expander/) you can find pcf8575 16bit version of the IC.

### Version 2.2

Library to use i2c analog IC with arduino and esp8266. Can read and write digital value with only 2 wire (perfect for ESP-01).

Tutorial: 

To download. click the DOWNLOADS button in the top right corner, rename the uncompressed folder PCF8574. Check that the PCF8574 folder contains `PCF8574\\.cpp` and `PCF8574.h`. Place the DHT library folder your `<arduinosketchfolder>/libraries/` folder. You may need to create the libraries subfolder if its your first library. Restart the IDE.

# Reef complete PCF8574 PCF8574AP digital input and output expander with i2c bus.
I try to simplify the use of this IC, with a minimal set of operation.

PCF8574P address map 0x20-0x27 
PCF8574AP address map 0x38-0x3f 

Constructor:
you must pas the address of i2c (to check the adress use this guide [I2cScanner](https://playground.arduino.cc/Main/I2cScanner)) 
```cpp
	PCF8574(uint8_t address);
```
for esp8266 if you want specify SDA e SCL pin use this:

```cpp
	PCF8574(uint8_t address, uint8_t sda, uint8_t scl);
```
You must set input/output mode:
```cpp
	pcf8574.pinMode(P0, OUTPUT);
	pcf8574.pinMode(P1, INPUT);
	pcf8574.pinMode(P2, INPUT);
```

then IC as you can see in the image have 8 digital input/output:

![PCF8574 schema](https://github.com/xreef/PCF8574_library/blob/master/resources/PCF8574-pins.gif)

So to read all analog input in one trasmission you can do (even if I use  a 10millis debounce time to prevent too much read from i2c):
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

To follow a request (you can see It on [issue #5](https://github.com/xreef/PCF8574_library/issues/5)) I create a define variable to work with low memori device, if you decomment this line on .h file of the library:

```cpp
// #define PCF8574_LOW_MEMORY
```

Enable low memory props and gain about 7byte of memory, and you must use the method to read all like so:

 ```cpp
	byte di = pcf8574.digitalReadAll();
	Serial.print("READ VALUE FROM PCF: ");
	Serial.println(di, BIN);
```

where di is a byte like 1110001, so you must do a bitwise operation to get the data, operation that I already do in the "normal" mode, here an example:

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
 

if you want read a single input:

```cpp
	int p1Digital = PCF8574.digitalRead(P1); // read P1
```

If you want write a digital value you must do:
```cpp
	PCF8574.digitalWrite(P1, HIGH);
```
or:
```cpp
	PCF8574.digitalWrite(P1, LOW);
```

You can also use interrupt pin:
You must initialize the pin and the function to call when interrupt raised from PCF8574
```cpp
	// Function interrupt
void keyPressedOnPCF8574();

// Set i2c address
PCF8574 pcf8574(0x39, ARDUINO_UNO_INTERRUPT_PIN, keyPressedOnPCF8574);
```
Remember you can't use Serial or Wire on interrupt function.

The better way is to set only a variable to read on loop:
```cpp
void keyPressedOnPCF8574(){
	// Interrupt called (No Serial no read no wire in this function, and DEBUG disabled on PCF library)
	 keyPressed = true;
}
```

For the examples I use this wire schema on breadboard:
![Breadboard](https://github.com/xreef/PCF8574_library/blob/master/resources/testReadWriteLedButton_bb.png)

