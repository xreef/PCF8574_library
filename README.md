Library to use i2c analog IC with arduino and esp8266. Can read and write digital value with only 2 wire (perfect for ESP-01).

Tutorial: 

To download. click the DOWNLOADS button in the top right corner, rename the uncompressed folder PCF8574. Check that the PCF8574 folder contains `PCF8574\\.cpp` and `PCF8574.h`. Place the DHT library folder your `<arduinosketchfolder>/libraries/` folder. You may need to create the libraries subfolder if its your first library. Restart the IDE.

# Reef complete PCF8574 digital input and output expander with i2c bus.
I try to simplify the use of this IC, with a minimal set of operation.

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

