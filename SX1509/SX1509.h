/******************************************************************************
SparkFunSX1509.h
SparkFun SX1509 I/O Expander Library Header File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: September 21, 2015
https://github.com/sparkfun/SparkFun_SX1509_Arduino_Library

Porting to onion omega:
Copyright (C) 2019 Gabriel Dimitriu
All rights reserved.

Here you'll find the Arduino code used to interface with the SX1509 I2C
16 I/O expander. There are functions to take advantage of everything the
SX1509 provides - input/output setting, writing pins high/low, reading 
the input value of pins, LED driver utilities (blink, breath, pwm), and
keypad engine utilites.

Development environment specifics:
	IDE: Arduino 1.6.5
	Hardware Platform: Arduino Uno
	SX1509 Breakout Version: v2.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/


#ifndef SX1509_H
#define SX1509_H

#define RECEIVE_TIMEOUT_VALUE 1000	// Timeout for I2C receive

// These are used for setting LED driver to linear or log mode:
#define LINEAR		0	
#define LOGARITHMIC	1

// These are used for clock config:
#define INTERNAL_CLOCK_2MHZ	2
#define EXTERNAL_CLOCK	1

#define SOFTWARE_RESET 0
#define HARDWARE_RESET 1

#define ANALOG_OUTPUT 0x3 // To set a pin mode for PWM output
typedef unsigned char uint8_t;
typedef unsigned char byte;
class SX1509
{
private:	// These private functions are not available to Arduino sketches.
			// If you need to read or write directly to registers, consider
			// putting the writeByte, readByte functions in the public section
	byte m_deviceAddress; // I2C Address of SX1509
	byte m_i2cDevice; //i2c device chanell 0 on onion omega
// Pin definitions:
	byte m_pinInterrupt;
	byte m_pinOscillator;
	byte m_pinReset;
// Misc variables:
	unsigned long m_clkX;
// Read Functions:
	byte readByte(byte t_registerAddress);
	unsigned int readWord(byte t_registerAddress);
	void readBytes(byte t_firstRegisterAddress, byte * t_destination, byte t_length);
// Write functions:
	void writeByte(byte t_registerAddress, byte t_writeValue);
	void writeWord(byte t_registerAddress, unsigned int t_writeValue);
	void writeBytes(byte t_firstRegisterAddress, byte * t_writeArray, byte t_length);
// Helper functions:
	// calculateLEDTRegister - Try to estimate an LED on/off duration register, 
	// given the number of milliseconds and LED clock frequency.
	byte calculateLEDTRegister(int t_ms);
	// calculateSlopeRegister - Try to estimate an LED rise/fall duration 
	// register, given the number of milliseconds and LED clock frequency.
	byte calculateSlopeRegister(int t_ms, byte t_onIntensity, byte t_offIntensity);

public:
// -----------------------------------------------------------------------------
// Constructor - SX1509: This function sets up the pins connected to the 
//		SX1509, and sets up the private deviceAddress variable.
// -----------------------------------------------------------------------------
	SX1509();

// -----------------------------------------------------------------------------
// begin(byte address, byte resetPin): This function initializes the SX1509.
//  	It begins the Wire library, resets the IC, and tries to read some 
//  	registers to prove it's connected.
// Inputs:
//		- address: should be the 7-bit address of the SX1509. This should be  
//		 one of four values - 0x3E, 0x3F, 0x70, 0x71 - all depending on what the
//		 ADDR0 and ADDR1 pins ar se to. This variable is required.
//		- resetPin: This is the Arduino pin tied to the SX1509 RST pin. This
//		 pin is optional. If not declared, the library will attempt to
//		 software reset the SX1509.
// Output: Returns a 1 if communication is successful, 0 on error.
// -----------------------------------------------------------------------------
	byte begin(byte t_address = 0x3E, byte t_resetPin = 0xFF, byte t_i2cDevice = 0);
	
// -----------------------------------------------------------------------------
// reset(bool hardware): This function resets the SX1509 - either a hardware 
//		reset or software. A hardware reset (hardware parameter = 1) pulls the 
//		reset line low, pausing, then pulling the reset line high. A software 
//		reset writes a 0x12 then 0x34 to the REG_RESET as outlined in the
//		datasheet.
//
//  Input:
//	 	- hardware: 0 executes a software reset, 1 executes a hardware reset
// -----------------------------------------------------------------------------
	void reset(bool t_hardware);

// -----------------------------------------------------------------------------
// pinMode(byte pin, byte inOut): This function sets one of the SX1509's 16 
//		outputs to either an INPUT or OUTPUT.
//
//	Inputs:
//	 	- pin: should be a value between 0 and 15
//	 	- inOut: The Arduino INPUT and OUTPUT constants should be used for the 
//		 inOut parameter. They do what they say!
// -----------------------------------------------------------------------------
	void pinMode(byte t_pin, byte t_inOut);
	
// -----------------------------------------------------------------------------
// digitalWrite(byte pin, byte highLow): This function writes a pin to either high 
//		or low if it's configured as an OUTPUT. If the pin is configured as an 
//		INPUT, this method will activate either the PULL-UP	or PULL-DOWN
//		resistor (HIGH or LOW respectively).
//
//	Inputs:
//		- pin: The SX1509 pin number. Should be a value between 0 and 15.
//		- highLow: should be Arduino's defined HIGH or LOW constants.
// -----------------------------------------------------------------------------
	void digitalWrite(byte t_pin, byte t_highLow);

// -----------------------------------------------------------------------------
// digitalRead(byte pin): This function reads the HIGH/LOW status of a pin.
//		The pin should be configured as an INPUT, using the pinDir function.
//
//	Inputs:
//	 	- pin: The SX1509 pin to be read. should be a value between 0 and 15.
//  Outputs:
//		This function returns a 1 if HIGH, 0 if LOW
// -----------------------------------------------------------------------------
	byte digitalRead(byte t_pin);
	
// -----------------------------------------------------------------------------
// ledDriverInit(byte pin, byte freq, bool log): This function initializes LED 
//		driving on a pin. It must be called if you want to use the pwm or blink 
//		functions on that pin.
//	
//	Inputs:
//		- pin: The SX1509 pin connected to an LED. Should be 0-15.
//   	- freq: Sets LED clock frequency divider.
//		- log: selects either linear or logarithmic mode on the LED drivers
//			- log defaults to 0, linear mode
//			- currently log sets both bank A and B to the same mode
//	Note: this function automatically decides to use the internal 2MHz osc.
// -----------------------------------------------------------------------------
	void ledDriverInit(byte t_pin, byte t_freq = 1, bool t_log = false);
	
// -----------------------------------------------------------------------------
// analogWrite(byte pin, byte iOn):	This function can be used to control the intensity 
//		of an output pin connected to an LED.
//
//	Inputs:
//		- pin: The SX1509 pin connecte to an LED.Should be 0-15.
//		- iOn: should be a 0-255 value setting the intensity of the LED
//			- 0 is completely off, 255 is 100% on.
//
//	Note: ledDriverInit should be called on the pin before calling this.
// -----------------------------------------------------------------------------
	void analogWrite(byte t_pin, byte t_iOn);
	
// -----------------------------------------------------------------------------
// setupBlink(byte pin, byte tOn, byte tOff, byte offIntensity, byte tRise, byte 
//		tFall):  blink performs both the blink and breath LED driver functions.
//
// 	Inputs:
//  	- pin: the SX1509 pin (0-15) you want to set blinking/breathing.
//		- tOn: the amount of time the pin is HIGH
//			- This value should be between 1 and 31. 0 is off.
//		- tOff: the amount of time the pin is at offIntensity
//			- This value should be between 1 and 31. 0 is off.
//		- offIntensity: How dim the LED is during the off period.
//			- This value should be between 0 and 7. 0 is completely off.
//		- onIntensity: How bright the LED will be when completely on.
//			- This value can be between 0 (0%) and 255 (100%).
//		- tRise: This sets the time the LED takes to fade in.
//			- This value should be between 1 and 31. 0 is off.
//			- This value is used with tFall to make the LED breath.
//		- tFall: This sets the time the LED takes to fade out.
//			- This value should be between 1 and 31. 0 is off.
// 	 Notes: 
//		- The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only. If tRise and 
//			tFall are set on 0-3 or 8-11 those pins will still only blink.
// 		- ledDriverInit should be called on the pin to be blinked before this.
// -----------------------------------------------------------------------------
	void setupBlink(byte t_pin, byte t_tOn, byte t_toff, byte t_onIntensity = 255, byte t_offIntensity = 0, byte t_tRise = 0, byte t_tFall = 0, bool t_log = false);

// -----------------------------------------------------------------------------
// blink(byte pin, unsigned long tOn, unsigned long tOff, byte onIntensity, byte offIntensity);
//  	Set a pin to blink output for estimated on/off millisecond durations.
//
// 	Inputs:
//  	- pin: the SX1509 pin (0-15) you want to set blinking
//   	- tOn: estimated number of milliseconds the pin is LOW (LED sinking current will be on)
//   	- tOff: estimated number of milliseconds the pin is HIGH (LED sinking current will be off)
//   	- onIntensity: 0-255 value determining LED on brightness
//   	- offIntensity: 0-255 value determining LED off brightness
// 	 Notes: 
//		- The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only. If tRise and 
//			tFall are set on 0-3 or 8-11 those pins will still only blink.
// 		- ledDriverInit should be called on the pin to be blinked before this.
// -----------------------------------------------------------------------------
	void blink(byte t_pin, unsigned long t_tOn, unsigned long t_tOff, byte t_onIntensity = 255, byte t_offIntensity = 0);
	
// -----------------------------------------------------------------------------
// breathe(byte pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, byte onInt, byte offInt, bool log);
//  	Set a pin to breathe output for estimated on/off millisecond durations, with
//  	estimated rise and fall durations.
//
// 	Inputs:
//  	- pin: the SX1509 pin (0-15) you want to set blinking
//   	- tOn: estimated number of milliseconds the pin is LOW (LED sinking current will be on)
//   	- tOff: estimated number of milliseconds the pin is HIGH (LED sinking current will be off)
//   	- rise: estimated number of milliseconds the pin rises from LOW to HIGH
//   	- falll: estimated number of milliseconds the pin falls from HIGH to LOW
//   	- onIntensity: 0-255 value determining LED on brightness
//   	- offIntensity: 0-255 value determining LED off brightness
// 	 Notes: 
//		- The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only. If tRise and 
//			tFall are set on 0-3 or 8-11 those pins will still only blink.
// 		- ledDriverInit should be called on the pin to be blinked before this,
//  	  Or call pinMode(<pin>, ANALOG_OUTPUT);
// -----------------------------------------------------------------------------
	void breathe(byte t_pin, unsigned long t_tOn, unsigned long t_tOff, unsigned long t_rise, unsigned long t_fall, byte t_onInt = 255, byte t_offInt = 0, bool t_log = LINEAR);

// -----------------------------------------------------------------------------
// keypad(byte rows, byte columns, byte sleepTime, byte scanTime, byte debounceTime)
//		Initializes the keypad function on the SX1509. Millisecond durations for sleep,
//		scan, and debounce can be set.
//
//	Inputs:
//		- rows: The number of rows in the button matrix.
//			- This value must be between 1 and 7. 0 will turn it off.
//			- eg: 1 = 2 rows, 2 = 3 rows, 7 = 8 rows, etc.
//		- columns: The number of columns in the button matrix
//			- This value should be between 0 and 7.
//			- 0 = 1 column, 7 = 8 columns, etc.
//		- sleepTime: Sets the auto-sleep time of the keypad engine.
//  	  Should be a millisecond duration between 0 (OFF) and 8000 (8 seconds).
//   	  Possible values are 0, 128, 256, 512, 1000, 2000, 4000, 8000
//		- scanTime: Sets the scan time per row. Must be set above debounce.
//  	  Should be a millisecond duration between 1 and 128.
//   	  Possible values are 1, 2, 4, 8, 16, 32, 64, 128.
//		- debounceTime: Sets the debounc time per button. Must be set below scan.
//  	  Should be a millisecond duration between 0 and 64.
//   	  Possible values are 0 (0.5), 1, 2, 4, 8, 16, 32, 64.
// -----------------------------------------------------------------------------
	void keypad(byte t_rows, byte t_columns, unsigned int t_sleepTime = 0, byte t_scanTime = 1, byte t_debounceTime = 0);

// -----------------------------------------------------------------------------
// readKeypad(): This function returns a 16-bit value containing the status of
//		keypad engine.
//
//	Output:
//		A 16-bit value is returned. The lower 8 bits represent the up-to 8 rows,
//		while the MSB represents the up-to 8 columns. Bit-values of 1 indicate a
//		button in that row or column is being pressed. As such, at least two
//		bits should be set.
// -----------------------------------------------------------------------------
	unsigned int readKeypad();
	
// -----------------------------------------------------------------------------
// getRow(): This function returns the first active row from the return value of
//  	readKeypad().
//
//	Input:
//      - keyData: Should be the unsigned int value returned from readKeypad().
//	Output:
//		A 16-bit value is returned. The lower 8 bits represent the up-to 8 rows,
//		while the MSB represents the up-to 8 columns. Bit-values of 1 indicate a
//		button in that row or column is being pressed. As such, at least two
//		bits should be set.
// -----------------------------------------------------------------------------
	byte getRow(unsigned int t_keyData);
	
// -----------------------------------------------------------------------------
// getCol(): This function returns the first active column from the return value of
//  	readKeypad().
//
//	Input:
//      - keyData: Should be the unsigned int value returned from readKeypad().
//	Output:
//		A 16-bit value is returned. The lower 8 bits represent the up-to 8 rows,
//		while the MSB represents the up-to 8 columns. Bit-values of 1 indicate a
//		button in that row or column is being pressed. As such, at least two
//		bits should be set.
// -----------------------------------------------------------------------------
	byte getCol(unsigned int t_keyData);
	
// -----------------------------------------------------------------------------
// sync(void): this function resets the PWM/Blink/Fade counters, syncing any 
//		blinking LEDs. Bit 2 of REG_MISC is set, which alters the functionality 
//		of the nReset pin. The nReset pin is toggled low->high, which should 
//		reset all LED counters. Bit 2 of REG_MISC is again cleared, returning 
//		nReset pin to POR functionality
// -----------------------------------------------------------------------------
	void sync(void);
	
// -----------------------------------------------------------------------------
// debounceConfig(byte configValue): This method configures the debounce time of 
//		every input.
//
//	Input:
//		- configValue: A 3-bit value configuring the debounce time.
//			000: 0.5ms * 2MHz/fOSC
//			001: 1ms * 2MHz/fOSC
//			010: 2ms * 2MHz/fOSC
//			011: 4ms * 2MHz/fOSC
//			100: 8ms * 2MHz/fOSC
//			101: 16ms * 2MHz/fOSC
//			110: 32ms * 2MHz/fOSC
//			111: 64ms * 2MHz/fOSC
//
//	Note: fOSC is set with the configClock function. It defaults to 2MHz.
// -----------------------------------------------------------------------------
	void debounceConfig(byte t_configVaule);
	
// -----------------------------------------------------------------------------
// debounceTime(byte configValue): This method configures the debounce time of 
//		every input to an estimated millisecond time duration.
//
//	Input:
//		- time: A millisecond duration estimating the debounce time. Actual
//		  debounce time will depend on fOSC. Assuming it's 2MHz, debounce will
//		  be set to the 0.5, 1, 2, 4, 8, 16, 32, or 64 ms (whatever's closest)
//
//	Note: fOSC is set with the configClock function. It defaults to 2MHz.
// -----------------------------------------------------------------------------
	void debounceTime(byte t_time);

// -----------------------------------------------------------------------------
// debouncePin(byte pin): This method enables debounce on SX1509 input pin.
//
//	Input:
//		- pin: The SX1509 pin to be debounced. Should be between 0 and 15.
// -----------------------------------------------------------------------------
	void debouncePin(byte t_pin);

// -----------------------------------------------------------------------------
// debounceKeypad(byte pin): This method enables debounce on all pins connected
//  to a row/column keypad matrix.
//
//	Input:
//		- time: Millisecond time estimate for debounce (see debounceTime()).
//		- numRows: The number of rows in the keypad matrix.
//		- numCols: The number of columns in the keypad matrix.
// -----------------------------------------------------------------------------	
	void debounceKeypad(byte t_time, byte t_numRows, byte t_numCols);

// -----------------------------------------------------------------------------	
// enableInterrupt(byte pin, byte riseFall): This function sets up an interrupt 
//		on a pin. Interrupts can occur on all SX1509 pins, and can be generated
//		on rising, falling, or both.
//
//	Inputs: 
//		-pin: SX1509 input pin that will generate an input. Should be 0-15.
//		-riseFall: Configures if you want an interrupt generated on rise fall or
//			both. For this param, send the pin-change values previously defined 
//			by Arduino:
//			#define CHANGE 1	<-Both
//			#define FALLING 2	<- Falling
//			#define RISING 3	<- Rising
//	
//	Note: This function does not set up a pin as an input, or configure	its 
//		pull-up/down resistors! Do that before (or after).
// -----------------------------------------------------------------------------
	void enableInterrupt(byte t_pin, byte t_riseFall);
	
// -----------------------------------------------------------------------------
// interruptSource(void): Returns an unsigned int representing which pin caused 
//		an interrupt.
//
//	Output: 16-bit value, with a single bit set representing the pin(s) that
//		generated an interrupt. E.g. a return value of	0x0104 would mean pins 8
//		and 3 (bits 8 and 3) have generated an interrupt.
//  Input:
//  	- clear: boolean commanding whether the interrupt should be cleared
//  	  after reading or not.
// -----------------------------------------------------------------------------
	unsigned int interruptSource(bool t_clear = true);

// -----------------------------------------------------------------------------
// checkInterrupt(void): Checks if a single pin generated an interrupt.
//
//	Output: Boolean value. True if the requested pin has triggered an interrupt/
//  Input:
//  	- pin: Pin to be checked for generating an input.
// -----------------------------------------------------------------------------	
	bool checkInterrupt(int t_pin);


// -----------------------------------------------------------------------------
// clock(byte oscSource, byte oscDivider, byte oscPinFunction, byte oscFreqOut)
//		This function configures the oscillator source/speed
//		and the clock, which is used to drive LEDs and time debounces.
//	Inputs:
//	- oscSource: Choose either internal 2MHz oscillator or an external signal 
//		applied to the OSCIO pin.
//		- INTERNAL_CLOCK and EXTERNAL_CLOCK are defined in the header file.
//			Use those.
//		- This value defaults to internal.
//	- oscDivider: Sets the clock divider in REG_MISC.
//		- ClkX = fOSC / (2^(RegMisc[6:4] -1))
//		- This value defaults to 1.
//	- oscPinFunction: Allows you to set OSCIO as an input or output.
//		- You can use Arduino's INPUT, OUTPUT defines for this value
//		- This value defaults to input
//	- oscFreqOut: If oscio is configured as an output, this will set the output 
//		frequency
//		- This should be a 4-bit value. 0=0%, 0xF=100%, else 
//			fOSCOut = FOSC / (2^(RegClock[3:0]-1))
//		- This value defaults to 0.
// -----------------------------------------------------------------------------
	void clock(byte t_oscSource = 2, byte t_oscDivider = 1, byte t_oscPinFunction = 0, byte t_oscFreqOut = 0);
};

// Add backwards compatibility for the old class name: sx1509Class
typedef SX1509 sx1509Class;

#endif	// SX1509_library_H
