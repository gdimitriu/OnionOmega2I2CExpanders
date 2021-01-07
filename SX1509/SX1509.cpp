/******************************************************************************
 SparkFunSX1509.cpp
 SparkFun SX1509 I/O Expander Library Source File
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

#include <Arduino.h>

#include <onion-i2c.h>
#include "SX1509.h"
#include "util/sx1509_registers.h"
#include <stdio.h>

SX1509::SX1509() {
	m_clkX = 0;
	m_deviceAddress = 0;
	m_i2cDevice = 0;
	m_pinInterrupt = 0;
	m_pinOscillator = 0;
	m_pinReset = 0;
}

byte SX1509::begin(byte t_address, byte t_resetPin, byte t_i2cDevice) {
	// Store the received parameters into member variables
	m_deviceAddress = t_address;
	m_pinReset = t_resetPin;
	this->m_i2cDevice = t_i2cDevice;

	// If the reset pin is connected
	if (m_pinReset != 255)
		reset(1);
	else
		reset(0);

	// Communication test. We'll read from two registers with different
	// default values to verify communication.
	unsigned int testRegisters = 0;
	testRegisters = readWord(REG_INTERRUPT_MASK_A);	// This should return 0xFF00
	// Then read a byte that should be 0x00
	if (testRegisters == 0xFF00) {
		// Set the clock to a default of 2MHz using internal
		clock(INTERNAL_CLOCK_2MHZ);

		return 1;
	}

	return 0;
}

void SX1509::reset(bool t_hardware) {
	// if t_hardware bool is set
	if (t_hardware) {
		// Check if bit 2 of REG_MISC is set
		// if so nReset will not issue a POR, we'll need to clear that bit first
		byte regMisc = readByte(REG_MISC);
		if (regMisc & (1 << 2)) {
			regMisc &= ~(1 << 2);
			writeByte(REG_MISC, regMisc);
		}
		// Reset the SX1509, the pin is active low
		pinMode(m_pinReset, OUTPUT);	// set reset pin as output
		digitalWrite(m_pinReset, LOW);	// pull reset pin low
		delay(1);	// Wait for the pin to settle
		digitalWrite(m_pinReset, HIGH);	// pull reset pin back high
	} else {
		// Software reset command sequence:
		writeByte(REG_RESET, 0x12);
		writeByte(REG_RESET, 0x34);
	}
}

void SX1509::pinMode(byte t_pin, byte t_inOut) {
	// The SX1509 RegDir registers: REG_DIR_B, REG_DIR_A
	//	0: IO is configured as an output
	//	1: IO is configured as an input
	byte modeBit;
	if ((t_inOut == OUTPUT) || (t_inOut == ANALOG_OUTPUT))
		modeBit = 0;
	else
		modeBit = 1;

	unsigned int tempRegDir = readWord(REG_DIR_B);
	if (modeBit)
		tempRegDir |= (1 << t_pin);
	else
		tempRegDir &= ~(1 << t_pin);

	writeWord(REG_DIR_B, tempRegDir);

	// If INPUT_PULLUP was called, set up the pullup too:
	if (t_inOut == INPUT_PULLUP)
		digitalWrite(t_pin, HIGH);

	if (t_inOut == ANALOG_OUTPUT) {
		ledDriverInit(t_pin);
	}
}

void SX1509::digitalWrite(byte t_pin, byte t_highLow) {
	unsigned int tempRegDir = readWord(REG_DIR_B);

	if ((0xFFFF ^ tempRegDir) & (1 << t_pin))// If the pin is an output, write high/low
			{
		unsigned int tempRegData = readWord(REG_DATA_B);
		if (t_highLow)
			tempRegData |= (1 << t_pin);
		else
			tempRegData &= ~(1 << t_pin);
		writeWord(REG_DATA_B, tempRegData);
	} else	// Otherwise the pin is an input, pull-up/down
	{
		unsigned int tempPullUp = readWord(REG_PULL_UP_B);
		unsigned int tempPullDown = readWord(REG_PULL_DOWN_B);

		if (t_highLow)	// if HIGH, do pull-up, disable pull-down
		{
			tempPullUp |= (1 << t_pin);
			tempPullDown &= ~(1 << t_pin);
			writeWord(REG_PULL_UP_B, tempPullUp);
			writeWord(REG_PULL_DOWN_B, tempPullDown);
		} else	// If LOW do pull-down, disable pull-up
		{
			tempPullDown |= (1 << t_pin);
			tempPullUp &= ~(1 << t_pin);
			writeWord(REG_PULL_UP_B, tempPullUp);
			writeWord(REG_PULL_DOWN_B, tempPullDown);
		}
	}
}

byte SX1509::digitalRead(byte t_pin) {
	unsigned int tempRegDir = readWord(REG_DIR_B);

	if (tempRegDir & (1 << t_pin))	// If the t_pin is an input
			{
		unsigned int tempRegData = readWord(REG_DATA_B);
		if (tempRegData & (1 << t_pin))
			return 1;
	}

	return 0;
}

void SX1509::ledDriverInit(byte t_pin, byte t_freq /*= 1*/, bool t_log /*= false*/) {
	unsigned int tempWord;
	byte tempByte;

	// Disable input buffer
	// Writing a 1 to the pin bit will disable that pins input buffer
	tempWord = readWord(REG_INPUT_DISABLE_B);
	tempWord |= (1 << t_pin);
	writeWord(REG_INPUT_DISABLE_B, tempWord);

	// Disable pull-up
	// Writing a 0 to the pin bit will disable that pull-up resistor
	tempWord = readWord(REG_PULL_UP_B);
	tempWord &= ~(1 << t_pin);
	writeWord(REG_PULL_UP_B, tempWord);

	// Set direction to output (REG_DIR_B)
	tempWord = readWord(REG_DIR_B);
	tempWord &= ~(1 << t_pin); // 0=output
	writeWord(REG_DIR_B, tempWord);

	// Enable oscillator (REG_CLOCK)
	tempByte = readByte(REG_CLOCK);
	tempByte |= (1 << 6);	// Internal 2MHz oscillator part 1 (set bit 6)
	tempByte &= ~(1 << 5);	// Internal 2MHz oscillator part 2 (clear bit 5)
	writeByte(REG_CLOCK, tempByte);

	// Configure LED driver clock and mode (REG_MISC)
	tempByte = readByte(REG_MISC);
	if (t_log) {
		tempByte |= (1 << 7);	// set logarithmic mode bank B
		tempByte |= (1 << 3);	// set logarithmic mode bank A
	} else {
		tempByte &= ~(1 << 7);	// set linear mode bank B
		tempByte &= ~(1 << 3);	// set linear mode bank A
	}

	// Use configClock to setup the clock divder
	if (m_clkX == 0) // Make clckX non-zero
			{
		m_clkX = 2000000.0 / (1 << (1 - 1)); // Update private clock variable

		byte freq = (1 & 0x07) << 4;	// t_freq should only be 3 bits from 6:4
		tempByte |= freq;
	}
	writeByte(REG_MISC, tempByte);

	// Enable LED driver operation (REG_LED_DRIVER_ENABLE)
	tempWord = readWord(REG_LED_DRIVER_ENABLE_B);
	tempWord |= (1 << t_pin);
	writeWord(REG_LED_DRIVER_ENABLE_B, tempWord);

	// Set REG_DATA bit low ~ LED driver started
	tempWord = readWord(REG_DATA_B);
	tempWord &= ~(1 << t_pin);
	writeWord(REG_DATA_B, tempWord);
}

void SX1509::analogWrite(byte t_pin, byte t_iOn) {
	// Write the on intensity of pin
	// Linear mode: Ion = t_iOn
	// Log mode: Ion = f(t_iOn)
	writeByte(REG_I_ON[t_pin], t_iOn);
}

void SX1509::blink(byte t_pin, unsigned long t_tOn, unsigned long t_tOff,
		byte onIntensity, byte offIntensity) {
	byte onReg = calculateLEDTRegister(t_tOn);
	byte offReg = calculateLEDTRegister(t_tOff);

	setupBlink(t_pin, onReg, offReg, onIntensity, offIntensity, 0, 0);
}

void SX1509::breathe(byte t_pin, unsigned long t_tOn, unsigned long t_tOff,
		unsigned long t_rise, unsigned long t_fall, byte t_onInt, byte t_offInt,
		bool t_log) {
	t_offInt = constrain(t_offInt, 0, 7);

	byte onReg = calculateLEDTRegister(t_tOn);
	byte offReg = calculateLEDTRegister(t_tOff);

	byte riseTime = calculateSlopeRegister(t_rise, t_onInt, t_offInt);
	byte fallTime = calculateSlopeRegister(t_fall, t_onInt, t_offInt);

	setupBlink(t_pin, onReg, offReg, t_onInt, t_offInt, riseTime, fallTime, t_log);
}

void SX1509::setupBlink(byte t_pin, byte t_tOn, byte t_tOff, byte t_onIntensity,
		byte t_offIntensity, byte t_tRise, byte t_tFall, bool t_log) {
	ledDriverInit(t_pin, t_log);

	// Keep parameters within their limits:
	t_tOn &= 0x1F;	// t_tOn should be a 5-bit value
	t_tOff &= 0x1F;	// t_tOff should be a 5-bit value
	t_offIntensity &= 0x07;
	// Write the time on
	// 1-15:  TON = 64 * tOn * (255/ClkX)
	// 16-31: TON = 512 * t_tOn * (255/ClkX)
	writeByte(REG_T_ON[t_pin], t_tOn);

	// Write the time/intensity off register
	// 1-15:  TOFF = 64 * tOff * (255/ClkX)
	// 16-31: TOFF = 512 * tOff * (255/ClkX)
	// linear Mode - IOff = 4 * t_offIntensity
	// t_log mode - Ioff = f(4 * t_offIntensity)
	writeByte(REG_OFF[t_pin], (t_tOff << 3) | t_offIntensity);

	// Write the on intensity:
	writeByte(REG_I_ON[t_pin], t_onIntensity);

	// Prepare t_tRise and tFall
	t_tRise &= 0x1F;	// t_tRise is a 5-bit value
	t_tFall &= 0x1F;	// tFall is a 5-bit value

	// Write regTRise
	// 0: Off
	// 1-15:  TRise =      (regIOn - (4 * offIntensity)) * tRise * (255/ClkX)
	// 16-31: TRise = 16 * (regIOn - (4 * t_offIntensity)) * t_tRise * (255/ClkX)
	if (REG_T_RISE[t_pin] != 0xFF)
		writeByte(REG_T_RISE[t_pin], t_tRise);
	// Write regTFall
	// 0: off
	// 1-15:  TFall =      (regIOn - (4 * offIntensity)) * tFall * (255/ClkX)
	// 16-31: TFall = 16 * (regIOn - (4 * t_offIntensity)) * t_tFall * (255/ClkX)
	if (REG_T_FALL[t_pin] != 0xFF)
		writeByte(REG_T_FALL[t_pin], t_tFall);
}

void SX1509::keypad(byte t_rows, byte t_columns, unsigned int t_sleepTime,
		byte t_scanTime, byte t_debounceTime) {
	unsigned int tempWord;
	byte tempByte;

	// If clock hasn't been set up, set it to internal 2MHz
	if (m_clkX == 0)
		clock(INTERNAL_CLOCK_2MHZ);

	// Set regDir 0:7 outputs, 8:15 inputs:
	tempWord = readWord(REG_DIR_B);
	for (int i = 0; i < t_rows; i++)
		tempWord &= ~(1 << i);
	for (int i = 8; i < (t_columns * 2); i++)
		tempWord |= (1 << i);
	writeWord(REG_DIR_B, tempWord);

	// Set regOpenDrain on 0:7:
	tempByte = readByte(REG_OPEN_DRAIN_A);
	for (int i = 0; i < t_rows; i++)
		tempByte |= (1 << i);
	writeByte(REG_OPEN_DRAIN_A, tempByte);

	// Set regPullUp on 8:15:
	tempByte = readByte(REG_PULL_UP_B);
	for (int i = 0; i < t_columns; i++)
		tempByte |= (1 << i);
	writeByte(REG_PULL_UP_B, tempByte);

	// Debounce Time must be less than scan time
	t_debounceTime = constrain(t_debounceTime, 1, 64);
	t_scanTime = constrain(t_scanTime, 1, 128);
	if (t_debounceTime >= t_scanTime) {
		t_debounceTime = t_scanTime >> 1; // Force debounceTime to be less than scanTime
	}
	debounceKeypad(t_debounceTime, t_rows, t_columns);

	// Calculate scanTimeBits, based on scanTime
	byte scanTimeBits = 0;
	for (byte i = 7; i > 0; i--) {
		if (t_scanTime & (1 << i)) {
			scanTimeBits = i;
			break;
		}
	}

	// Calculate sleepTimeBits, based on sleepTime
	byte sleepTimeBits = 0;
	if (t_sleepTime != 0) {
		for (byte i = 7; i > 0; i--) {
			if (t_sleepTime & ((unsigned int) 1 << (i + 6))) {
				sleepTimeBits = i;
				break;
			}
		}
		// If sleepTime was non-zero, but less than 128, 
		// assume we wanted to turn sleep on, set it to minimum:
		if (sleepTimeBits == 0)
			sleepTimeBits = 1;
	}

	// RegKeyConfig1 sets the auto sleep time and scan time per row
	sleepTimeBits = (sleepTimeBits & 0b111) << 4;
	scanTimeBits &= 0b111;	// Scan time is bits 2:0
	tempByte = t_sleepTime | scanTimeBits;
	writeByte(REG_KEY_CONFIG_1, tempByte);

	// RegKeyConfig2 tells the SX1509 how many rows and columns we've got going
	t_rows = (t_rows - 1) & 0b111;	// 0 = off, 0b001 = 2 rows, 0b111 = 8 rows, etc.
	t_columns = (t_columns - 1) & 0b111;// 0b000 = 1 column, ob111 = 8 columns, etc.
	writeByte(REG_KEY_CONFIG_2, (t_rows << 3) | t_columns);
}

unsigned int SX1509::readKeypad() {
	return (0xFFFF ^ readWord(REG_KEY_DATA_1));
}

byte SX1509::getRow(unsigned int t_keyData) {
	byte rowData = byte(t_keyData & 0x00FF);

	for (int i = 0; i < 8; i++) {
		if (rowData & (1 << i))
			return i;
	}
	return 0;
}

byte SX1509::getCol(unsigned int t_keyData) {
	byte colData = byte((t_keyData & 0xFF00) >> 8);

	for (int i = 0; i < 8; i++) {
		if (colData & (1 << i))
			return i;
	}
	return 0;

}

void SX1509::sync(void) {
	// First check if nReset functionality is set
	byte regMisc = readByte(REG_MISC);
	if (!(regMisc & 0x04)) {
		regMisc |= (1 << 2);
		writeByte(REG_MISC, regMisc);
	}

	// Toggle nReset pin to sync LED timers
	pinMode(m_pinReset, OUTPUT);	// set reset pin as output
	digitalWrite(m_pinReset, LOW);	// pull reset pin low
	delay(1);	// Wait for the pin to settle
	digitalWrite(m_pinReset, HIGH);	// pull reset pin back high	

	// Return nReset to POR functionality
	writeByte(REG_MISC, (regMisc & ~(1 << 2)));
}

void SX1509::debounceConfig(byte t_configValue) {
	// First make sure clock is configured
	byte tempByte = readByte(REG_MISC);
	if ((tempByte & 0x70) == 0) {
		tempByte |= (1 << 4);	// Just default to no divider if not set
		writeByte(REG_MISC, tempByte);
	}
	tempByte = readByte(REG_CLOCK);
	if ((tempByte & 0x60) == 0) {
		tempByte |= (1 << 6);	// default to internal osc.
		writeByte(REG_CLOCK, tempByte);
	}

	t_configValue &= 0b111;	// 3-bit value
	writeByte(REG_DEBOUNCE_CONFIG, t_configValue);
}

void SX1509::debounceTime(byte t_time) {
	if (m_clkX == 0) // If clock hasn't been set up.
		clock(INTERNAL_CLOCK_2MHZ, 1); // Set clock to 2MHz.

	// Debounce time-to-byte map: (assuming fOsc = 2MHz)
	// 0: 0.5ms		1: 1ms
	// 2: 2ms		3: 4ms
	// 4: 8ms		5: 16ms
	// 6: 32ms		7: 64ms
	// 2^(n-1)
	byte configValue = 0;
	// We'll check for the highest set bit position, 
	// and use that for debounceConfig
	for (int i = 7; i >= 0; i--) {
		if (t_time & (1 << i)) {
			configValue = i + 1;
			break;
		}
	}
	configValue = constrain(configValue, 0, 7);

	debounceConfig(configValue);
}

void SX1509::debouncePin(byte t_pin) {
	unsigned int debounceEnable = readWord(REG_DEBOUNCE_ENABLE_B);
	debounceEnable |= (1 << t_pin);
	writeWord(REG_DEBOUNCE_ENABLE_B, debounceEnable);
}

void SX1509::debounceKeypad(byte t_time, byte t_numRows, byte t_numCols) {
	// Set up debounce time:
	debounceTime(t_time);

	// Set up debounce pins:
	for (int i = 0; i < t_numRows; i++)
		debouncePin(i);
	for (int i = 0; i < (8 + t_numCols); i++)
		debouncePin(i);
}

void SX1509::enableInterrupt(byte t_pin, byte t_riseFall) {
	// Set REG_INTERRUPT_MASK
	unsigned int tempWord = readWord(REG_INTERRUPT_MASK_B);
	tempWord &= ~(1 << t_pin);	// 0 = event on IO will trigger interrupt
	writeWord(REG_INTERRUPT_MASK_B, tempWord);

	byte sensitivity = 0;
	switch (t_riseFall) {
	case CHANGE:
		sensitivity = 0b11;
		break;
	case FALLING:
		sensitivity = 0b10;
		break;
	case RISING:
		sensitivity = 0b01;
		break;
	}

	// Set REG_SENSE_XXX
	// Sensitivity is set as follows:
	// 00: None
	// 01: Rising
	// 10: Falling
	// 11: Both
	byte pinMask = (t_pin & 0x07) * 2;
	byte senseRegister;

	// Need to select between two words. One for bank A, one for B.
	if (t_pin >= 8)
		senseRegister = REG_SENSE_HIGH_B;
	else
		senseRegister = REG_SENSE_HIGH_A;

	tempWord = readWord(senseRegister);
	tempWord &= ~(0b11 << pinMask);	// Mask out the bits we want to write
	tempWord |= (sensitivity << pinMask);	// Add our new bits
	writeWord(senseRegister, tempWord);
}

unsigned int SX1509::interruptSource(bool t_clear /* =true*/) {
	unsigned int intSource = readWord(REG_INTERRUPT_SOURCE_B);
	if (t_clear)
		writeWord(REG_INTERRUPT_SOURCE_B, 0xFFFF);	// Clear interrupts
	return intSource;
}

bool SX1509::checkInterrupt(int t_pin) {
	if (interruptSource(false) & (1 << t_pin))
		return true;

	return false;
}

void SX1509::clock(byte t_oscSource, byte t_oscDivider, byte t_oscPinFunction,
		byte t_oscFreqOut) {
	// RegClock constructed as follows:
	//	6:5 - Oscillator frequency souce
	//		00: off, 01: external input, 10: internal 2MHz, 1: reserved
	//	4 - OSCIO pin function
	//		0: input, 1 ouptut
	//	3:0 - Frequency of oscout pin
	//		0: LOW, 0xF: high, else fOSCOUT = FoSC/(2^(RegClock[3:0]-1))
	t_oscSource = (t_oscSource & 0b11) << 5;		// 2-bit value, bits 6:5
	t_oscPinFunction = (t_oscPinFunction & 1) << 4;	// 1-bit value bit 4
	t_oscFreqOut = (t_oscFreqOut & 0b1111);	// 4-bit value, bits 3:0
	byte regClock = t_oscSource | t_oscPinFunction | t_oscFreqOut;
	writeByte(REG_CLOCK, regClock);

	// Config RegMisc[6:4] with oscDivider
	// 0: off, else ClkX = fOSC / (2^(RegMisc[6:4] -1))
	t_oscDivider = constrain(t_oscDivider, 1, 7);
	m_clkX = 2000000.0 / (1 << (t_oscDivider - 1)); // Update private clock variable
	t_oscDivider = (t_oscDivider & 0b111) << 4;	// 3-bit value, bits 6:4

	byte regMisc = readByte(REG_MISC);
	regMisc &= ~(0b111 << 4);
	regMisc |= t_oscDivider;
	writeByte(REG_MISC, regMisc);
}

byte SX1509::calculateLEDTRegister(int t_ms) {
	int regOn1, regOn2;
	float timeOn1, timeOn2;

	if (m_clkX == 0)
		return 0;

	regOn1 = (float) (t_ms / 1000.0) / (64.0 * 255.0 / (float) m_clkX);
	regOn2 = regOn1 / 8;
	regOn1 = constrain(regOn1, 1, 15);
	regOn2 = constrain(regOn2, 16, 31);

	timeOn1 = 64.0 * regOn1 * 255.0 / m_clkX * 1000.0;
	timeOn2 = 512.0 * regOn2 * 255.0 / m_clkX * 1000.0;

	if (abs(timeOn1 - t_ms) < abs(timeOn2 - t_ms))
		return regOn1;
	else
		return regOn2;
}

byte SX1509::calculateSlopeRegister(int t_ms, byte t_onIntensity,
		byte t_offIntensity) {
	int regSlope1, regSlope2;
	float regTime1, regTime2;

	if (m_clkX == 0)
		return 0;

	float tFactor = ((float) t_onIntensity - (4.0 * (float) t_offIntensity)) * 255.0
			/ (float) m_clkX;
	float timeS = float(t_ms) / 1000.0;

	regSlope1 = timeS / tFactor;
	regSlope2 = regSlope1 / 16;

	regSlope1 = constrain(regSlope1, 1, 15);
	regSlope2 = constrain(regSlope2, 16, 31);

	regTime1 = regSlope1 * tFactor * 1000.0;
	regTime2 = 16 * regTime1;

	if (abs(regTime1 - t_ms) < abs(regTime2 - t_ms))
		return regSlope1;
	else
		return regSlope2;
}

// readByte(byte t_registerAddress)
//	This function reads a single byte located at the t_registerAddress register.
//	- deviceAddress should already be set by the constructor.
//	- Return value is the byte read from registerAddress
//		- Currently returns 0 if communication has timed out
byte SX1509::readByte(byte t_registerAddress) {
	byte readValue;
	i2c_writeBufferRaw(m_i2cDevice, m_deviceAddress, &t_registerAddress, 1);
	i2c_readRaw(m_i2cDevice, m_deviceAddress, &readValue, 1);
	return readValue;
}

// readWord(byte t_registerAddress)
//	This function will read a two-byte word beginning at t_registerAddress
//	- A 16-bit unsigned int will be returned.
//		- The msb of the return value will contain the value read from t_registerAddress
//		- The lsb of the return value will contain the value read from t_registerAddress + 1
unsigned int SX1509::readWord(byte t_registerAddress) {
	unsigned int readValue;
	unsigned int msb, lsb;
	byte tmpRead;

	i2c_writeBufferRaw(m_i2cDevice, m_deviceAddress, &t_registerAddress, 1);

	i2c_readRaw(m_i2cDevice, m_deviceAddress, &tmpRead, 1);
	msb = (tmpRead & 0x00FF) << 8;
	i2c_readRaw(m_i2cDevice, m_deviceAddress, &tmpRead, 1);
	lsb = (tmpRead & 0x00FF);
	readValue = msb | lsb;

	return readValue;
}

// readBytes(byte t_firstRegisterAddress, byte * t_destination, byte t_length)
//	This function reads a series of bytes incrementing from a given address
//	- t_firstRegsiterAddress is the first address to be read
//	- t_destination is an array of bytes where the read values will be stored into
//	- t_length is the number of bytes to be read
//	- No return value.
void SX1509::readBytes(byte t_firstRegisterAddress, byte *t_destination,
		byte length) {
	i2c_writeBufferRaw(m_i2cDevice, m_deviceAddress, &t_firstRegisterAddress, 1);
	for (int i = 0; i < length; i++) {
		i2c_readRaw(m_i2cDevice, m_deviceAddress, &t_destination[i], 1);
	}
}

// writeByte(byte t_registerAddress, byte t_writeValue)
//	This function writes a single byte to a single register on the SX509.
//	- writeValue is written to t_registerAddress
//	- t_deviceAddres should already be set from the constructor
//	- No return value.
void SX1509::writeByte(byte t_registerAddress, byte t_writeValue) {
	i2c_writeBuffer(m_i2cDevice, m_deviceAddress, t_registerAddress, &t_writeValue,
			1);
}

// writeWord(byte t_registerAddress, ungisnged int t_writeValue)
//	This function writes a two-byte word to t_registerAddress and t_registerAddress + 1
//	- the upper byte of writeValue is written to t_registerAddress
//		- the lower byte of writeValue is written to t_registerAddress + 1
//	- No return value.
void SX1509::writeWord(byte t_registerAddress, unsigned int t_writeValue) {
	byte msb, lsb;
	msb = ((t_writeValue & 0xFF00) >> 8);
	lsb = (t_writeValue & 0x00FF);
	//i2c_writeBufferRaw(i2cDevice, deviceAddress, &t_registerAddress, 1);
	i2c_writeBuffer(m_i2cDevice, m_deviceAddress, t_registerAddress, &msb, 1);
	i2c_writeBuffer(m_i2cDevice, m_deviceAddress, t_registerAddress + 1, &lsb, 1);
}

// writeBytes(byte t_firstRegisterAddress, byte * t_writeArray, byte t_length)
//	This function writes an array of bytes, beggining at a specific adddress
//	- firstRegisterAddress is the initial register to be written.
//		- All writes following will be at incremental register addresses.
//	- writeArray should be an array of byte values to be written.
//	- length should be the number of bytes to be written.
//	- no return value.
void SX1509::writeBytes(byte t_firstRegisterAddress, byte *t_writeArray,
		byte length) {
	i2c_writeBufferRaw(m_i2cDevice, m_deviceAddress, &t_firstRegisterAddress, 1);
	for (int i = 0; i < length; i++) {
		i2c_writeBufferRaw(m_i2cDevice, m_deviceAddress, &t_writeArray[i], 1);
	}
}
