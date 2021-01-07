/*
PC8754 extender operations driver.
Copyright (C) 2019 Gabriel Dimitriu
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include <RegisterOperations.h>
#include <Arduino.h>
#ifndef PCF8574_h
#define PCF8574_h

class PCF8574
{
public:
	PCF8574(unsigned char t_i2cDev, unsigned char t_address);
	unsigned char readPin(unsigned char pin);
	uint8_t readAll();
	/* return -1 if operation is not supported by pin, 1 if the state of the pin is the same, 0 for success*/
	void writePin(unsigned char t_pin, unsigned char t_value);
	void writePins(unsigned char *t_pins, unsigned char *t_values, unsigned char t_size);
	void setPin(unsigned char t_pin, bool t_direction);
	void resetValues();
private:
	void setValuePin(unsigned char t_pin, unsigned char t_value);
	unsigned char m_i2cAddress;
	unsigned char m_i2cDevice;
	bool m_pinDirection[8];
	uint8_t m_expanderState;
};
#endif
