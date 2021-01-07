/*
PCF8591 Analog extender.
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

#ifndef PCF8591_h
#define PCF8591_h

typedef unsigned char uint8_t;

#define SINGLE_ENDED_INPUT 0b00000000
#define TREE_DIFFERENTIAL_INPUT 0b00010000
#define TWO_SINGLE_ONE_DIFFERENTIAL_INPUT 0b00100000
#define TWO_DIFFERENTIAL_INPUT 0b00110000
#define AUTOINCREMENT_READ 0b00000100
#define DISABLE_OUTPUT 0b01000000
class PCF8591
{
public:
	PCF8591(unsigned char t_i2cDev, unsigned char t_address);
	unsigned char getSinglePinValue(unsigned char t_pin);
	unsigned char* getAllPinsValue();
private:
	void sendControllByte(unsigned char t_pin, unsigned char t_type);
	unsigned char m_i2cAddress;
	unsigned char m_i2cDevice;
	unsigned char m_analog[4];
};
#endif
