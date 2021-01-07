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

#include "PCF8591.h"
#include<onion-i2c.h>
#include<stdlib.h>

PCF8591::PCF8591(unsigned char t_i2cDev, unsigned char t_address)
{
	this->m_i2cDevice = t_i2cDev;
	this->m_i2cAddress = t_address;
}

void PCF8591::sendControllByte(unsigned char t_pin, unsigned char t_type)
{
	unsigned char controllByte = t_pin | t_type | DISABLE_OUTPUT;
	i2c_writeBufferRaw(m_i2cDevice, m_i2cAddress, &controllByte, 1);
}

unsigned char PCF8591::getSinglePinValue(unsigned char t_pin)
{
	if (t_pin > 3) {
		return 0;
	}
	sendControllByte(t_pin, SINGLE_ENDED_INPUT);
	unsigned char tmp;
	//read control register
	i2c_readRaw(m_i2cDevice, m_i2cAddress, &tmp, 1);
	i2c_readRaw(m_i2cDevice, m_i2cAddress, &tmp, 1);
	return tmp;
}

unsigned char* PCF8591::getAllPinsValue()
{
	unsigned char controllByte = 4 | DISABLE_OUTPUT;
	i2c_writeBufferRaw(m_i2cDevice, m_i2cAddress, &controllByte, 1);
	unsigned char tmp;
	//read control register
	i2c_readRaw(m_i2cDevice, m_i2cAddress, &tmp, 1);
	//read all registers
	for (int i = 0; i < 4; i++) {
		i2c_readRaw(m_i2cDevice, m_i2cAddress, &m_analog[i], 1);
	}
	return m_analog;
}
