/*
Register operations.
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

#include<RegisterOperations.h>

void setBit(unsigned char *t_registerValue, unsigned char t_position, unsigned char t_value) {
	if (t_position > 7) {
		return;
	}
	unsigned char bitPossion = 1;
	if (t_position != 0 )
		bitPossion = 1 << t_position;
	unsigned char currentValue = getBit(*t_registerValue, t_position);
	if (t_value==1 && currentValue == 0)
		*t_registerValue += bitPossion;
	else if (t_value == 0 && currentValue == 1)
		*t_registerValue -= bitPossion;
}

unsigned char getBit(unsigned char t_registerValue, unsigned char t_position){
	if (t_position > 7) {
		return 0;
	}
	unsigned char value;
	value = t_registerValue >> t_position;
	value = value << 7;
	value = value >> 7;
	return value;
}