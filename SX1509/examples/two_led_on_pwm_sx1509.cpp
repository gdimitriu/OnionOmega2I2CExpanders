/*
SX1509 extender example how to use.
Copyright (C) 2029 Gabriel Dimitriu
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

#include <SX1509.h>
#include <unistd.h>

#define FIRST_LED_PIN 0
#define SECOND_LED_PIN 1

int main() {
	SX1509 *extender = new SX1509();
	extender->begin(0x3E);
	extender->clock(INTERNAL_CLOCK_2MHZ, 7);
	extender->pinMode(FIRST_LED_PIN, ANALOG_OUTPUT);
	extender->pinMode(SECOND_LED_PIN, ANALOG_OUTPUT);
	while(1)
	{
		for (int i = 0; i < 256; i++)
		{
			extender->analogWrite(FIRST_LED_PIN, i);
			extender->analogWrite(SECOND_LED_PIN, 255-i);
			usleep(20*1000);
		}
		sleep(1);
		for (int i = 255; i >= 0; i--)
		{
			extender->analogWrite(FIRST_LED_PIN, i);
			extender->analogWrite(SECOND_LED_PIN, 255-i);
			usleep(20*1000);
		}
		sleep(1);
	}
}
