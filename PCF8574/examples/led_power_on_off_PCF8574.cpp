/*
PC8754 extender operations test.
This will power on the leds in order 0,1,2,3,4,5,6,7
This will power on the odd leds.
This will power off all leds.
This will power on the 0,1,2 leds.
If the led are in sourcing mode is the oposite.

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

#include <PCF8574.h>
#include<unistd.h>
#include <stdio.h>

int main(int argc, char *argv[]) {
	if (argc != 2) {
		fprintf(stderr, "usage: %s address", argv[0]);
		exit(-1);
	}
	PCF8574 *expander = new PCF8574(0, strtol(argv[1],NULL, 0));
	expander->setPin(0, true);
	expander->setPin(1, true);
	expander->setPin(2, true);
	expander->setPin(3, true);
	expander->setPin(4, true);
	expander->setPin(5, true);
	expander->setPin(6, true);
	expander->setPin(7, true);
	for (int i = 0; i < 8; i++) {
		if (i != 0) {
			expander->writePin(i - 1, 0);
		}
		expander->writePin(i, 1);
		sleep(1);
	}
	expander->writePin(7, 0);
	sleep(2);
	for (int i = 0; i < 8; i++) {
		if (i%2 == 0) {
			expander->writePin(i, 1);
		}
	}
	sleep(2);
	expander->resetValues();
	sleep(2);
	unsigned char pins[3] = { 0,1, 2 };
	unsigned char values[3] = { 1, 1, 1};
	expander->writePins(pins, values, 3);
	sleep(2);
}
