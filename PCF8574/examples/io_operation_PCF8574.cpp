/*
PC8754 extender io operations.
extender on 0x21 for input
extender of 0x20 for output
on output all pins are link to leds
on input 1 pin to distance senzor
on input 2 pin to button brick
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
#include <unistd.h>
#include <stdio.h>
#include<onion-debug.h>

void inOrderLight(PCF8574 * expander)
{
	for (int i = 0; i < 8; i++) {
		if (i != 0) {
			expander->writePin(i - 1, 0);
		}
		expander->writePin(i, 1);
		sleep(1);
	}
	expander->writePin(7, 0);
	sleep(2);
	expander->resetValues();
}

int main()
{
//	onionSetVerbosity(ONION_VERBOSITY_EXTRA_VERBOSE);
	PCF8574 *inputExtender = new PCF8574(0, 0x24);
	PCF8574 *outputExtender = new PCF8574(0, 0x20);
	outputExtender->setPin(0, true);
	outputExtender->setPin(1, true);
	outputExtender->setPin(2, true);
	outputExtender->setPin(3, true);
	outputExtender->setPin(4, true);
	outputExtender->setPin(5, true);
	outputExtender->setPin(6, true);
	outputExtender->setPin(7, true);
	inputExtender->setPin(1, false);
	inputExtender->setPin(2, false);
	while (1) {
		if (inputExtender->readPin(2) == 1) {
			printf("read pin 2\n");
			inOrderLight(outputExtender);
		}
		if (inputExtender->readPin(1) == 0) {
			printf("read pin 1\n");
			outputExtender->resetValues();
			outputExtender->writePin(1, 1);
			sleep(1);
			outputExtender->resetValues();
		}
	}
}