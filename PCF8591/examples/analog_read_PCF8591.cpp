/*
PCF8591 Analog extender example how to use.
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

#include<PCF8591.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
	if (argc != 2) {
		fprintf(stderr, "usage: %s address", argv[0]);
		exit(-1);
	}
	PCF8591 *analogExpander = new PCF8591(0, strtol(argv[1],NULL, 0));
	printf("Analog read from 0 has value=%d\n", analogExpander->getSinglePinValue(0));
	printf("Analog read from 1 has value=%d\n", analogExpander->getSinglePinValue(1));
	printf("Analog read from 2 has value=%d\n", analogExpander->getSinglePinValue(2));
	printf("Analog read from 3 has value=%d\n", analogExpander->getSinglePinValue(3));
	printf("Now read all in batch values\n\n");
	fflush(stdout);
	for (int j = 0; j < 10 ; j++) {
		unsigned char *values = analogExpander->getAllPinsValue();
		for (int i = 0; i < 4; i++) {
			printf("Analog read from %d has value=%d\n", i, values[i]);
		}
		sleep(1);
	}
}
