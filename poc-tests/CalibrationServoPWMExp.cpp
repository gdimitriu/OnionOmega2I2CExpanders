/*
 Calibrations with sg9 servo and Onion PWM Expansion
 Copyright (C) 2021 Gabriel Dimitriu
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

#include "CalibrationServoPWMExp.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pwm-exp.h>

int main(int argc, char **argv) {
	if (argc != 2) {
		printf(
				"Usage %s <pin number of the servo>\n",argv[0]);
		return -1;
	}
	CalibrationServoPWMExp *servo = new CalibrationServoPWMExp(atoi(argv[1]));
	servo->start();
}

CalibrationServoPWMExp::~CalibrationServoPWMExp() {
}

CalibrationServoPWMExp::CalibrationServoPWMExp(uint8_t t_pinNumber) {

	int status  = pwmDriverInit();
	if (status != 0) {
		printf("PWM expander not initialized\n");
		exit(-1);
	}
	m_pinNumber = t_pinNumber;
	m_servoFreq = 50.0;
	m_servoMin = 1.5;
	m_servoMax = 13.5;
	m_servoFront = 8.0;
	m_oscFreq = 27000000;
	if (pwmSetFrequency(m_servoFreq) != 0) {
		printf("Could not set frequency to %u\n", m_servoFreq);
	}
	resetConfiguration();
}

void CalibrationServoPWMExp::resetConfiguration() {
	if (pwmSetFrequency(m_servoFreq) != 0) {
		printf("Could not set frequency to %u\n", m_servoFreq);
	}
}

void CalibrationServoPWMExp::printMenu() {
	printf("Calibration of servo using onion PWM extender\n");
	printf("\to - oscillator frequency [oxxx]\n");
	printf("\ts - servo frequency [sxxx.xx]\n");
	printf("\tl - rotate 90 degree left [l<xxx.xx>] where xxx.xx is the duty cycle\n");
	printf("\tr - rotate 90 degree right[r<xxx.xx>] where xxx.xx is the duty cycle\n");
	printf("\tf - front position [f<xxx.xx>] where xxx.xx is the duty cycle\n");
	printf("\td - rotate xxx degree [dxxx]\n");
	printf("\tv - input manual the value [vxxx.xx] where xxx.xx is the duty cycle\n");
	printf("\tq  - quit test\n");
	fflush(stdout);
}

void CalibrationServoPWMExp::start() {
	char *last;
	int iValue;
	long lValue;
	float fValue;
	printMenu();
	std::string inputParameter;
	while (1) {
		std::getline(std::cin, inputParameter);
		if (!inputParameter.empty()) {
			switch (inputParameter.front()) {
			case 'q':
				pwmDisableChip ();
				return;
			case 'o':
				inputParameter.erase(0, 1);
				lValue = strtol(inputParameter.c_str(), &last, 10);
				if (lValue > 0) {
					printf("Setting the Oscillator frequency from %d to %d\n",
							m_oscFreq, lValue);
					fflush(stdout);
					m_oscFreq = lValue;
					resetConfiguration();
				} else {
					printf(
							"Oscillator frequency should be greater than 0 but it was %s\n",
							inputParameter.c_str());
					fflush(stdout);
					printMenu();
				}
				break;
			case 's':
				inputParameter.erase(0, 1);
				fValue = strtof(inputParameter.c_str(), &last);
				if (fValue > 0) {
					printf("Setting the pwm servo frequency from %f to %f\n",
							m_servoFreq, fValue);
					fflush(stdout);
					m_servoFreq = fValue;
					resetConfiguration();
				} else {
					printf(
							"Servo frequency should be greater than 0 but it was %s\n",
							inputParameter.c_str());
					fflush(stdout);
					printMenu();
				}
				break;
			case 'l':
				inputParameter.erase(0, 1);
				if (!inputParameter.empty()) {
					fValue = atof(inputParameter.c_str());
					if (fValue > 0) {
						printf("Set servo minimum from %f to %f\n", m_servoMin,
								fValue);
						fflush(stdout);
						m_servoMin = fValue;
					} else {
						printf(
								"Minimum value should be greater than 0 but it was %s",
								inputParameter.c_str());
						fflush(stdout);
						break;
					}
				}
				pwmSetFrequency(m_servoFreq);
				pwmSetupDriver(m_pinNumber, m_servoMin, 0);
				break;
			case 'r':
				inputParameter.erase(0, 1);
				if (!inputParameter.empty()) {
					fValue = atof(inputParameter.c_str());
					if (fValue > 0) {
						printf("Set servo maximum from %f to %f\n", m_servoMax,
								fValue);
						fflush(stdout);
						m_servoMax = fValue;
					} else {
						printf(
								"Maximum value should be greater than 0 but it was %s",
								inputParameter.c_str());
						fflush(stdout);
						printMenu();
						break;
					}
				}
				pwmSetFrequency(m_servoFreq);
				pwmSetupDriver(m_pinNumber,m_servoMax, 0);
				break;
			case 'f':
				inputParameter.erase(0, 1);
				if (!inputParameter.empty()) {
					fValue = atof(inputParameter.c_str());
					if (fValue > 0) {
						printf("Set servo front from %f to %f\n", m_servoFront,
								fValue);
						fflush(stdout);
						m_servoFront = fValue;
					} else {
						printf(
								"Servo front value should be greater than 0 but it was %s",
								inputParameter.c_str());
						fflush(stdout);
						printMenu();
						break;
					}
				}
				pwmSetFrequency(m_servoFreq);
				pwmSetupDriver(m_pinNumber, m_servoFront, 0);
				break;
			case 'v':
				inputParameter.erase(0, 1);
				fValue = atof(inputParameter.c_str());//, &last, 10);
				if (fValue < 0) {
					printf(
							"Movement should be positive between 0 and 180 but it was %s\n",
							inputParameter.c_str());
					fflush(stdout);
					printMenu();
				} else {
					pwmSetFrequency(m_servoFreq);
					pwmSetupDriver(m_pinNumber,fValue,0);
				}
				break;
			case 'd':
				inputParameter.erase(0, 1);
				iValue = strtol(inputParameter.c_str(), &last, 10);
				if (iValue > 0) {
					pwmSetupDriver(m_pinNumber,
							m_servoMin
									+ ((m_servoMax - m_servoMin) / 180) * iValue, 0);
				} else {
					printf(
							"Rotation angle should be positive between 0 and 180 but it was %s\n",
							inputParameter.c_str());
					fflush(stdout);
					printMenu();
				}
				break;
			default:
				printMenu();
			}
		}
	}
}

