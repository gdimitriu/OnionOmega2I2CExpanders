/*
 Calibrations with sg9 servo and Onion PWM expander
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
#include <pwm-exp.h>

#ifndef CALIBRATIONSERVOPWMExp_H_
#define CALIBRATIONSERVOPWMExp_H_


class CalibrationServoPWMExp {
public:
	CalibrationServoPWMExp(uint8_t t_pinNumber);
	virtual ~CalibrationServoPWMExp();
	void start();
private:
	void printMenu();
	void resetConfiguration();
	uint8_t m_pinNumber;
	float m_servoMin;
	float m_servoMax;
	float m_servoFront;
	float m_servoFreq;
	uint32_t m_oscFreq;
};

#endif /* CALIBRATIONSERVOPWMExp_H_ */
