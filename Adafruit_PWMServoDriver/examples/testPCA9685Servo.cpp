/*
 Servo test using adafruit PWM Servo Driver based on PCA9685.
 Copyright (C) 2020 Gabriel Dimitriu
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
#include <Adafruit_PWMServoDriver.h>
#include <stdio.h>

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

int main() {
	Adafruit_PWMServoDriver *pwm = new Adafruit_PWMServoDriver();
	pwm->begin();
	pwm->setOscillatorFrequency(27000000);
	pwm->setPWMFreq(SERVO_FREQ);
	uint8_t servoPin = 12;
	while(1) {
		for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
		    pwm->setPWM(servoPin, 0, pulselen);
		  }

		  delay(500);
		  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
		    pwm->setPWM(servoPin, 0, pulselen);
		  }

		  delay(500);

		  // Drive each servo one at a time using writeMicroseconds(), it's not precise due to calculation rounding!
		  // The writeMicroseconds() function is used to mimic the Arduino Servo library writeMicroseconds() behavior.
		  for (uint16_t microsec = USMIN; microsec < USMAX; microsec++) {
		    pwm->writeMicroseconds(servoPin, microsec);
		  }

		  delay(500);
		  for (uint16_t microsec = USMAX; microsec > USMIN; microsec--) {
		    pwm->writeMicroseconds(servoPin, microsec);
		  }

		  delay(500);
	}
	return 0;
}
