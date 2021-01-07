all:
	cd Adafruit_PWMServoDriver && make
	cd SX1509 && make
	cd RegisterOperations && make
	cd PCF8574 && make
    
clean:
	cd Adafruit_PWMServoDriver && make clean
	cd SX1509 && make clean
	cd RegisterOperations && make clean
	cd PCF8574 && make clean
	