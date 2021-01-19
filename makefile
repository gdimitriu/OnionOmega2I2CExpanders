all:
	cd Adafruit_PWMServoDriver && make
	cd SX1509 && make
	cd RegisterOperations && make
	cd PCF8574 && make
	cd PCF8591 && make
	cd poc-tests && make
    
clean:
	cd Adafruit_PWMServoDriver && make clean
	cd SX1509 && make clean
	cd RegisterOperations && make clean
	cd PCF8574 && make clean
	cd PCF8591 && make clean
	cd poc-tests && make clean
	
