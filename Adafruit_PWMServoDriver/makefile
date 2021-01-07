CC = mipsel-openwrt-linux-g++
INCLUDEPATHS = -I./ -I../include -I$(PREFIX_I2C_EXP_DRIVER)/include

all: Adafruit_PWMServoDriver
	cd examples && make

Adafruit_PWMServoDriver : Adafruit_PWMServoDriver.o 
	$(CC) -shared -fPIC -o libadafruitpwmservo.so Adafruit_PWMServoDriver.o
	cp libadafruitpwmservo.so ../lib
	cp Adafruit_PWMServoDriver.h ../include
	
Adafruit_PWMServoDriver.o : Adafruit_PWMServoDriver.cpp Adafruit_PWMServoDriver.h
	$(CC) -fPIC -c $(INCLUDEPATHS) Adafruit_PWMServoDriver.cpp
	
clean :
	if [ -f Adafruit_PWMServoDriver.o ]; then rm Adafruit_PWMServoDriver.o; fi;
	if [ -f libadafruitpwmservo.so ]; then rm libadafruitpwmservo.so; fi;
	if [ -f ../lib/libadafruitpwmservo.so ]; then rm ../lib/libadafruitpwmservo.so; fi;
	if [ -f ../include/Adafruit_PWMServoDriver.h ]; then rm ../include/Adafruit_PWMServoDriver.h; fi;
	cd examples; make clean