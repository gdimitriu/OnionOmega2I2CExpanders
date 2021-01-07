CC = mipsel-openwrt-linux-g++
INCLUDEPATHS = -I../../include -I$(PREFIX_I2C_EXP_DRIVER)/include 
LIBPATHS = -L$(PREFIX_I2C_EXP_DRIVER)/lib -L../../lib
LIBS = -loniondebug -lonioni2c -ladafruitpwmservo

all: testPCA9685Servo CalibrationServoPCA9685
	cp testPCA9685Servo ../../bin
	cp calibrationServoPCA9685 ../../bin
	
testPCA9685Servo : testPCA9685Servo.cpp 
	$(CC) -fPIC -o testPCA9685Servo testPCA9685Servo.cpp $(INCLUDEPATHS) $(LIBPATHS) $(LIBS)

CalibrationServoPCA9685 : CalibrationServoPCA9685.cpp
	$(CC) -fPIC -o calibrationServoPCA9685 CalibrationServoPCA9685.cpp $(INCLUDEPATHS) $(LIBPATHS) $(LIBS)
		
clean :
	if [ -f testPCA9685Servo ]; then rm testPCA9685Servo; fi;
	if [ -f ../../bin/calibrationServoPCA9685 ]; then rm ../../bin/calibrationServoPCA9685; fi;
	if [ -f calibrationServoPCA9685 ]; then rm calibrationServoPCA9685; fi;
	if [ -f ../../bin/testPCA9685Servo ]; then rm ../../bin/testPCA9685Servo; fi;	
	