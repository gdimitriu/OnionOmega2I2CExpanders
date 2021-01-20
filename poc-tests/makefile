CC = mipsel-openwrt-linux-g++
INCLUDEPATHS = -I./ -I../include -I$(PREFIX_I2C_EXP_DRIVER)/include
LIBS=-loniondebug -lonioni2c -lonionpwmexp -L$(PREFIX_I2C_EXP_DRIVER)/lib
OBJECTS= CalibrationServoPWMExp.o
EXECUTABLES= CalibrationServoPWMExp
HEADERS=CalibrationServoPWMExp.h

all: $(EXECUTABLES)

CalibrationServoPWMExp.o:CalibrationServoPWMExp.cpp CalibrationServoPWMExp.h
	$(CC) -fPIC -c CalibrationServoPWMExp.cpp $(INCLUDEPATHS)
	
CalibrationServoPWMExp:CalibrationServoPWMExp.o
	$(CC) -o CalibrationServoPWMExp CalibrationServoPWMExp.o $(LIBS)
			
clean :
	for object in $(EXECUTABLES);\
	do \
		if [ -f $$object ]; then rm $$object; fi; \
		if [ -f ../lib/$$object ]; then rm ../lib/$$object; fi; \
	done
	for object in $(OBJECTS);\
	do \
		if [ -f $$object ]; then rm $$object; fi; \
	done
	for object in $(HEADERS);\
	do \
		if [ -f ../include/$$object ]; then rm ../include/$$object; fi; \
	done
