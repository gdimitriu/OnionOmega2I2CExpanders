CC = mipsel-openwrt-linux-g++
INCLUDEPATHS = -I../../include -I$(PREFIX_I2C_EXP_DRIVER)/include
LIBPATHS = -L$(PREFIX_I2C_EXP_DRIVER)/lib -L../../lib
LIBS = -loniondebug -lonioni2c -lpcf8591

all: analog_read_PCF8591
	cp analog_read_PCF8591 ../../bin/
	
analog_read_PCF8591 : analog_read_PCF8591.cpp
	$(CC) -fPIC -o analog_read_PCF8591 analog_read_PCF8591.cpp $(INCLUDEPATHS) \
	$(LIBPATHS) $(LIBS)
	
clean :
	if [ -f analog_read_PCF8591 ]; then rm analog_read_PCF8591; fi;
	if [ -f ../../bin/analog_read_PCF8591 ]; then rm ../../bin/analog_read_PCF8591; fi;
