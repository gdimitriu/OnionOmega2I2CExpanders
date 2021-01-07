CC = mipsel-openwrt-linux-g++
INCLUDEPATHS = -I../../include -I$(PREFIX_I2C_EXP_DRIVER)/include
LIBPATHS = -L$(PREFIX_I2C_EXP_DRIVER)/lib -L../../lib
LIBS = -loniondebug -lonioni2c -lregisteroperations -lpcf8574
EXECUTABLES= led_power_on_off_PCF8574 io_operation_PCF8574
all: $(EXECUTABLES)

led_power_on_off_PCF8574 : led_power_on_off_PCF8574.cpp
	$(CC) -fPIC -o led_power_on_off_PCF8574 led_power_on_off_PCF8574.cpp $(INCLUDEPATHS) \
	$(LIBPATHS) $(LIBS)
	cp led_power_on_off_PCF8574 ../../bin

io_operation_PCF8574 : io_operation_PCF8574.cpp
	$(CC) -fPIC -o io_operation_PCF8574 io_operation_PCF8574.cpp $(INCLUDEPATHS) \
	$(LIBPATHS) $(LIBS)
	cp io_operation_PCF8574 ../../bin
	
clean :
	for object in $(EXECUTABLES);\
	do \
		if [ -f $$object ]; then rm $$object; fi; \
		if [ -f ../../bin/$$object ]; then rm ../../bin/$$object; fi; \
	done
