CC = mipsel-openwrt-linux-g++
INCLUDEPATHS = -I../../include -I$(PREFIX_I2C_EXP_DRIVER)/include
LIBPATHS = -L$(PREFIX_I2C_EXP_DRIVER)/lib -L../../lib
LIBS = -loniondebug -lonioni2c -lsx1509
EXECUTABLES=two_led_on_pwm_sx1509 three_led_on_pwm_sx1509 
all: $(EXECUTABLES)
	
two_led_on_pwm_sx1509 : two_led_on_pwm_sx1509.cpp
	$(CC) -fPIC -o two_led_on_pwm_sx1509 two_led_on_pwm_sx1509.cpp $(INCLUDEPATHS) \
	$(LIBPATHS) $(LIBS)
	cp two_led_on_pwm_sx1509 ../../bin
	
	
three_led_on_pwm_sx1509 : three_led_on_pwm_sx1509.cpp
	$(CC) -fPIC -o three_led_on_pwm_sx1509 three_led_on_pwm_sx1509.cpp $(INCLUDEPATHS) \
	$(LIBPATHS) $(LIBS)
	cp three_led_on_pwm_sx1509 ../../bin	
	
clean :
	for object in $(EXECUTABLES);\
	do \
		if [ -f $$object ]; then rm $$object; fi; \
		if [ -f ../../bin/$$object ]; then rm ../../bin/$$object; fi; \
	done
	for object in $(OBJECTS);\
	do \
		if [ -f $$object ]; then rm $$object; fi; \
	done
	for object in $(HEADERS);\
	do \
		if [ -f ../include/$$object ]; then rm ../include/$$object; fi; \
	done
	
