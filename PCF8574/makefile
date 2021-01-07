CC = mipsel-openwrt-linux-g++

INCLUDEPATHS = -I./ -I../include -I$(PREFIX_I2C_EXP_DRIVER)/include
EXECUTABLES=libpcf8574.so
OBJECTS=PCF8574.o
HEADERS= PCF8574.h

all: $(EXECUTABLES)
	cd examples && make

libpcf8574.so : PCF8574.o 
	$(CC) -shared -fPIC -o libpcf8574.so PCF8574.o
	cp libpcf8574.so ../lib
	
PCF8574.o : PCF8574.cpp PCF8574.h
	$(CC) -fPIC -c $(INCLUDEPATHS) PCF8574.cpp
	cp PCF8574.h ../include
	
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
	cd examples && make clean
	
