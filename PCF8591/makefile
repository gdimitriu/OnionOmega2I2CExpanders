CC = mipsel-openwrt-linux-g++
INCLUDEPATHS = -I./  -I$(PREFIX_I2C_EXP_DRIVER)/include
EXECUTABLES=libppcf8591.so
OBJECTS=PCF8591.o
HEADERS=PCF8591.h

all: $(EXECUTABLES)
	cd examples && make

libppcf8591.so : PCF8591.o 
	$(CC) -shared -fPIC -o libpcf8591.so PCF8591.o
	cp libpcf8591.so ../lib
	
PCF8591.o : PCF8591.cpp PCF8591.h
	$(CC) -fPIC -c $(INCLUDEPATHS) PCF8591.cpp
	cp PCF8591.h ../include

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