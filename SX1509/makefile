CC = mipsel-openwrt-linux-g++
INCLUDEPATHS = -I./ -I../include -I$(PREFIX_I2C_EXP_DRIVER)/include
OBJECTS=SX1509.o
EXECUTABLES= libsx1509.so
HEADERS=SX1509.h

all: $(EXECUTABLES)
	cd examples && make

libsx1509.so:SX1509.o 
	$(CC) -shared -fPIC -o libsx1509.so SX1509.o
	cp libsx1509.so ../lib
	
SX1509.o:SX1509.cpp SX1509.h
	$(CC) -fPIC -c $(INCLUDEPATHS) SX1509.cpp
	cp SX1509.h ../include
	
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
	cd examples; make clean