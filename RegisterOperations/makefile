CC = mipsel-openwrt-linux-g++

all: registeroperations

registeroperations : RegisterOperations.o 
	$(CC) -shared -fPIC -o libregisteroperations.so RegisterOperations.o
	cp libregisteroperations.so ../lib
	cp RegisterOperations.h ../include
	
RegisterOperations.o : RegisterOperations.c RegisterOperations.h
	$(CC) -fPIC -c -I ./ RegisterOperations.c
	

clean :
	if [ -f libregisteroperations.so ]; then rm libregisteroperations.so; fi;
	if [ -f RegisterOperations.o ]; then rm RegisterOperations.o; fi;
	if [ -f ../lib/libregisteroperations.so ]; then rm ../lib/libregisteroperations.so; fi;
	if [ -f ../include/RegisterOperations.h ]; then rm ../include/RegisterOperations.h; fi;
