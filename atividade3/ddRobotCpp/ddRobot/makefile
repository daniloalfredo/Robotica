CFLAGS = -I../remoteApi -I../include -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	CFLAGS += -D__linux
else
	CFLAGS += -D__APPLE__
endif

all: 
	@rm -f bin/*.*
	@rm -f *.o 
	@rm -f ../remoteApi/*.o 
	g++ $(CFLAGS) -c Utils.cpp -o Utils.o
	g++ $(CFLAGS) -c EnvMap.cpp -o EnvMap.o
	g++ $(CFLAGS) -c Robot.cpp -o Robot.o
	g++ $(CFLAGS) -c ddRobot.cpp -o ddRobot.o
	gcc $(CFLAGS) -c ../remoteApi/extApi.c -o ../remoteApi/extApi.o
	gcc $(CFLAGS) -c ../remoteApi/extApiPlatform.c -o ../remoteApi/extApiPlatform.o
	@mkdir -p bin
	g++ ../remoteApi/extApi.o ../remoteApi/extApiPlatform.o ddRobot.o Utils.o EnvMap.o Robot.o -o bin/ddRobot.bin -lpthread
	@rm -f *.o 

clean:
	@rm -f bin/*.*
	@rm -f *.o 
	@rm -f ../remoteApi/*.o 
