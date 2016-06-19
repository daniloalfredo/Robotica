CFLAGS = -I vrepAPI -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	CFLAGS += -D__linux
else
	CFLAGS += -D__APPLE__
endif

all: 
	@rm -f bin/*.*
	@rm -f *.o 
	@rm -f vrepAPI/*.o 
	g++ $(CFLAGS) -c Utils.cpp -o Utils.o
	g++ $(CFLAGS) -c RobotAPI.cpp -o RobotAPI.o
	g++ $(CFLAGS) -c EnvMap.cpp -o EnvMap.o
	g++ $(CFLAGS) -c Robot.cpp -o Robot.o
	g++ $(CFLAGS) -c Task1.cpp -o Task1.o
	gcc $(CFLAGS) -c vrepAPI/extApi.c -o vrepAPI/extApi.o
	gcc $(CFLAGS) -c vrepAPI/extApiPlatform.c -o vrepAPI/extApiPlatform.o
	@mkdir -p bin
	g++ vrepAPI/extApi.o vrepAPI/extApiPlatform.o Task1.o Utils.o RobotAPI.o EnvMap.o Robot.o -o bin/task1.bin -lpthread
	@rm -f *.o 
	@rm -f vrepAPI/*.o 

clean:
	@rm -f bin/*.*
	@rm -f *.o 
	@rm -f vrepAPI/*.o 
