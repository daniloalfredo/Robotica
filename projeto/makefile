ifeq (${mode},debug)  
   MODEFLAGS = -g
endif

ifeq (${mode},release)
   MODEFLAGS = -O2
endif

CFLAGS   = $(MODEFLAGS) -Wall -I. -I/usr/local/lib
VREPFLAGS = -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255

ROBOTAPIDIR = ./src/robotAPI
ROBOTAPISOURCES := $(wildcard $(ROBOTAPIDIR)/*.cpp)
RAPIINCLUDES := $(wildcard $(ROBOTAPIDIR)/*.h)
RAPIOBJS := $(.obj/$(RAPISOURCES:.cpp=.o))

all: 
	@mkdir -p bin
	@rm -f bin/*.*
	@rm -f obj/*.o 

	g++ $(CFLAGS) -c src/Utils.cpp -o Utils.o
	g++ $(CFLAGS) -c src/RobotAPI.cpp -o RobotAPI.o
	g++ $(CFLAGS) -c src/EnvMap.cpp -o EnvMap.o
	g++ $(CFLAGS) -c src/Robot.cpp -o Robot.o
	g++ $(CFLAGS) -c src/Task1.cpp -o Task1.o

	g++ $(CFLAGS) -c src/robotAPI/Encoder.cpp -o obj/Encoder.o -lwiringPi
	g++ $(CFLAGS) -c src/robotAPI/KBAsync.cpp -o obj/KBAsync.o -lwiringPi
	g++ $(CFLAGS) -c src/robotAPI/MotionControl.cpp -o obj/MotionControl.o -lwiringPi
	g++ $(CFLAGS) -c src/robotAPI/Motor.cpp -o obj/Motor.o -lwiringPi
	g++ $(CFLAGS) -c src/robotAPI/Odometry.cpp -o obj/Odometry.o -lwiringPi
	g++ $(CFLAGS) -c src/robotAPI/PIDController.cpp -o obj/PIDController.o -lwiringPi
	g++ $(CFLAGS) -c src/robotAPI/Pins.cpp -o obj/Pins.o -lwiringPi
	g++ $(CFLAGS) -c src/robotAPI/RobotTimer.cpp -o obj/RobotTimer.o -lwiringPi
	g++ $(CFLAGS) -c src/robotAPI/Sonar.cpp -o obj/Sonar.o -lwiringPi

	g++  obj/Encoder.o obj/KBAsync.o obj/MotionControl.o obj/Motor.o obj/Odometry.o obj/PIDController.o obj/Pins.o obj/RobotTimer.o obj/Sonar.o obj/Utils.o obj/RobotAPI.o obj/EnvMap.o obj/Robot.o obj/Task1.o -o bin/task1.bin -lpthread

	@rm -f obj/*.o 

sim:
	@mkdir -p bin
	@rm -f bin/*.*
	@rm -f obj/*.o 

	g++ $(CFLAGS) -c src/Utils.cpp -o obj/Utils.o
	g++ $(CFLAGS) -c src/EnvMap.cpp -o obj/EnvMap.o
	g++ $(CFLAGS) -c src/Robot.cpp -o obj/Robot.o
	g++ $(CFLAGS) -c src/Task1.cpp -o obj/Task1.o

	g++ $(CFLAGS) $(VREPFLAGS) -c src/RobotAPI.cpp -o obj/RobotAPI.o
	gcc $(VREPFLAGS) -c src/vrepAPI/extApi.c -o obj/extApi.o
	gcc $(VREPFLAGS) -c src/vrepAPI/extApiPlatform.c -o obj/extApiPlatform.o
	
	g++ obj/extApi.o obj/extApiPlatform.o obj/Utils.o obj/RobotAPI.o obj/EnvMap.o obj/Robot.o obj/Task1.o -o bin/task1.bin -lpthread

	@rm -f obj/*.o 

clean:
	@rm -f bin/*.*
	@rm -f obj/*.o 