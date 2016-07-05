#Compiler flags
CFLAGS       = -O2 -lpthread -fopenmp
WARNINGFLAGS = -Wno-unused-result
ROBOTFLAGS   = -lwiringPi
VREPFLAGS    = -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255
OPENCVFLAGS  = `pkg-config --cflags --libs opencv`
INCLUDEPATHS  = -I ./src -I ./src/robotAPI -I ./src/vrepAPI -I ./src/projectAPI

#Compile to run in the real robot
real: clean
	#Deactivate automatic white balance
	raspivid -awb off

	#Deactivate V-REP flag in projectAPI/RobotAPI.h
	sed -i -- 's/#define USING_VREP 1/#define USING_VREP 0/g' src/projectAPI/RobotAPI.h 

	#Compile robotAPI
	echo 'Compiling robotAPI...'
	cd src/robotAPI; \
	g++ -c *.cpp $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(ROBOTFLAGS) $(OPENCVFLAGS); \
	mv *.o ../../obj/; \
	cd ../..; \

	#Compile projectAPI
	echo 'Compiling projectAPI...'
	cd src/projectAPI; \
	g++ -c *.cpp $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(ROBOTFLAGS) $(OPENCVFLAGS);\
	mv *.o ../../obj/; \
	cd ../..; \

	#Generate Binaries
	echo 'Generating binaries...'
	g++ src/task1.cpp obj/*.o -o bin/task1.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(ROBOTFLAGS)
	g++ src/remoteControl.cpp obj/*.o -o bin/remoteControl.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(ROBOTFLAGS)
	g++ src/trainDetector.cpp obj/*.o -o bin/trainDetector.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(ROBOTFLAGS)
	g++ src/testDetector.cpp obj/*.o -o bin/testDetector.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(ROBOTFLAGS)
	g++ src/testSonar.cpp obj/*.o -o bin/testSonar.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(ROBOTFLAGS)
	g++ src/testCamera.cpp obj/*.o -o bin/testCamera.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(ROBOTFLAGS)
	g++ src/testLine.cpp obj/*.o -o bin/testLine.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(ROBOTFLAGS)
	g++ src/testSpeed.cpp obj/*.o -o bin/testSpeed.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(ROBOTFLAGS)
	echo 'Done.'

#Compile to run in V-REP simulation
sim: clean
	#Activate V-REP flag in projectAPI/RobotAPI.h
	sed -i -- 's/#define USING_VREP 0/#define USING_VREP 1/g' src/projectAPI/RobotAPI.h 

	#Compile vrepAPI
	echo 'Compiling vrepAPI...'
	cd src/vrepAPI; \
	gcc -c *.c $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(VREPFLAGS) $(OPENCVFLAGS); \
	mv *.o ../../obj/; \
	cd ../..; \

	#Compile projectAPI
	echo 'Compiling projectAPI...'
	cd src/projectAPI; \
	g++ -c *.cpp $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(VREPFLAGS) $(OPENCVFLAGS); \
	mv *.o ../../obj/; \
	cd ../..; \

	#Generate Binaries
	echo 'Generating binaries...'
	g++ src/task1.cpp obj/*.o -o bin/task1.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(VREPFLAGS)
	g++ src/remoteControl.cpp obj/*.o -o bin/remoteControl.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(VREPFLAGS)
	g++ src/trainDetector.cpp obj/*.o -o bin/trainDetector.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(VREPFLAGS)
	g++ src/testDetector.cpp obj/*.o -o bin/testDetector.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(VREPFLAGS)
	g++ src/testSonar.cpp obj/*.o -o bin/testSonar.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(VREPFLAGS)
	g++ src/testCamera.cpp obj/*.o -o bin/testCamera.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(VREPFLAGS)
	g++ src/testLine.cpp obj/*.o -o bin/testLine.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(VREPFLAGS)
	g++ src/testSpeed.cpp obj/*.o -o bin/testSpeed.bin $(INCLUDEPATHS) $(CFLAGS) $(WARNINGFLAGS) $(OPENCVFLAGS) $(VREPFLAGS)
	echo 'Done.'

#Clear binary and object files
clean:
	echo 'Cleaning temporary files...'
	mkdir -p bin
	mkdir -p obj
	mkdir -p img/pics
	rm -f bin/*.*
	rm -f obj/*.o
	rm -f img/pics/*.jpg

#Supress echo in makefile
.SILENT: