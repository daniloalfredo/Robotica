CFLAGS     = -O2 -Wall -I/usr/local/lib
ROBOTFLAGS = -lwiringPi
VREPFLAGS  = -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255

#Compile to run in the real robot
real: 
	#Remove unnecessary files
	@mkdir -p bin
	@rm -f bin/*.*
	@rm -f obj/*.o 

	#Compile robotAPI
	cd src/robotAPI; \
	g++ $(CFLAGS) $(ROBOTFLAGS) -c *.cpp; \
	mv *.o ../../obj/; \
	cd ../..; \

	#Compile projectAPI
	cd src/projectAPI; \
	g++ $(CFLAGS) $(ROBOTFLAGS) -c *.cpp; \
	mv *.o ../../obj/; \
	cd ../..; \

	#Generate Binaries
	g++ src/Task1.cpp obj/*.o -o bin/task1.bin -lpthread
	
	#Remove generated object files
	@rm -f obj/*.o 

#Compile to run in V-REP simulation
sim:
	#Remove unnecessary files
	@mkdir -p bin
	@rm -f bin/*.*
	@rm -f obj/*.o 

	#Compile vrepAPI
	cd src/vrepAPI; \
	gcc $(CFLAGS) $(VREPFLAGS) -c *.c; \
	mv *.o ../../obj/; \
	cd ../..; \

	#Compile projectAPI
	cd src/projectAPI; \
	g++ $(CFLAGS) $(VREPFLAGS) -c *.cpp; \
	mv *.o ../../obj/; \
	cd ../..; \

	#Generate Binaries
	g++ src/Task1.cpp obj/*.o -o bin/task1.bin -lpthread
	
	#Remove generated object files
	@rm -f obj/*.o 

clean:
	@rm -f bin/*.*
	@rm -f obj/*.o 