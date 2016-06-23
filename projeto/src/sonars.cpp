/*
 * File:   main.cpp
 * Author: hans
 *
 * Created on 4 de Junho de 2015, 20:25
 */

#include <cstdlib>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <iomanip> 
#include "opencv2/highgui/highgui.hpp"

#include <wiringPi.h>
#include "robotAPI/Pins.h"
#include "robotAPI/Sonar.h"
#include "robotAPI/Motor.h"
#include "robotAPI/Encoder.h"
#include "robotAPI/KBAsync.h"

using namespace std;
using namespace cv;

bool running;
bool hasCamera = true;

//Detect ^C
void handleCTRLC(int s);
void setupCTRLCHandle();
void takePicture(VideoCapture *cap);

typedef enum {EXPLORE, OBSERVE, RUNAWAY} TRobotSate;

int main(int argc, char** argv) {

	cout << "Keyboard keys:" << endl;
	cout << "q - exit." << endl;
	cout << "p - take a picture." << endl;

	setupCTRLCHandle();

	if (PIN_MODE==PIN_BCM) {
		cout << "Pins in BCM mode." << endl;
		if (wiringPiSetupGpio()<0) {
			cout << "Could not setup GPIO pins" << endl;
			return -1;
		}
	} else {
		cout << "Pins in wiringPi mode." << endl;
		wiringPiSetup();
	}

	Sonar sonarFront, sonarLeft, sonarRight;
	sonarFront.setup(SONAR_FRONT_TRIGGER, SONAR_FRONT_ECHO);
	sonarLeft.setup(SONAR_LEFT_TRIGGER, SONAR_LEFT_ECHO);
	sonarRight.setup(SONAR_RIGHT_TRIGGER, SONAR_RIGHT_ECHO);

	KBAsync kb;
	int key;

	std::cout << std::setprecision(4) << std::fixed;
	while ((key=kb.getKey())!='q') {

		if (key>0)
		  cout << "Key " << key << " '" << (char)key << "' pressed." << endl;

		float dL, dF, dR;
		dL = sonarLeft.measureDistance();
		dF = sonarFront.measureDistance();
		dR = sonarRight.measureDistance();

	   cout << dL << " | " << dF << " | " << dR << "\r\n";

		delayMicroseconds(100000);
	}

	cout << "\r\nExiting...\r\n";

	return 0;
}

//Detect ^C
void handleCTRLC(int s) {
  running=false;
}

void setupCTRLCHandle() {
	struct sigaction sigIntHandler;

   sigIntHandler.sa_handler = handleCTRLC;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;

   sigaction(SIGINT, &sigIntHandler, NULL);
}

