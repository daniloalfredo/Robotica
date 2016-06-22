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

	Sonar sonar;
	Encoder encoderL, encoderR;
	Motor motorL, motorR;

	encoderL.setup(ENCODER_LEFT, LEFT_SIDE);
	encoderR.setup(ENCODER_RIGHT, RIGHT_SIDE);
	sonar.setup(SONAR_FRONT_TRIGGER, SONAR_FRONT_ECHO);
	motorL.setup(MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_LEFT_E, &encoderL);
	motorR.setup(MOTOR_RIGHT_A, MOTOR_RIGHT_B, MOTOR_RIGHT_E, &encoderR);
	motorL.pid.setKp(0.1);
	motorR.pid.setKp(0.1);

	VideoCapture cap(0); // open the video camera no. 0
	if (cap.isOpened()) {
		hasCamera = true;
		cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
		double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
		double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
		cout << "Camera frame size : " << dWidth << " x " << dHeight << endl;
	} else {
		cout << "Cannot open the video cam" << endl;
		hasCamera = false;
	} 


	float fdist = 40;
	float filterWeight = 0.6;

	TRobotSate currentState=OBSERVE, oldState=OBSERVE;
	running = true;
	KBAsync kb;
	int key;

	while ((key=kb.getKey())!='q') {

		if (key>0)
		  cout << "\r\n" << "Key " << key << " '" << (char)key << "' pressed." << endl;

		if (key=='p') takePicture(&cap);

		float distance = sonar.measureDistance();

		if (distance>=0) {
			oldState = currentState;

			fdist = filterWeight*fdist + (1-filterWeight)*distance;

			cout << "\r" << flush << "Distance:" << fdist << "cm";
			cout << "\tEncoders L: " << encoderL.getSteps() << ", R: " << encoderR.getSteps();  

			if (fdist < 30) {
				currentState = RUNAWAY;
			} else if (fdist >= 30 && fdist <= 50) {
				currentState = OBSERVE;
			} else {
				currentState = EXPLORE;
			}

			if (currentState!=oldState) 
			switch (currentState) {
				case OBSERVE:
					cout << "\r\nObserve..." << endl;
					motorL.stop();
					motorR.stop();
					delayMicroseconds(200000);
					takePicture(&cap);
				break;
				case EXPLORE:
					cout << "\r\nExplore..." << endl;
					motorL.setTargetSpeed(8);
					motorR.setTargetSpeed(10);
				break;
				case RUNAWAY:
					cout << "\r\nToo close, runnaway!!!" << endl;
					motorL.setTargetSpeed(-20);
					motorR.setTargetSpeed(-20);
				break;
			}

		} else
	 		cout << "\rSonar returned: " << distance;

		motorL.controlSpeed();
		motorR.controlSpeed();

		delayMicroseconds(100000);
	}

	motorL.stop();
	motorR.stop();
	cout << "\r\nExiting...\r\n";

	return 0;
}

#define OPENCV_BUFFER_SIZE 5
void takePicture(VideoCapture *cap) {
   if (hasCamera) {
		static int pictureCount = 1;
		cout << "\rTaking picture " << pictureCount << "...";
		stringstream ss;
		ss << "./pictures/picture" << pictureCount << ".png";

		Mat frame;
		bool bSuccess = true;

		//Clear the video buffer in order to get the a recent frame
		//only required for exporadic captures
      for (int i=0; i<OPENCV_BUFFER_SIZE && bSuccess; i++)
			bSuccess = cap->read(frame);

		// read a new frame from video
		bSuccess = cap->read(frame); 

		//now "frame" stores our image, writ it to the file.
		if (bSuccess && imwrite(ss.str(), frame)){
			cout << "ok." << endl;
			pictureCount++;
		} else
			cout << "error." << endl;
	}
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

