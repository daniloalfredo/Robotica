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
#include <vector>
#include "opencv2/highgui/highgui.hpp"

#include <wiringPi.h>
#include "robotAPI/Pins.h"
#include "robotAPI/Motor.h"
#include "robotAPI/Encoder.h"
#include "robotAPI/Odometry.h"
#include "robotAPI/KBAsync.h"

#define nl "\n\r"

int main(int argc, char** argv) {

	if (PIN_MODE==PIN_BCM) {
		std::cout << "Pins in BCM mode." << nl;
		if (wiringPiSetupGpio()<0) {
			std::cout << "Could not setup GPIO pins" << nl;
			return -1;
		}
	} else {
		std::cout << "Pins in wiringPi mode." << nl;
		wiringPiSetup();
	}

	Encoder encoderL, encoderR;
	Motor motorL, motorR;

	encoderL.setup(ENCODER_LEFT, LEFT_SIDE);
	encoderR.setup(ENCODER_RIGHT, RIGHT_SIDE);
	motorL.setup(MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_LEFT_E, &encoderL);
	motorR.setup(MOTOR_RIGHT_A, MOTOR_RIGHT_B, MOTOR_RIGHT_E, &encoderR);

	KBAsync kb;
	int key;
	int dir = 1;
	float l = 7.25;
	float r = 3.25;
	float pos[3];

	encoderL.setDirection(dir);
	encoderR.setDirection(dir);
	Odometry odometry(l, l, r, r, &encoderL, &encoderR);

	motorL.setTargetSpeed(13);
	motorR.setTargetSpeed(-13);

	while ((key=kb.getKey())!='q') {

		float aL = encoderL.getAngle();
		float aR = encoderR.getAngle();

		odometry.updatePosition(pos);

		std::cout << " x: " << pos[0] << " y: " << pos[1] << " theta: " << pos[2]*180/M_PI << "o dsL: " << aR*r << " dsR: " << aL*r << " aL: " << aR << " aR: " << aL << nl;
		delayMicroseconds(100000);
	}

	motorL.stop();
	motorR.stop();
	std::cout << nl << "Exiting..." << nl;

	return 0;
}

