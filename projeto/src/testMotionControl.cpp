/*
 * File:   main.cpp
 * Author: hans
 *
 * Created on 4 de Junho de 2015, 20:25
 */

#include <cstdlib>
#include <iostream>
#include <iomanip> 
#include <signal.h>
#include <sstream>
#include "opencv2/highgui/highgui.hpp"

#include <wiringPi.h>
#include "robotAPI/Pins.h"
#include "robotAPI/Sonar.h"
#include "robotAPI/Motor.h"
#include "robotAPI/Encoder.h"
#include "robotAPI/Odometry.h"
#include "robotAPI/MotionControl.h"
#include "robotAPI/KBAsync.h"
#include <vector>
#include <math.h>

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

	std::cout << std::setprecision(4) << std::fixed;

	encoderL.setup(ENCODER_LEFT, LEFT_SIDE);
	encoderR.setup(ENCODER_RIGHT, RIGHT_SIDE);
	motorL.setup(MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_LEFT_E, &encoderL);
	motorR.setup(MOTOR_RIGHT_A, MOTOR_RIGHT_B, MOTOR_RIGHT_E, &encoderR);

	motorL.setMinPower(10);
	motorR.setMinPower(10);
	motorL.setK(0.1);
	motorR.setK(0.1);
	
	//Correct values	l=7.25 r=2.0
	Odometry odometry(7.25, 7.25, 2.0, 2.0, &encoderL, &encoderR);
	MotionControl control(&motorL, &motorR, &odometry);

	float goal[] = {20, 0, 0}; //[cm, cm, rad]
	//float goal[] = {0, 0, -M_PI}; //[cm, cm, rad]

	control.setGoal(goal, 2, 0.02);
	control.setMaxSpinSpeed(15);

	float k_rho = 0.5;
	float k_alpha = 1;
	float k_beta = -0.15*k_alpha;
	bool strongControl = 0;

	if (k_rho>0 && k_beta < 0) {
		if ((k_alpha + (5.0/3.0)*k_beta - (2/M_PI)*k_rho) > 0) {
			std::cout << "Strong stability control." << nl;
			strongControl = 1;
		}
		else if (k_alpha>k_rho) 
			std::cout << "Weak stability control."  << nl;
		else 
			std::cout << "Unstable control."  << nl;
	} else
		std::cout << "Unstable control."  << nl;

	if (!strongControl) {
		std::cout << "Press enter to continue..."  << nl;
		char c;
		std::cin >> c;
	}
		
	control.setControlParameters(k_rho, k_alpha, k_beta);


	KBAsync kb;
	int key;
	float k=1;
	while ((key=kb.getKey())!='q') {

		if (key=='a') {
		} else if (key=='[') {
			k += 0.25;
			std::cout << nl << "k: " << k << nl;
			motorL.setK(k);
			motorR.setK(k);
		} else if (key==']') {
			k -= 0.25;
			std::cout << nl << "k: " << k << nl;
			motorL.setK(k);
			motorR.setK(k);
		}

		control.controlStep();
		if (control.getState()==2) {
			std::cout << nl << "done." << nl;
			break;
		}

		motorL.controlSpeed();
		motorR.controlSpeed();
	}

	motorL.stop();
	motorR.stop();
	std::cout << nl << "Exiting..." << nl;

	return 0;
}

