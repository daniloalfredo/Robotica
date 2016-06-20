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
#include <stdlib.h>

#define nl "\n\r"

float deg(float rad) {
	return rad*180.0/M_PI;
}

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

	motorL.setMinPower(1);
	motorR.setMinPower(1);
	
	//Correct values	l=7.25 r= 3.5
	Odometry odometry(5.8, 5.8, 3.0, 3.0, &encoderL, &encoderR);
	MotionControl control(&motorL, &motorR, &odometry);
	control.setMaxSpinSpeed(8);

	float k_rho = 0.4;
	float k_alpha = 8;
	float k_beta = -5;

	if (argc>1)
		k_rho = atof(argv[1]);
	if (argc>2)
		k_alpha = atof(argv[2]);
	if (argc>3)
		k_beta = atof(argv[3]);

	std::cout << "krho: " << k_rho << " kalpha: " << k_alpha << " kbeta: " << k_beta << nl;
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

	float squareSide = 300; //cm
	float drho = 5; //cm
	float dtheta = 2*0.0174533; //x1º
	std::vector<float *> path;
	unsigned int p = 0;


	float p1[] = {squareSide, 0, 0}; path.push_back(p1); //[cm, cm, rad]
//	float p1t[] = {squareSide, 0, -M_PI/2}; path.push_back(p1t); //[cm, cm, rad]
//	float p2[] = {squareSide, -squareSide, -M_PI/2}; path.push_back(p2); //[cm, cm, rad]
//	float p2t[] = {squareSide, -squareSide, -M_PI}; path.push_back(p2t);//[cm, cm, rad]
//	float p3[] = {0, -squareSide, -M_PI}; path.push_back(p3); //[cm, cm, rad]
//	float p3t[] = {0, -squareSide, M_PI/2}; path.push_back(p3t); //[cm, cm, rad]
//	float p4[] = {0, 0, M_PI/2}; path.push_back(p4); //[cm, cm, rad]
//	float p4t[] = {0, 0, 0}; path.push_back(p4t); //[cm, cm, rad]

	control.setGoal(p1, drho, dtheta);

	KBAsync kb;
	int key;

	std::cout << "Goal: [" << path[p][0] << "," << path[p][1] << "," << deg(path[p][2])  << "º]\n\r";

	while ((key=kb.getKey())!='q') {

		int state = control.controlStep();

		static rbtTime lastTime=0;
		rbtTime time = RobotTimer::getTime_us();
		rbtTime deltat = time-lastTime;

		if (deltat>60000) {
			float pos[3];
			odometry.getPosition(pos);		
			std::cout << std::flush << "Pos:  [" << pos[0] << "," << pos[1] << "," << deg(pos[2])  << "º] state:" << state;
			std::cout << " phiR: " << motorR.getTargetSpeed() << " phiL: " << motorL.getTargetSpeed();
			std::cout << " eR: " << motorR.pid.err() << " eL: " << motorL.pid.err();
			std::cout << nl; //"                 ";
			lastTime = time;
		}

		if (state==2) {
		 	motorL.stop();
			motorR.stop();
			std::cout << nl << nl << "set point reached." << nl;
			p++;
			if (p>=path.size()) {
				std::cout << nl << "done." << nl;
				break;
			}
			delayMicroseconds(1000000);
			control.setGoal(path[p], drho, dtheta);
			std::cout << "Next Goal: [" << path[p][0] << "," << path[p][1] << "," << deg(path[p][2])  << "º] state:" << state <<  "\n\r";
		}

		motorL.controlSpeed();
		motorR.controlSpeed();

		delayMicroseconds(50);
	}

	motorL.stop();
	motorR.stop();
	std::cout << nl << "Exiting..." << "\n\r";

	return 0;
}

