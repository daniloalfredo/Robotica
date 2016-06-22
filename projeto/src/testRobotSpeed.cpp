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

//define nl "                \r"
#define nl "\r\n"

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
	float speed=0;
	float kp=0.05, ki=0.00, kd=0.0;
	float minPower = 5;
        float spdSampRate = 0;
	int dir = 1;

	motorL.pid.setParam(kp, ki, kd);
	motorR.pid.setParam(kp, ki, kd);
	motorL.setSpdSamplingRate(spdSampRate);
	motorR.setSpdSamplingRate(spdSampRate);
	motorL.setMinPower(minPower);
	motorR.setMinPower(minPower);

	//std::cout << std::setprecision(4) << std::fixed;
	std::cout << "speed: " << dir*speed << nl;
	motorL.setTargetSpeed(dir*speed);
	motorR.setTargetSpeed(dir*speed);
	while ((key=kb.getKey())!='q') {

	   if (key=='d')
		{
			speed+=0.5;
			std::cout << nl << "Target speed: " << dir*speed << nl;
			motorL.setTargetSpeed(dir*speed);
			motorR.setTargetSpeed(dir*speed);
		}
		else if (key=='c') {
			speed-=0.5;
			std::cout << nl << "Target speed: " << dir*speed << nl;
			motorL.setTargetSpeed(dir*speed);
			motorR.setTargetSpeed(dir*speed);
		}
		else if (kb.getKey()=='s') {
			motorL.stop();
			motorR.stop();
			std::cout << nl << "stop." << nl;
		}
		else if (key=='[') {
			dir = 1;
			motorL.setTargetSpeed(dir*speed);
			motorR.setTargetSpeed(dir*speed);
			std::cout << nl << "dir: " << dir << nl;
		} else if (key==']') {
			dir = -1;
			motorL.setTargetSpeed(dir*speed);
			motorR.setTargetSpeed(dir*speed);
			std::cout << nl << "dir: " << dir << nl;
		} else if (key=='g') {
			kp += 0.01;
			std::cout << nl << "kp: " << kp << " ki: " << ki << " kd: " << kd << nl;
			motorL.pid.setKp(kp);
			motorR.pid.setKp(kp);
		} else if (key=='b') {
			kp -= 0.01;
			std::cout << nl << "kp: " << kp << " ki: " << ki << " kd: " << kd << nl;
			motorL.pid.setKp(kp);
			motorR.pid.setKp(kp);
		}  else if (key=='h') {
			ki += 0.001;
			std::cout << nl << "kp: " << kp << " ki: " << ki << " kd: " << kd << nl;
			motorL.pid.setKi(ki);
			motorR.pid.setKi(ki);
		} else if (key=='n') {
			ki -= 0.001;
			std::cout << nl << "kp: " << kp << " ki: " << ki << " kd: " << kd << nl;
			motorL.pid.setKi(ki);
			motorR.pid.setKi(ki);
		} else if (key=='j') {
			kd += 0.5;
			std::cout << nl << "kp: " << kp << " ki: " << ki << " kd: " << kd << nl;
			motorL.pid.setKd(kd);
			motorR.pid.setKd(kd);
		} else if (key=='m') {
			kd -= 0.5;
			std::cout << nl << "kp: " << kp << " ki: " << ki << " kd: " << kd << nl;
			motorL.pid.setKd(kd);
			motorR.pid.setKd(kd);
		} else if (key=='f') {
			minPower += 1;
			std::cout << nl << "minPower: " << minPower << nl;
			motorL.setMinPower(minPower);
			motorR.setMinPower(minPower);
		} else if (key=='v') {
			minPower -= 1;
			std::cout << nl << "minPower: " << minPower << nl;
			motorL.setMinPower(minPower);
			motorR.setMinPower(minPower);
		}

		motorL.controlSpeed();
		motorR.controlSpeed();

		static rbtTime lastTime=0;
		rbtTime time = RobotTimer::getTime_us();
		rbtTime deltat = time-lastTime;
		if (deltat>200000) { 

			std::cout << "Target: " << dir*speed << " SpeedLR: " << motorL.getSpeed() << " | " << motorR.getSpeed() << " PwrLR: " << motorL.getPower() << " | " <<  motorR.getPower() << nl;

			lastTime = time;
		}
		delayMicroseconds(10000);	
	}

	motorL.stop();
	motorR.stop();
	std::cout << nl << "Exiting..." << nl;

	return 0;
}

