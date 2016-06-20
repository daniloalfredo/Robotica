/* 
 * File:   Sonar.cpp
 * Author: hans
 * 
 * Created on 4 de Junho de 2015, 20:26
 */

#include "Sonar.h"
#include "RobotTimer.h"
#include <stdlib.h>
#include <wiringPi.h>
#include <sstream>
#include <iostream>
#include <errno.h>

using namespace std;

Sonar::Sonar() {
   divisor = DIVISOR;
}

int Sonar::setup(int pinTrigger, int pinEcho) {
    //wiringPiISR(pinEcho, INT_EDGE_FALLING, &echo);
	 this->pinTrigger = pinTrigger;
	 this->pinEcho = pinEcho;

    pinMode(pinTrigger, OUTPUT);
    pinMode(pinEcho,INPUT);
    digitalWrite(pinTrigger, LOW);
    std::cout << "Waiting for sonar sensor to settle... ";
    delayMicroseconds(2000000);
    std::cout << "ok" << std::endl;

    return 0;
}

void Sonar::setDivisor(float divisor) {
  this->divisor = divisor;
}

float Sonar::measureDistance() {
    rbtTime start, pulseTime, delta;

    //Send pulse
    digitalWrite(pinTrigger,LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrigger,HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrigger,LOW);

    //wait for pulse:
    start = RobotTimer::getTime_us();
    do {
		pulseTime = RobotTimer::getTime_us();
		if (pulseTime-start > TIMEOUT) return -1;
    } while (digitalRead(pinEcho)==LOW);

    //wait for echo:
    do {
		delta = (RobotTimer::getTime_us() - pulseTime);
		if (delta > TIMEOUT) return -1;
    } while (digitalRead(pinEcho)==HIGH);

    return delta*divisor;
}

Sonar::~Sonar() {
}

