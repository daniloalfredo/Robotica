/* 
 * File:   Sonar.h
 * Author: hans
 *
 * Created on 4 de Junho de 2015, 20:26
 */

#ifndef SONAR_H
#define	SONAR_H

#include "Pins.h"
#include <wiringPi.h>

#define TIMEOUT 30000
//#define DIVISOR (113.42/2.0) //Should be sound speed/2 but was adjusted for better precision.
#define DIVISOR (0.034029/2.0) //speed of sound in cm/us divided by 2

class Sonar {
    int pinTrigger;
	 int pinEcho;
	 float divisor;

public:
    Sonar();
    int setup(int pinTrigger, int pinEcho);
	 void setDivisor(float divisor);

    float measureDistance();
    virtual ~Sonar();
private:

};

#endif	/* SONAR_H */

