#ifndef _ODOMETRY_
#define _ODOMETRY_

#include <iostream>
#include <vector>
#include <cmath>

#include "Encoder.h"

class Odometry
{
	private:
		Encoder* encoderL;
		Encoder* encoderR;
		double lR, lL, rR, rL;
		float position[3];

	public:
		Odometry(double lR, double ll, double rR, double rL, Encoder *encoderL, Encoder * encoderR);
		void setPosition(const float position[]);
		void updatePosition(float* pos=0);
		void getPosition(float *pos);
		double getLL();
		double getLR();
		double getRL();
		double getRR();
		double to180range(double angle);
};

#endif
