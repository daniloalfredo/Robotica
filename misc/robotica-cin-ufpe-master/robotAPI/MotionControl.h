#ifndef _MOTION_CONTROL_H_
#define _MOTION_CONTROL_H_

#include <vector>
#include <cmath>
#include <iostream>

#include "Odometry.h"
#include "Motor.h"

class MotionControl
{
	private:
		Motor *motorL, *motorR;
		Odometry *odometry;
		float goal[3];
		double rho, drho_th;
		double dtheta_th;
		int state;
		double k_rho, k_alpha, k_beta;
		double maxSpinSpeed;
		double maxSpeed;

		double to_positive_angle(double angle);
		double to180range(double angle);
	 	double smallestAngleDiff(double target, double source);

	public:
		MotionControl(Motor *motorL, Motor *motorR, Odometry *odometry);
		~MotionControl();
		void setControlParameters(const double k_rho, const double k_alpha, const double k_beta);
		void setGoal(const float goal[], float drho=5, float dtheta = 0.1);
		void setMaxSpinSpeed(const double maxSpinSpeed);
		int controlStep();
		double getRho();
		int getState();
};

#endif
