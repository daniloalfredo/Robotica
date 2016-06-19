#include "Odometry.h"
#include <math.h>
#include <iostream>

Odometry::Odometry(double lR, double lL, double rR, double rL, Encoder *encoderL, Encoder * encoderR) {
	this->lR = lR;
	this->lL = lL;
	this->rL = rL;
	this->rR = rR;
	this->encoderL = encoderL;
	this->encoderR = encoderR;
	this->position[0] = 0;
	this->position[1] = 0;
	this->position[2] = 0;
}

void Odometry::setPosition(const float position[]) {
	this->position[0] = position[0];
	this->position[1] = position[1];
	this->position[2] = position[2];
}

void Odometry::updatePosition(float *pos) {

	double odom_theta = position[2];

	double odom_phiL = encoderL->getDeltaAngle();
   double odom_phiR = encoderR->getDeltaAngle();

	double dodom_x_robot = (rR*odom_phiR/2)+(rL*odom_phiL/2);
	double dodom_y_robot = 0;
	double dodom_theta_robot = (rR*odom_phiR/(2*lR))-(rL*odom_phiL/(2*lL));

	double dodom_x = dodom_x_robot*cos(odom_theta) - dodom_y_robot*sin(odom_theta) + 0;
	double dodom_y = dodom_x_robot*sin(odom_theta) + dodom_y_robot*cos(odom_theta) + 0;
	double dodom_theta = dodom_theta_robot;

	position[0] = position[0] + dodom_x;
	position[1] = position[1] + dodom_y;
	position[2] = position[2] + dodom_theta;

	if (pos!=NULL) {
		pos[0] = position[0];
		pos[1] = position[1];
		pos[2] = position[2];
	}
}

void Odometry::getPosition(float *pos) {
	if (pos!=NULL) {
		pos[0] = position[0];
		pos[1] = position[1];
		pos[2] = position[2];
	}
}

double Odometry::getLR() {
	return lR;
}

double Odometry::getLL() {
	return lL;
}

double Odometry::getRL() {
	return rL;
}

double Odometry::getRR() {
	return rR;
}

double Odometry::to180range(double angle) {

   angle = fmod(angle, 2*M_PI);
   if (angle<-M_PI)
		angle = angle + 2*M_PI;
	else 
	if (angle>M_PI)
		angle = angle - 2*M_PI;

   return angle;
}

