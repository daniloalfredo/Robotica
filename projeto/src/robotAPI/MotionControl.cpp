#include <iostream>
#include "MotionControl.h"	
#include <math.h>

MotionControl::MotionControl(Motor *motorL, Motor *motorR, Odometry *odometry) {
	this->motorL = motorL;
	this->motorR = motorR;
	this->odometry = odometry;

	//(k_rho>0; k_beta<0; k_alpha > k_rho):
	k_rho = 1;
	k_alpha = 10;
	k_beta = -1.5;
	maxSpinSpeed = 20;

	this->goal[0] = 0;
	this->goal[1] = 0;
	this->goal[2] = 0;

	rho = 0;
	state = 0;
}

MotionControl::~MotionControl() {
}

void MotionControl::setGoal(const float goal[], float drho, float dtheta){
	this->goal[0] = goal[0];
	this->goal[1] = goal[1];
	this->goal[2] = goal[2];
	this->drho_th = drho;
	this->dtheta_th = dtheta;
	this->state = 0;
}

void MotionControl::setControlParameters(double k_rho, double k_alpha, double k_beta) {
	this->k_rho =k_rho;
	this->k_alpha = k_alpha;
	this->k_beta = k_beta;
}

void MotionControl::setMaxSpinSpeed(const double maxSpinSpeed) {
	this->maxSpinSpeed = maxSpinSpeed;
	this->maxSpeed = maxSpinSpeed*((odometry->getRR()+odometry->getRL())/2);
	std::cout << "MaxSpeed: " << maxSpeed << " ";
}

int MotionControl::controlStep() {

	float position[3];
	odometry->updatePosition(position);
	
	double dx = goal[0] - position[0];
	double dy = goal[1] - position[1];
	double theta = position[2];
	double dtheta = smallestAngleDiff(goal[2], theta);
	rho = sqrt(dx*dx + dy*dy);
	
	double atg = atan2(dy, dx);
	atg = to180range(atg);
	double alpha = smallestAngleDiff(atg,theta);
	alpha = to180range(alpha);
	double beta = goal[2] - theta - alpha;
	beta = to180range(beta);

	double v = k_rho*rho;
	if (v>maxSpeed) v = maxSpeed;
	double omega = k_alpha*alpha + k_beta*beta;

	double wR = v + odometry->getLR()*omega;
	double wL = v - odometry->getLL()*omega;

	double phiR = wR/odometry->getRR();
	double phiL = wL/odometry->getRL();

	if (rho<this->drho_th || state>0) {
		state = 1;
		if (fabs(dtheta)>this->dtheta_th) {
			phiR = maxSpinSpeed*dtheta/M_PI;
			phiL = -maxSpinSpeed*dtheta/M_PI;
		} else {
			phiR = 0;
			phiL = 0;
			state=2;
		}
	}

	motorL->setTargetSpeed(phiL);
	motorR->setTargetSpeed(phiR);

#ifdef _DEBUG_
	std::cout << "Goal: [" << goal[0] << "," << goal[1] << "," << goal[2]  << "] state:" << state <<  "\n\r";
	std::cout << "Pos: [" << position[0] << "," << position[1] << "," << position[2] << "]\n\r";	
	std::cout << "dx: " << dx << " dy: " << dy << " dtheta: " << dtheta << "\n\r";
	std::cout << "atg: " << atg << " rho: " << rho <<  "\n\r";
	std::cout << "alpha: " << alpha  << " beta: " << beta << "\n\r";
	std::cout << "r: " << odometry->getR()  << " l: " << odometry->getL() << "\n\r";   
	std::cout << "v: " << v  << " omega: " << omega << " wL:" << wL << " wR:" << wR << "\n\r";
	std::cout << "phiL: " << phiL << " phiR: " << phiR << "\n\r";
	std::cout << "speedL: " << motorL->getSpeed() << " speedR: " << motorR->getSpeed() << "\n\r";
	std::cout << "pwrL: " << motorL->getPower() << " pwrR: " << motorR->getPower() << "\n\r\n\r";
#endif

	return state;
}

double MotionControl::getRho() {
	return this->rho;
}

int MotionControl::getState() {
	return this->state;
}


double MotionControl::to_positive_angle(double angle) {

   angle = fmod(angle, 2*M_PI);
   while(angle < 0)
     angle = angle + 2*M_PI;

   return angle;
}

double MotionControl::to180range(double angle) {

   angle = fmod(angle, 2*M_PI);
   if (angle<-M_PI)
		angle = angle + 2*M_PI;
	else 
	if (angle>M_PI)
		angle = angle - 2*M_PI;

   return angle;
}

double MotionControl::smallestAngleDiff(double target, double source) {
   double a = to_positive_angle(target) - to_positive_angle(source);
   
   if (a > M_PI)
      a = a - 2*M_PI;
   else 
	if (a < -M_PI)
      a = a + 2*M_PI;
   
   return a;
}
