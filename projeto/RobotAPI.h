#ifndef ROBOTAPI_H_INCLUDED
#define ROBOTAPI_H_INCLUDED

//Escolhe entre usar o VREP ou o robô real  
#define USING_VREP 1

//Includes e Variáveis do V-REP
#if USING_VREP == 1
	extern "C" {
		#include "extApi.h"
	}

	#define V_REP_IP_ADDRESS "127.0.0.1"
	#define V_REP_PORT 19997

	extern simxInt clientID;

	//Componentes do robô
	extern simxInt ddRobotHandle;
	extern simxInt leftMotorHandle;
	extern simxInt rightMotorHandle;
	extern simxInt sonarL;
	extern simxInt sonarR;
	extern simxInt sonarF;
#endif

#include "Utils.h" 

bool APIInitConection();
bool APIStartSimulation();
bool APISimulationIsRunning();
void APIFinishSimulation();
void APIWaitMsecs(int msecs);

float APIGetSimulationTimeInSecs();
void APIGetTrueRobotPosition(Matrix* realpos);
void APIReadOdometers(float* dPhiL, float* dPhiR);
void APISetRobotSpeed(float phiL, float phiR);
float APIReadSonarLeft();
float APIReadSonarRight();
float APIReadSonarFront();

#endif