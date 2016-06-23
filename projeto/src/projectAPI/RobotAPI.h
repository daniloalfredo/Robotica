#ifndef ROBOTAPI_H_INCLUDED
#define ROBOTAPI_H_INCLUDED

//Escolhe entre usar o VREP ou o robô real  
#define USING_VREP 1

//Includes e Variáveis do V-REP
#if USING_VREP == 1
	extern "C" {
		#include "../vrepAPI/extApi.h"
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

//Includes e variáveis do robô real
#elif USING_VREP == 0
	#include <wiringPi.h>
	#include "../robotAPI/Pins.h"
	#include "../robotAPI/Sonar.h"
	#include "../robotAPI/Motor.h"
	#include "../robotAPI/Encoder.h"
	#include "../robotAPI/KBAsync.h"
	#include "../robotAPI/RobotTimer.h"

	extern Encoder encoderL, encoderR;
	extern Motor motorL, motorR;
	extern Sonar sonarL, sonarR, sonarF;
	extern KBAsync kb;

	extern rbtTime simulationBeginTime;
#endif

#include "Utils.h" 

bool APIInitConection();
bool APIStartSimulation();
bool APISimulationIsRunning();
void APIFinishSimulation();
void APIWait();

float APIGetSimulationTimeInSecs();
void APIGetTrueRobotPosition(Matrix* realpos);
void APIReadOdometers(float* dPhiL, float* dPhiR);
void APISetRobotSpeed(float phiL, float phiR);
float APIReadSonarLeft();
float APIReadSonarRight();
float APIReadSonarFront();

#endif