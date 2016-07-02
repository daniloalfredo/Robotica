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
	#include "../robotAPI/RobotTimer.h"

	extern Encoder encoderL, encoderR;
	extern Motor motorL, motorR;
	extern Sonar sonarL, sonarR, sonarF;
#endif

//Includes e Variáveis gerais
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
#include <cstdio>
#include <cstdlib>
	
#include "KBAsync.h"
#include "Utils.h"

extern KBAsync kb;
extern int kb_key;
extern bool stopped;
extern cv::VideoCapture cap;
extern TimeStamp simulationBeginTime;

bool APIInitConection();
bool APIStartSimulation();
bool APISimulationIsRunning();
void APIFinishSimulation();
void APIWait();
int APIGetKey();
float APIGetSimulationTimeInSecs();
void APIGetTrueRobotPosition(Matrix* realpos);
void APIGetTrueRobotPosition(float* realpos);
void APIReadOdometers(float* dPhiL, float* dPhiR);
void APISetRobotSpeed(float phiL, float phiR);
void APIStopRobot();
cv::Mat APIReadCamera();
void APISavePicture(cv::Mat picture);
float APIReadSonarLeft();
float APIReadSonarFront();
float APIReadSonarRight();

#endif