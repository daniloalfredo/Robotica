#include "Robot.h"
#include <bits/stdc++.h>
using namespace std;
void Robot::Init(simxInt clientID)
{
	//Pega referência para os componentes do robô 
	simxGetObjectHandle(clientID, "RobotFrame#", &ddRobotHandle, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "LeftMotor#", &leftMotorHandle, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "RightMotor#", &rightMotorHandle, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "GraphOdometry#", &graphOdometryHandle, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "ProximitySensorL#", &sonarL, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "ProximitySensorR#", &sonarR, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "ProximitySensorF#", &sonarF, simx_opmode_oneshot_wait);
	
	//Inicializa as constantes de controle de movimento
	K_RHO = 0.15;
	K_ALPHA = 2;
	K_BETA = -0.5;
	WHEEL1_R = 0.0325;
	WHEEL2_R = 0.0325;
	WHEEL_L = 0.075;
}

void Robot::Log()
{
	printf("Posicao Atual: [%.2f %.2f %.2fº]\n", pos[0], pos[1], to_deg(pos[2]));
	printf("Leituras dos Sonares (L,R,F) = [%.4f, %.4f, %.4f] \n", sonar_reading[0], sonar_reading[1], sonar_reading[2]);
}

void Robot::GetAPIPosition(simxInt clientID)
{
	simxInt ret = simxGetObjectPosition(clientID, ddRobotHandle, -1, pos, simx_opmode_oneshot_wait);
    if (ret > 0) {
        printf("Error reading robot position\n");
        return;
    }
 
    simxFloat orientation[3];
    ret = simxGetObjectOrientation(clientID, ddRobotHandle, -1, orientation, simx_opmode_oneshot_wait);
    if (ret > 0) {
        printf("Error reading robot orientation\n");
        return;
    }
 
    simxFloat theta = orientation[2];
    pos[2] = theta;
}

void Robot::GetSonarReadings(simxInt clientID)
{
	sonar_reading[0] = readSonar(clientID, sonarL);
	sonar_reading[1] = readSonar(clientID, sonarR);
	sonar_reading[2] = readSonar(clientID, sonarF);
  //  simxFloat sonar_reading[3]; // [L, R, F]
    std::cout << "Sonar L : "<< sonarL << " : "<< (float)sonar_reading[0] << std::endl;
   // printf("%.2f",sonar)
    std::cout << "Sonar R : "<< sonarR << " : "<< (float)sonar_reading[1] << std::endl;
    std::cout << "Sonar F : "<< sonarF<< " : "<< (float)sonar_reading[2] << std::endl;
}

void Robot::SetNextGoal(simxFloat x, simxFloat y, simxFloat theta)
{
	goal[0] = x;
	goal[1] = y;
	goal[2] = theta;
}

int Robot::ExecuteMotionControl(simxInt clientID)
{
	simxFloat phiL, phiR;

	//Read current wheels angle variation:
	simxFloat dPhiL, dPhiR; //rad
	readOdometers(clientID, dPhiL, dPhiR);
	
    simxFloat theta = pos[2];
    simxFloat dtheta = smallestAngleDiff(goal[2], theta);
 
    simxFloat deltax = goal[0] - pos[0];
    simxFloat deltay = goal[1] - pos[1];
    simxFloat rho = sqrt(deltax*deltax + deltay*deltay);
 
    simxFloat atg = atan2(deltay, deltax);
    atg = to180range(atg);
    simxFloat alpha = smallestAngleDiff(atg, theta);
    alpha = to180range(alpha);
    simxFloat beta = goal[2] - theta - alpha;
    beta = to180range(beta);
 
    simxFloat v = K_RHO * rho;
    if(v < 0.2){v = 0.2;}

    simxFloat w = (K_ALPHA * alpha + K_BETA * beta);
 
    simxFloat wR = 2*v + WHEEL_L*w;
    simxFloat wL = wR - 2*WHEEL_L*w;
 
    phiL = wL/WHEEL1_R; //rad/s
    phiR = wR/WHEEL2_R; //rad/s

    if(rho < 0.05)
    {
		phiR = 2 * dtheta;
		phiL = -2 * dtheta;
		if(fabs(dtheta) >= 0.9)
		{
			phiR = 2 * dtheta;
			phiL = -2 * dtheta;
		}
		else
		{
			phiR = 0;
			phiL = 0;
			SetTargetSpeed(clientID, phiL, phiR);
			return 1;
		}
		//printf("Angle correction\n");
    }

    SetTargetSpeed(clientID, phiL, phiR);
    return 0;
}

void Robot::SetTargetSpeed(int clientID, simxFloat phiL, simxFloat phiR)
{
    simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, simx_opmode_oneshot);  
}

// bool Robot::HasReachedGoal()
// {
// 	const float max_distance_pos = 0.02;
// 	const float max_distance_angle = 0.5;
	
// 	float distance_pos = sqrt( (pos[0]-goal[0])*(pos[0]-goal[0]) 
// 						 + (pos[1]-goal[1])*(pos[1]-goal[1]));
						 
// 	float cur_angle = to_positive_angle(pos[2]);
// 	float distance_angle = sqrt((cur_angle-goal[2])*(cur_angle-goal[2]));	
			 
// 	if(distance_pos <= max_distance_pos && distance_angle <= max_distance_angle)
// 		return true;
// 	else
// 		return false;
// }

//------------------------------------------------------------------------
//Funções auxiliares
//------------------------------------------------------------------------

void Robot::readOdometers(int clientID, simxFloat &dPhiL, simxFloat &dPhiR)
{
    //old joint angle position
    static simxFloat lwprev=0;
    static simxFloat rwprev=0;
   
    //current joint angle position
    simxFloat lwcur=0;
    simxFloat rwcur=0;
 
    simxGetJointPosition(clientID, leftMotorHandle, &lwcur, simx_opmode_oneshot);
    simxGetJointPosition(clientID, rightMotorHandle, &rwcur, simx_opmode_oneshot);
 
    dPhiL = smallestAngleDiff(lwcur, lwprev);
    dPhiR = smallestAngleDiff(rwcur, rwprev);
    lwprev = lwcur;
    rwprev = rwcur;
}

simxFloat Robot::readSonar(int clientID, simxInt &sonar)
{
    simxUChar detectionState;
    simxFloat detectedPoint[3];
    simxInt detectedObjectHandle;
    simxFloat detectedSurfaceNormalVector[3];
    simxInt operationMode;
    
    simxInt errorCode = simxReadProximitySensor(clientID,sonar,&detectionState,detectedPoint,&detectedObjectHandle,detectedSurfaceNormalVector,operationMode);
    if (detectionState!=0)
       return detectedPoint[2];
    else
        return -1;
}
