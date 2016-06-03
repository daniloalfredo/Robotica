/**
 * Robô de direção diferencial
 * Disciplina de Robótica CIn/UFPE
 * 
 * @autor Prof. Hansenclever Bassani
 * 
 * Este código é proporcionado para facilitar os passos iniciais da programação.
 * Porém, não há garantia de seu correto funcionamento.
 * 
 * Testado em: Ubuntu 14.04 + Netbeans
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <unistd.h>

#define V_REP_IP_ADDRESS "127.0.0.1"
#define V_REP_PORT 19997//1999;

#define K_RHO 0.4
#define K_ALPHA 0.8
#define K_BETA -0.15

#define WHEEL1_R 0.0325
#define WHEEL2_R 0.0325
#define WHEEL_L 0.075

#define MODO 'B' // or B

extern "C" {
#include "extApi.h"
    /*	#include "extApiCustom.h" if you wanna use custom remote API functions! */
}

simxInt ddRobotHandle;
simxInt leftMotorHandle;
simxInt rightMotorHandle;
simxInt sensorHandle;
simxInt graphOdometryHandle;

float to180range(float angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle<-M_PI) 
        angle = angle + 2 * M_PI;
	else if (angle > M_PI)
        angle = angle - 2 * M_PI;

    return angle;
}

void getPosition(int clientID, simxFloat pos[])  //[x,y,theta]
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

simxInt getSimTimeMs(int clientID) //In Miliseconds
{ 
    return simxGetLastCmdTime(clientID);
}

float to_positive_angle(float angle)
{

    angle = fmod(angle, 2 * M_PI);
    while (angle < 0) {
        angle = angle + 2 * M_PI;
    }
    return angle;
}

float smallestAngleDiff(float target, float source)
{
    float a;
    a = to_positive_angle(target) - to_positive_angle(source);

    if (a > M_PI) {
        a = a - 2 * M_PI;
    } else if (a < -M_PI) {
        a = a + 2 * M_PI;
    }
    return a;
}

void readOdometers(int clientID, simxFloat &dPhiL, simxFloat &dPhiR)
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

void setTargetSpeed(int clientID, simxFloat phiL, simxFloat phiR)
{
    simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, simx_opmode_oneshot);   
}

inline double to_deg(double radians)
{
    return radians * (180.0 / M_PI);
}

void motion_control(simxFloat pos[3], simxFloat goal[3],
simxFloat dPhiL, simxFloat dPhiR,
simxFloat* phiL, simxFloat* phiR)
{
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

    simxFloat v = K_RHO*rho;
	simxFloat w = (K_ALPHA*alpha + K_BETA*beta);

	simxFloat wR = 2 * v + WHEEL_L*w;
	simxFloat wL = wR - 2 * WHEEL_L*w;

    if(MODO == 'A')
    {
		*phiL = wL/WHEEL1_R; //rad/s
	    *phiR = wR/WHEEL2_R; //rad/s
    }
    else if (MODO == 'B')
    {
    	if(fabs(to_deg(alpha)) > 90)
    	{
			*phiL = - wL/WHEEL1_R; //rad/s
		    *phiR = - wR/WHEEL2_R; //rad/s
		}
		else
		{
			*phiL = wL/WHEEL1_R; //rad/s
	    	*phiR = wR/WHEEL2_R; //rad/s
		}
    }

    if(rho < 0.02)
    {
    	if(fabs(dtheta) < 0.03)
    	{
    		*phiR = 0;
            *phiL = 0;
			printf("Goal reached!!!\n\n");
        } 
        else
        {
            *phiR = 2 * dtheta;
            *phiL = -2 * dtheta;
			printf("Angle correction\n");
		}

    } 
}
            
int main(int argc, char* argv[])
{
    std::string ipAddr = V_REP_IP_ADDRESS;
    int portNb = V_REP_PORT;

    if (argc > 1) {
        ipAddr = argv[1];
    }

    printf("Iniciando conexao com: %s...\n", ipAddr.c_str());

    int clientID = simxStart((simxChar*) (simxChar*) ipAddr.c_str(), portNb, true, true, 2000, 5);
    if (clientID != -1) {
        printf("Conexao efetuada\n");
        
        //Get handles for robot parts, actuators and sensores:
        simxGetObjectHandle(clientID, "RobotFrame#", &ddRobotHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "LeftMotor#", &leftMotorHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "RightMotor#", &rightMotorHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "GraphOdometry#", &graphOdometryHandle, simx_opmode_oneshot_wait);
        
        printf("RobotFrame: %d\n", ddRobotHandle);
        printf("LeftMotor: %d\n", leftMotorHandle);
        printf("RightMotor: %d\n", rightMotorHandle);
        printf("GraphOdometry: %d\n", graphOdometryHandle);

        //start simulation
        int ret = simxStartSimulation(clientID, simx_opmode_oneshot_wait);
        
        if (ret==-1) {
            printf("Não foi possível iniciar a simulação.\n");
            return -1;
        }
        
        printf("Simulação iniciada.\n");

        //While is connected:
        while (simxGetConnectionId(clientID) != -1)
        {    
            //Read current position:
            simxFloat pos[3]; //[x,y,theta] in [cm cm rad]
            getPosition(clientID, pos);

            //Read simulation time of the last command:
            simxInt time = getSimTimeMs(clientID); //Simulation time in ms or 0 if sim is not running
            
            //stop the loop if simulation is has been stopped:
            if (time == 0)
            	break;             
            
            printf("Posicao: [%.2f %.2f %.2fº], time: %dms\n", pos[0], pos[1], to_deg(pos[2]), time);
            
            //Set robot goal
            simxFloat goal[3]; goal[0] = 0.25; goal[1] = -0.75; goal[2] = 0;
            
            //Read current wheels angle variation:
            simxFloat dPhiL, dPhiR; //rad
            readOdometers(clientID, dPhiL, dPhiR);
            printf("dPhiL: %.2f dPhiR: %.2f\n", dPhiL, dPhiR);
            
            //Set new target speeds: robot going in a circle:
            simxFloat phiL, phiR;
			motion_control(pos, goal, dPhiL, dPhiR, &phiL, &phiR);     
            setTargetSpeed(clientID, phiL, phiR);

            //Let some time for V-REP do its work:
            extApi_sleepMs(2);
        }
        
        //Stop the robot and disconnect from V-Rep;
        setTargetSpeed(clientID, 0, 0);
        simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
        simxFinish(clientID);
        
    } else {
        printf("Nao foi possivel conectar.\n");
        return -2;
    }
    
    return 0;
}


