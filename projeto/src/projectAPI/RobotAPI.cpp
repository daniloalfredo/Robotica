#include "RobotAPI.h"

#if USING_VREP == 1
    simxInt clientID;
    simxInt ddRobotHandle;
    simxInt leftMotorHandle;
    simxInt rightMotorHandle;
    simxInt sonarL;
    simxInt sonarR;
    simxInt sonarF;

#elif USING_VREP == 0
    Encoder encoderL, encoderR;
    Motor motorL, motorR;
    Sonar sonarL, sonarR, sonarF;
    KBAsync kb;
    rbtTime simulationBeginTime;
#endif

bool APIInitConection()
{
    #if USING_VREP == 1
    	printf("Iniciando conexão com: %s...\n", V_REP_IP_ADDRESS);
    	clientID = simxStart(V_REP_IP_ADDRESS, V_REP_PORT, true, true, 2000, 5);

    	if(clientID != -1)
    	{
    		//Pega referência para os componentes do robô 
    		simxGetObjectHandle(clientID, "RobotFrame#", &ddRobotHandle, simx_opmode_oneshot_wait);
    		simxGetObjectHandle(clientID, "LeftMotor#", &leftMotorHandle, simx_opmode_oneshot_wait);
    		simxGetObjectHandle(clientID, "RightMotor#", &rightMotorHandle, simx_opmode_oneshot_wait);
    		simxGetObjectHandle(clientID, "ProximitySensorL#", &sonarL, simx_opmode_oneshot_wait);
    		simxGetObjectHandle(clientID, "ProximitySensorR#", &sonarR, simx_opmode_oneshot_wait);
    		simxGetObjectHandle(clientID, "ProximitySensorF#", &sonarF, simx_opmode_oneshot_wait);

    		return true;
    	}

    #elif USING_VREP == 0
        if (PIN_MODE==PIN_BCM)
        {
            printf("Pins in BCM mode.\n");
            
            if (wiringPiSetupGpio()<0)
            {
                printf("Could not setup GPIO pins\n");
                return false;
            }
        } 

        else 
        {
            printf("Pins in wiringPi mode.\n");
            wiringPiSetup();
        }

        encoderL.setup(ENCODER_LEFT, LEFT_SIDE);
        encoderR.setup(ENCODER_RIGHT, RIGHT_SIDE);
        sonarL.setup(SONAR_LEFT_TRIGGER, SONAR_LEFT_ECHO);
        sonarR.setup(SONAR_RIGHT_TRIGGER, SONAR_RIGHT_ECHO);
        sonarF.setup(SONAR_FRONT_TRIGGER, SONAR_FRONT_ECHO);
        motorL.setup(MOTOR_LEFT_A, MOTOR_LEFT_B, MOTOR_LEFT_E, &encoderL);
        motorR.setup(MOTOR_RIGHT_A, MOTOR_RIGHT_B, MOTOR_RIGHT_E, &encoderR);
        motorL.pid.setKp(0.1);
        motorR.pid.setKp(0.1);
        return true;
    #endif

    return false;
}

bool APIStartSimulation()
{
    #if USING_VREP == 1
	   return (simxStartSimulation(clientID, simx_opmode_oneshot_wait) != -1);
    #elif USING_VREP == 0
       simulationBeginTime = GetTimeMicroSecs();
       return true;
    #endif
}

bool APISimulationIsRunning()
{
    #if USING_VREP == 1
    	if(simxGetConnectionId(clientID) == -1
    	|| simxGetLastCmdTime(clientID) == 0)
    	{
    		printf("Parando a simulação...\n");
    		return false;
    	}

    	return true;

    #elif USING_VREP == 0
        if(kb.getKey() == 'q')
            return false;
        return true;
    #endif
}

void APIFinishSimulation()
{
    #if USING_VREP == 1
    	simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
    	simxFinish(clientID);
    #elif USING_VREP == 0
        motorL.stop();
        motorR.stop();
    #endif
}

void APIWait()
{
    #if USING_VREP == 1
        extApi_sleepMs(2);
    #elif USING_VREP == 0
        motorL.controlSpeed();
        motorR.controlSpeed();
        //delayMicroseconds(msecs*1000);
    #endif
}

float APIGetSimulationTimeInSecs()
{
    #if USING_VREP == 1
        return (((float) simxGetLastCmdTime(clientID)) / 1000.0);
    #elif USING_VREP == 0
        return (GetTimeMicroSecs() - simulationBeginTime) / 1000000.0;
    #endif
}

float APIGetTimeSinceLastCommandInSecs(float lastCommandTime)
{
    #if USING_VREP == 1
        return APIGetSimulationTimeInSecs() - lastCommandTime;
    #elif USING_VREP == 0
        return -1.0; //função não implementada no robô real
    #endif
}

void APIGetTrueRobotPosition(Matrix* realpos)
{
    #if USING_VREP == 1
    	static simxFloat real[3];
    	simxInt ret = simxGetObjectPosition(clientID, ddRobotHandle, -1, real, simx_opmode_oneshot_wait);
        if (ret > 0) {
            printf("Erro ao ler posição do robô.\n");
            return;
        }
     
        static simxFloat orientation[3];
        ret = simxGetObjectOrientation(clientID, ddRobotHandle, -1, orientation, simx_opmode_oneshot_wait);
        if (ret > 0) {
            printf("Erro ao ler orientação do robô.\n");
            return;
        }

    	realpos->mat[0][0] = real[0];
    	realpos->mat[1][0] = real[1];
        realpos->mat[2][0] = orientation[2];

    #elif USING_VREP == 0
        realpos->mat[0][0] = -1;
        realpos->mat[1][0] = -1;
        realpos->mat[2][0] = -1;

    #endif
}

void APIReadOdometers(float* dPhiL, float* dPhiR)
{
    #if USING_VREP == 1
        //Old joint angle position
        static simxFloat lwprev = 0;
        static simxFloat rwprev = 0;
       
        //Current joint angle position
        simxFloat lwcur = 0;
        simxFloat rwcur = 0;
     
        simxGetJointPosition(clientID, leftMotorHandle, &lwcur, simx_opmode_oneshot);
        simxGetJointPosition(clientID, rightMotorHandle, &rwcur, simx_opmode_oneshot);
     
        *dPhiL = smallestAngleDiff(lwcur, lwprev);
        *dPhiR = smallestAngleDiff(rwcur, rwprev);
        lwprev = lwcur;
        rwprev = rwcur;

    #elif USING_VREP == 0
        *dPhiL = encoderL.getDeltaAngle();
        *dPhiR = encoderR.getDeltaAngle();
    #endif
}

void APISetRobotSpeed(float phiL, float phiR)
{
    #if USING_VREP == 1
        simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, simx_opmode_oneshot);
        simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, simx_opmode_oneshot); 
    #elif USING_VREP == 0
        motorL.setTargetSpeed(phiL);
        motorR.setTargetSpeed(phiR);
    #endif 
}

float APIReadSonarLeft()
{
    #if USING_VREP == 1
        simxUChar detectionState;
        simxInt detectedObjectHandle;
        simxFloat detectedPoint[3];
        simxFloat detectedSurfaceNormalVector[3];
        
        simxReadProximitySensor(clientID, sonarL, &detectionState, detectedPoint, &detectedObjectHandle, detectedSurfaceNormalVector, simx_opmode_oneshot);
        if (detectionState != 0)
    		return detectedPoint[2];
        else
            return -1;

    #elif USING_VREP == 0
        return sonarL.measureDistance();

    #endif
}

float APIReadSonarRight()
{
    #if USING_VREP == 1
        simxUChar detectionState;
        simxInt detectedObjectHandle;
        simxFloat detectedPoint[3];
        simxFloat detectedSurfaceNormalVector[3];
        
        simxReadProximitySensor(clientID, sonarR, &detectionState, detectedPoint, &detectedObjectHandle, detectedSurfaceNormalVector, simx_opmode_oneshot);
        if (detectionState != 0)
            return detectedPoint[2];
        else
            return -1;

    #elif USING_VREP == 0
        return sonarR.measureDistance();

    #endif
}

float APIReadSonarFront()
{
    #if USING_VREP == 1
        simxUChar detectionState;
        simxInt detectedObjectHandle;
        simxFloat detectedPoint[3];
        simxFloat detectedSurfaceNormalVector[3];
        
        simxReadProximitySensor(clientID, sonarF, &detectionState, detectedPoint, &detectedObjectHandle, detectedSurfaceNormalVector, simx_opmode_oneshot);
        if (detectionState != 0)
            return detectedPoint[2];
        else
            return -1;

    #elif USING_VREP == 0
        return sonarF.measureDistance();    

    #endif
}
