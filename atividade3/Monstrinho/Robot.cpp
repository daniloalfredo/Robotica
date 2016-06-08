#include "Robot.h"

void Robot::Init(simxInt clientID, std::vector<simxFloat*> path)
{
	//Guarda referência do clientID da API
	this->clientID = clientID; 

	//Pega referência para os componentes do robô 
	simxGetObjectHandle(clientID, "RobotFrame#", &ddRobotHandle, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "LeftMotor#", &leftMotorHandle, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "RightMotor#", &rightMotorHandle, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "ProximitySensorL#", &sonarL, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "ProximitySensorR#", &sonarR, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "ProximitySensorF#", &sonarF, simx_opmode_oneshot_wait);
	
	//Inicializa as constantes de controle de movimento
	K_RHO = 0.07;
	K_ALPHA = 2.0;
	K_BETA = -0.25;
	WHEEL1_R = 0.0325;
	WHEEL2_R = 0.0325;
	WHEEL_L = 0.075;
	
	//Variáveis do caminho
	this->path = path;
	current_goal = -1;
	reached_goal = true;
	
	//Inicializa a posição
	UpdatePositionWithAPI();

	//Variáveis da odometria
	odoVarianceX = 0.0;
	odoVarianceY = 0.0;
	odoVarianceTheta = 0.0;
	ERROR_PER_METER_X = 0.10;
	ERROR_PER_METER_Y = 0.10;
	ERROR_PER_METER_THETA = 1*(PI/180.0);
}

void Robot::Stop()
{
	SetTargetSpeed(0, 0);
}

void Robot::Log(EnvMap envmap)
{
	printf("RealPos: [%.2f, %.2f, %.2f]\t//[X, Y, THETA]\n", realpos[0], realpos[1], realpos[2]);
	printf("CalcPos: [%.2f, %.2f, %.2f]\t//[X, Y, THETA]\n", pos[0], pos[1], pos[2]);
	printf("ErroPos: [%.2f, %.2f, %.2f]\t//realpos - pos\n", realpos[0]-pos[0], realpos[1]-pos[1], realpos[2]-pos[2]);
	printf("VarOdom: [%.2f, %.2f, %.2f]\t\n", odoVarianceX, odoVarianceY, odoVarianceTheta);
	//printf("Sonares: [%.2f, %.2f, %.2f] \t//[F, L, R]\n", sonar_reading[2], sonar_reading[0], sonar_reading[1]);
	//printf("MapDist: [%.2f]\n", envmap.MapDistance(pos[0], pos[1], pos[2]));
}

void Robot::Update(EnvMap testmap)
{	
	//Guarda a posicao real da api para comparação
	GetAPIPosition();

	//Atualiza a posição usando odometria
	UpdatePositionWithOdometry();
	
	//Faz a leitura dos sonares
	UpdateSonarReadings();
	
	//Se já se deslocou o bastante
	if(odoVarianceX >= 0.05 || odoVarianceY >= 0.05 || odoVarianceTheta >= 0.1*(PI/180.0))
	{	
		//Atualiza a posição atual do robô usando os sensores e o mapa
		UpdatePositionWithSensorsAndMap(testmap);
		printf("Passo de Correção.\n");
	}
	
	//Executa o controle de movimento
	ExecuteMotionControl();
	
	//Verifica se já chegou no objetivo
	if(reached_goal)
	{
		current_goal++;
		
		if(current_goal >= (int) path.size())
			current_goal = 0;
			
		goal[0] = path[current_goal][0];
		goal[1] = path[current_goal][1];
		goal[2] = path[current_goal][2];
		reached_goal = false;
	}
}

void Robot::GetAPIPosition()
{
	simxInt ret = simxGetObjectPosition(clientID, ddRobotHandle, -1, realpos, simx_opmode_oneshot_wait);
    if (ret > 0) {
        printf("Erro ao ler posicao do robo.\n");
        return;
    }
 
    simxFloat orientation[3];
    ret = simxGetObjectOrientation(clientID, ddRobotHandle, -1, orientation, simx_opmode_oneshot_wait);
    if (ret > 0) {
        printf("Erro ao ler orientacao do robo.\n");
        return;
    }

    realpos[2] = orientation[2];
}

void Robot::ExecuteMotionControl()
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
 
    simxFloat v = K_RHO * rho;
    
    if(v < 0.2)
    	v = 0.2;

    simxFloat w = (K_ALPHA * alpha + K_BETA * beta);
 
    simxFloat wR = 2*v + WHEEL_L*w;
    simxFloat wL = wR - 2*WHEEL_L*w;
 
 	simxFloat phiL, phiR;
    phiL = wL/WHEEL1_R; //rad/s
    phiR = wR/WHEEL2_R; //rad/s

    if(rho < 0.08/*0.05*/)
    {
		phiR = 2 * dtheta;
		phiL = -2 * dtheta;
		
		if(fabs(dtheta) >= 0.9)
		{
			phiR = 2 * dtheta;
			phiL = -2 * dtheta;
		}
		
		//Se chegou ao objetivo
		else
		{
			reached_goal = true;
			//printf("Objetivo alcançado.\n");
		}
    }

    SetTargetSpeed(phiL, phiR);
}

void Robot::SetTargetSpeed(simxFloat phiL, simxFloat phiR)
{
    simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, simx_opmode_oneshot);  
}

void Robot::UpdateSonarReadings()
{
	sonar_reading[0] = readSonar(sonarL);
	sonar_reading[1] = readSonar(sonarR);
	sonar_reading[2] = readSonar(sonarF);
}

void Robot::UpdatePositionWithAPI()
{
	odoVarianceX = 0.0;
	odoVarianceY = 0.0;
	odoVarianceTheta = 0.0;

	GetAPIPosition();
	pos[0] = realpos[0];
	pos[1] = realpos[1];
	pos[2] = realpos[2];
}

void Robot::UpdatePositionWithOdometry()
{
	//Leitura do odometro para saber as variações dPhiL e dPhiR 
	readOdometers();
	
	float deltaSl = WHEEL1_R * dPhiL;
	float deltaSr = WHEEL1_R * dPhiR;
	float deltaS = (deltaSl + deltaSr) / 2.0;
	float deltaTheta = (deltaSr - deltaSl) / (2*WHEEL_L);
	float deltaX = deltaS * cos(pos[2] + deltaTheta/2.0);
	float deltaY = deltaS * sin(pos[2] + deltaTheta/2.0);

	//Atualiza Incertezas de posição
	odoVarianceX += fabs(deltaX * ERROR_PER_METER_X);
	odoVarianceY += fabs(deltaY * ERROR_PER_METER_Y);
	odoVarianceTheta += fabs(deltaTheta * ERROR_PER_METER_THETA);

	//Nova posição
	pos[0] += deltaX;
	pos[1] += deltaY;
	pos[2] += deltaTheta;

	//Por enquanto usando posição da API (remover depois)
	//UpdatePositionWithAPI();
}

void Robot::UpdatePositionWithSensorsAndMap(EnvMap testmap)
{
	//Por enquanto usando api (remover depois)
	UpdatePositionWithAPI();
}

void Robot::readOdometers()
{
    //Old joint angle position
    static simxFloat lwprev = 0;
    static simxFloat rwprev = 0;
   
    //Current joint angle position
    simxFloat lwcur = 0;
    simxFloat rwcur = 0;
 
    simxGetJointPosition(clientID, leftMotorHandle, &lwcur, simx_opmode_oneshot);
    simxGetJointPosition(clientID, rightMotorHandle, &rwcur, simx_opmode_oneshot);
 
    dPhiL = smallestAngleDiff(lwcur, lwprev);
    dPhiR = smallestAngleDiff(rwcur, rwprev);
    lwprev = lwcur;
    rwprev = rwcur;
}

simxFloat Robot::readSonar(simxInt &sonar)
{
    simxUChar detectionState;
    simxFloat detectedPoint[3];
    simxInt detectedObjectHandle;
    simxFloat detectedSurfaceNormalVector[3];
    simxInt operationMode;
    
    simxInt errorCode = simxReadProximitySensor(clientID,sonar,&detectionState,detectedPoint,&detectedObjectHandle,detectedSurfaceNormalVector,operationMode);
    if (detectionState != 0)
		return detectedPoint[2];
    else
        return -1;
}
