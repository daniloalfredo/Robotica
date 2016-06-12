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
	
	//Inicializa constantes de odometria
	kl = 0.8;
	kr = 0.8;
	
	//Variáveis do Filtro de Kalman
	R.Resize(3, 3);
	R.mat[0][0] = 0.2;
	R.mat[0][1] = 0.0;
	R.mat[0][2] = 0.0;
	R.mat[1][0] = 0.0;
	R.mat[1][1] = 0.2;
	R.mat[1][2] = 0.0;
	R.mat[2][0] = 0.0;
	R.mat[2][1] = 0.0;
	R.mat[2][2] = 0.2; 
	
	//Variáveis do caminho
	this->path = path;
	current_goal = -1;
	reached_goal = true;
	num_voltas = 0;
	
	//Inicializa a posição estimada e a incerteza do robô
	pos.ResizeAndNulify(3, 1);
	realpos.ResizeAndNulify(3, 1);
	UpdatePositionWithAPI();
}

void Robot::Stop()
{
	SetTargetSpeed(0, 0);
}

void Robot::Log(EnvMap envmap)
{
	printf("----------------------------------------------------\n");
	printf("realpos[]:           [%.2f, %.2f, %.2f]  //[X, Y, THETA]\n", realpos.mat[0][0], realpos.mat[1][0], realpos.mat[2][0]);
	printf("pos[]:               [%.2f, %.2f, %.2f]  //[X, Y, THETA]\n", pos.mat[0][0], pos.mat[1][0], pos.mat[2][0]);
	printf("Leitura dos Sonares: [%.2f, %.2f, %.2f]  //[F, L, R]\n", sonar_reading[2], sonar_reading[0], sonar_reading[1]);
	printf("Numero de voltas:    [%d]\n", num_voltas);
	
	float t_time = GetSimulationTimeInSecs(clientID);
	printf("Tempo de simulação:  [%.2f, %0d:%0d]              //[Em segundos, Em minutos]\n", t_time,(int) t_time/60,(int) t_time%60);
}

void Robot::Update(EnvMap testmap)
{	
	//Guarda a posicao real da api para comparação
	GetAPIPosition();

	//Atualiza a posição usando odometria
	UpdatePositionWithOdometry();
	
	//Faz a leitura dos sonares
	UpdateSonarReadings();
	
	//Atualiza a posição atual do robô usando os sensores e o mapa
	UpdatePositionWithSensorsAndMap(testmap);
	
	//Executa o controle de movimento
	ExecuteMotionControl();
	
	//Verifica se já chegou no objetivo
	if(reached_goal)
	{
		current_goal++;
		
		if(current_goal >= (int) path.size())
		{
			current_goal = 0;
			num_voltas++;
		}
			
		goal[0] = path[current_goal][0];
		goal[1] = path[current_goal][1];
		goal[2] = path[current_goal][2];
		reached_goal = false;
	}
}

void Robot::GetAPIPosition()
{
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

	realpos.mat[0][0] = real[0];
	realpos.mat[1][0] = real[1];
    realpos.mat[2][0] = orientation[2];
}

void Robot::ExecuteMotionControl()
{	
    simxFloat theta = pos.mat[2][0];
    simxFloat dtheta = smallestAngleDiff(goal[2], theta);
 
    simxFloat deltax = goal[0] - pos.mat[0][0];
    simxFloat deltay = goal[1] - pos.mat[1][0];
    simxFloat rho = sqrt(deltax*deltax + deltay*deltay);
 
    simxFloat atg = atan2(deltay, deltax);
    atg = to_pi_range(atg);
    simxFloat alpha = smallestAngleDiff(atg, theta);
    alpha = to_pi_range(alpha);
    simxFloat beta = goal[2] - theta - alpha;
    beta = to_pi_range(beta);
 
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

void Robot::UpdateSonarReadings()
{
	sonar_reading[0] = readSonar(sonarL);
	sonar_reading[1] = readSonar(sonarR);
	sonar_reading[2] = readSonar(sonarF);
}

void Robot::UpdatePositionWithAPI()
{
	//Lê posição real e guarda em realpos[]
	GetAPIPosition();

	//Utiliza a posição de referência realpos[]
	pos.mat[0][0] = realpos.mat[0][0];
	pos.mat[1][0] = realpos.mat[1][0];
	pos.mat[2][0] = realpos.mat[2][0];

	//A incerteza é 0 já que a posição é precisa
	sigmapos.ResizeAndNulify(3, 3);
}

void Robot::UpdatePositionWithOdometry()
{
	//Leitura do odometro para saber as variações dPhiL e dPhiR 
	readOdometers();
	
	//Cálculo de deltaX, deltaY e deltaTheta
	float deltaSl = WHEEL1_R * dPhiL;
	float deltaSr = WHEEL1_R * dPhiR;
	float b = (2*WHEEL_L);

	float deltaS = (deltaSl + deltaSr) / 2.0;
	float deltaTheta = (deltaSr - deltaSl) / b;

	float argumento = pos.mat[2][0] + (deltaTheta/2);
	
	float deltaX = deltaS * cos(argumento);
	float deltaY = deltaS * sin(argumento);
	
	//Atualiza a média da posição	
	pos.mat[0][0] += deltaX;
	pos.mat[1][0] += deltaY;
	pos.mat[2][0] += deltaTheta;
	
	//Calcula o sigmaDelta
	Matrix sigmaDelta(2, 2);
	sigmaDelta.mat[0][0] = kr * fabs(deltaSr);
	sigmaDelta.mat[1][1] = kl * fabs(deltaSl);
	
	//Calcula Fp
	static Matrix fp(3, 3);
	fp.mat[0][0] = 1;
	fp.mat[0][1] = 0;
	fp.mat[0][2] = -deltaY;
	fp.mat[1][0] = 0;
	fp.mat[1][1] = 1;
	fp.mat[1][2] = deltaX;
	fp.mat[2][0] = 0;
	fp.mat[2][1] = 0;
	fp.mat[2][2] = 1;
	
	//Calcula fDeltaRl
	static Matrix fDeltaRl(3, 2);
	fDeltaRl.mat[0][0] = cos(argumento)/2.0 - (deltaS*sin(argumento)) / (2.0*b);
	fDeltaRl.mat[0][1] = cos(argumento)/2.0 + (deltaS*sin(argumento)) / (2.0*b);
	fDeltaRl.mat[1][0] = sin(argumento)/2.0 + (deltaS*cos(argumento)) / (2.0*b);
	fDeltaRl.mat[1][1] = sin(argumento)/2.0 - (deltaS*cos(argumento)) / (2.0*b);
	fDeltaRl.mat[2][0] = 1/b;
	fDeltaRl.mat[2][1] = -1/b;
	
	//Calcula fDeltaRl transposta
	Matrix fDeltaRlT = Transpose(fDeltaRl);
	
	//Atualiza matriz de covariancias da posição estimada
	Matrix result = ((fp * sigmapos) * fp) + ((fDeltaRl * sigmaDelta) * fDeltaRlT);
	sigmapos = result;
	
	//Por enquanto usando posição da API (remover depois)
	//UpdatePositionWithAPI();
}



void Robot::UpdatePositionWithSensorsAndMap(EnvMap testmap)
{
	//Calcula sigmav
	Matrix sigmav = sigmapos + R;

	//Calcula Kt
	Matrix invSigmav = Invert3x3(sigmav);
	Matrix K = sigmapos * invSigmav;

	//Calcula xz
	Matrix xz = realpos; //OBS: ACHAR xz DE ALGUMA FORMA

	//Calcula Vt  
	Matrix v = xz - pos; 

	//Atualiza posição
	Matrix result = pos + (K * v);
	pos = result;
	
	//Atualiza modelo de incerteza de posição
	Matrix result2 = sigmapos - ((K * sigmav) * Transpose(K));
	sigmapos = result2;
	
	//Por enquanto usando API (remover depois)
	//UpdatePositionWithAPI();
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

void Robot::SetTargetSpeed(simxFloat phiL, simxFloat phiR)
{
    simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, simx_opmode_oneshot);  
}
