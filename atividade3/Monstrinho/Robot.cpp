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
	
	//Inicializa variáveis de odometria
	kl = 1.0;
	kr = 1.0;
	
	//Variáveis do Filtro de Kalman
	R.Resize(3, 3);
	R.mat[0][0] = 0.05;
	R.mat[0][1] = 0.0;
	R.mat[0][2] = 0.0;
	R.mat[1][0] = 0.0;
	R.mat[1][1] = 0.05;
	R.mat[1][2] = 0.0;
	R.mat[2][0] = 0.0;
	R.mat[2][1] = 0.0;
	R.mat[2][2] = 0.01; 
	
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
	printf("Posição Real:        [%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", realpos.mat[0][0], realpos.mat[1][0], realpos.mat[2][0]);
	printf("Posição Estimada:    [%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", pos.mat[0][0], pos.mat[1][0], pos.mat[2][0]);
	printf("Leitura dos Sonares: [%f, %f, %f]  //[F, L, R]\n", sonar_reading[2], sonar_reading[0], sonar_reading[1]);
	printf("Numero de voltas:    [%d]\n", num_voltas);
	float t_time = GetSimulationTimeInSecs(clientID);
	printf("Tempo de simulação:  [%.2dm %.2ds]\n", (int) t_time/60,(int) t_time%60);
}

void Robot::Update(EnvMap testmap)
{	
	//Guarda a posicao real da api para comparação
	GetAPIPosition();
	
	//Faz a leitura dos sonares
	UpdateSonarReadings();

	//Passo de Atualização de Ação
	UpdatePositionWithOdometry();
	
	//Passo de Atualização de Percepção
	UpdatePositionWithSensorsAndMap(testmap);
	
	//Executa o controle de movimento
	ExecuteMotionControl();
	
	//Verifica se já chegou no objetivo
	if(reached_goal)
	{
		printf("Objetivo alcançado. Erro de posição: [%.4f, %.4f, %.4f] //Ex, Ey, Etheta\n", realpos.mat[0][0]-pos.mat[0][0], realpos.mat[1][0]-pos.mat[1][0], realpos.mat[2][0]-pos.mat[2][0]);
	
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
    phiL = wL/WHEEL1_R;
    phiR = wR/WHEEL2_R;

    if(rho < 0.08)
    {
		phiR = 2 * dtheta;
		phiL = -2 * dtheta;
		
		if(fabs(dtheta) >= 0.9)
		{
			phiR = 2 * dtheta;
			phiL = -2 * dtheta;
		}
		
		else
			reached_goal = true;
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
	//Lê posição real e guarda em realpos
	GetAPIPosition();

	//Utiliza a posição de referência com realpos
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
	Matrix result = (((fp * sigmapos) * fp) + ((fDeltaRl * sigmaDelta) * fDeltaRlT));
	sigmapos = result;
}

void Robot::UpdatePositionWithSensorsAndMap(EnvMap testmap)
{
	//Calcula sigmav
	Matrix sigmav = sigmapos + R;

	//Calcula Kt
	Matrix invSigmav = Invert3x3(sigmav);
	Matrix K = sigmapos * invSigmav;

	//Calcula xz
	Matrix xz = EstimateXz(testmap);

	//Calcula Vt  
	Matrix v = xz - pos; 

	//Atualiza posição
	Matrix result = pos + (K * v);
	pos = result;
	
	//Atualiza modelo de incerteza de posição
	Matrix result2 = sigmapos - ((K * sigmav) * Transpose(K));
	sigmapos = result2;
}

Matrix Robot::EstimateXz(EnvMap testmap)
{
	Matrix xz = pos;
	
	/*
	//---------------------------------------
	//ALGORITMO PARA ENCONTRAR XZ A PARTIR
	//DA LEITURA DOS SENSORES E DO MAPA
	//---------------------------------------
	
	float diffX = 0.2;//fabs(sigmapos.mat[0][0]);
	float diffY = 0.2;//fabs(sigmapos.mat[1][1]);
	float diffTheta = 1.0*(PI/180.0);//fabs(sigmapos.mat[2][2]);
	float minX = pos.mat[0][0] - diffX;
	float maxX = pos.mat[0][0] + diffX;
	float minY = pos.mat[1][0] - diffY;
	float maxY = pos.mat[1][0] + diffY;
	float minTheta = pos.mat[2][0] - diffTheta;
	float maxTheta = pos.mat[2][0] + diffTheta;
	float stepX = diffX / 20.0;
	float stepY = diffY / 20.0;
	float stepTheta =  diffTheta / 10.0;
	float sensorDeviation = 0.15;
	
	float bestCompat = 0.0;
	
	for(float x = minX; x <= maxX; x += stepX)
	{
		for(float y = minY; y <= maxY; y += stepY)
		{
			for(float theta = minTheta; theta <= maxTheta; theta += stepTheta)
			{
				//O que eu deveria estar vendo se estivesse nessa postura
				float desiredL = testmap.MapDistance2(x, y, theta+PI/2.0);
				float desiredR = testmap.MapDistance2(x, y, theta-PI/2.0);
				float desiredF = testmap.MapDistance2(x, y, theta);
				
				//Calcula compatibilidade com a medição real
				float compatL = Compatibility(desiredL, sonar_reading[0], sensorDeviation);
				float compatR = Compatibility(desiredR, sonar_reading[1], sensorDeviation);
				float compatF = Compatibility(desiredF, sonar_reading[2], sensorDeviation);
				
				float compatSum = compatL + compatR + compatF;
				
				if(compatSum > bestCompat)
				{
					bestCompat = compatSum;
					xz.mat[0][0] = x;
					xz.mat[1][0] = y;
					xz.mat[2][0] = theta;
				}
			}
		}
	}
	*/
	
	//Simula a melhor estimativa possível
	xz = realpos;
	
	return xz;
}

float Robot::Compatibility(float desiredMeasure, float realMeasure, float sensorDeviation)
{
	float distance = fabs(desiredMeasure - realMeasure);
	
	if(distance > 0.0)
		return 1.0 / distance;
	
	return 0.0;
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
