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
	num_voltas = 0;
	
	//Inicializa a posição estimada e variâncias do robô
	UpdatePositionWithAPI();

	//Variáveis da odometria // Ignora essa parte
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
	printf("----------------------------------------------------\n");
	printf("realpos[]:           [%.2f, %.2f, %.2f]  //[X, Y, THETA]\n", realpos[0], realpos[1], realpos[2]);
	printf("pos[]:               [%.2f, %.2f, %.2f]  //[X, Y, THETA]\n", pos[0], pos[1], pos[2]);
	printf("posVariance[]:       [%.2f, %.2f, %.2f]  //[X, Y, THETA]\n", posVariance[0], posVariance[1], posVariance[2]);
	printf("Erro de Posição:     [%.2f, %.2f, %.2f]  //[E = realpos-pos]\n", realpos[0]-pos[0], realpos[1]-pos[1], realpos[2]-pos[2]);
	printf("Leitura dos Sonares: [%.2f, %.2f, %.2f]  //[F, L, R]\n", sonar_reading[2], sonar_reading[0], sonar_reading[1]);
	float t_time = GetSimulationTimeInSecs(clientID);
	printf("Tempo de simulação:  [%.2f, %0d:%0d]              //[Em segundos, Em minutos]\n", t_time,(int) t_time/60,(int) t_time%60);
	printf("Numero de voltas:    [%d]\n", num_voltas);
}

void Robot::Update(EnvMap testmap)
{	
	//Guarda a posicao real da api para comparação
	GetAPIPosition();

	//Atualiza a posição usando odometria
	UpdatePositionWithOdometry();
	
	//Faz a leitura dos sonares
	UpdateSonarReadings();
	
	//Se o erro na estimativa fica grande demais
	if(posVariance[0] >= 0.05 || posVariance[1] >= 0.05 || posVariance[2] >= 0.1*(PI/180.0))
	{	
		//Atualiza a posição atual do robô usando os sensores e o mapa
		printf("Corrigindo estimativa de posição...\n");
		UpdatePositionWithSensorsAndMap(testmap);
	}
	
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
	simxInt ret = simxGetObjectPosition(clientID, ddRobotHandle, -1, realpos, simx_opmode_oneshot_wait);
    if (ret > 0) {
        printf("Erro ao ler posição do robô.\n");
        return;
    }
 
    simxFloat orientation[3];
    ret = simxGetObjectOrientation(clientID, ddRobotHandle, -1, orientation, simx_opmode_oneshot_wait);
    if (ret > 0) {
        printf("Erro ao ler orientação do robô.\n");
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
	pos[0] = realpos[0];
	pos[1] = realpos[1];
	pos[2] = realpos[2];

	//A incerteza é 0 já que a posição é precisa
	posVariance[0] = 0.0;
	posVariance[1] = 0.0;
	posVariance[2] = 0.0;
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
	float deltaX = deltaS * cos(argumento);
	float deltaY = deltaS * sin(argumento);

	//Atualiza incertezas de posição
	//posVariance[0] += fabs(deltaX * ERROR_PER_METER_X);
	//posVariance[1] += fabs(deltaY * ERROR_PER_METER_Y);
	//posVariance[2] += fabs(deltaS * ERROR_PER_METER_THETA);

	//Atualiza estimativa de posição
	//pos[0] += deltaX;
	//pos[1] += deltaY;
	//pos[2] += deltaTheta;


	// Heitor here // \/ \/
	// Equações da página 67 - Slide 12	
	
	float p_linha[3];

	p_linha[0] = pos[0] + deltaX;
	p_linha[1] = pos[1] + deltaX;
	p_linha[2] = pos[2] + deltaTheta;


	// Equações da página 68 - Slide 12	
	// Lei da propagação dos Erros
	
	float kr = 1;
	float kl = 1;
	
	// Somatorio delta = covar(deltaSr,deltaSl);
	float covar[2][2];
	covar[0][0] = kr*abs(deltaSr);
	covar[1][1] = kl*abs(deltaSl);
	covar[0][1] = covar[1][0] = 0;
	

	// Fp , fp = fp_transposto // No slide diz que é igual
	// Mas se vc transpor, não da igual
	float fp[3][3], fpt[3][3];
	fp[0][0] = 1;
	fp[0][1] = 0;
	fp[0][2] = -(deltaY);
	fp[1][0] = 0;
	fp[1][1] = 1;
	fp[1][2] = (deltaX);
	fp[2][0] = 0;
	fp[2][1] = 0;
	fp[2][2] = 1;

	// fpt, considerando fp != fpt
	/*
	fpt[0][0] = fp[0][0];
	fpt[0][1] = fp[1][0];
	fpt[0][2] = fp[2][0];
	fpt[1][0] = fp[0][1];
	fpt[1][1] = fp[1][1];
	fpt[1][2] = fp[2][1];
	fpt[2][0] = fp[0][2];
	fpt[2][1] = fp[1][2];
	fpt[2][2] = fp[2][2];
	*/

	fpt = matrixTranspose(3,3,fp);

	// fDeltaRl
	float fDeltaRL[3][2];
	float argumento;  
	fDeltaRL[0][0] = cos(argumento)/2 - deltaS*sin(argumento)/(2*b);
	fDeltaRL[0][1] = cos(argumento)/2 + deltaS*sin(argumento)/(2*b);
	fDeltaRL[1][0] = sin(argumento)/2 + deltaS*cos(argumento)/(2*b);
	fDeltaRL[1][1] = sin(argumento)/2 - deltaS*cos(argumento)/(2*b);
	fDeltaRL[2][0] = 1/b;
	fDeltaRL[2][1] = -1/b;

	// fDeltaRLt 
	float fDeltaRLt[2][3];
	/*
	fDeltaRLt[0][0] = fDeltaRL[0][0];
	fDeltaRLt[0][1] = fDeltaRL[1][0];
	fDeltaRLt[0][2] = fDeltaRL[2][0];
	fDeltaRLt[1][0] = fDeltaRL[0][1];
	fDeltaRLt[1][1] = fDeltaRL[1][1];
	fDeltaRLt[1][2] = fDeltaRL[2][1];
	*/
	fDeltaRLt = matrixTranspose(3,2,fDeltaRL);

	// incertezaPosicaoInterior = fp * CovarianciaAnterior * fpt  
	// Covariancia inicial = {0000000000000000}
	// Essa matriz de covariancia tem que ser global, inicializada com zeros, e depois atualizadas
	// em cada passo futuro, vai usar ela pra atualizar
	float covarianciaAnterior[3][3];

	float incertezaPosicaoAnterior[3][3];
	/*
	// fp * Covaranancia 
	incertezaPosicaoAnterior[0][0] = fp[0][0]*covarianciaAnterior[0][0]+fp[0][1]*covarianciaAnterior[1][0]+fp[0][2]*covarianciaAnterior[2][0];
	incertezaPosicaoAnterior[0][1] = fp[0][0]*covarianciaAnterior[0][1]+fp[0][1]*covarianciaAnterior[1][1]+fp[0][2]*covarianciaAnterior[2][1];
	incertezaPosicaoAnterior[0][2] = fp[0][0]*covarianciaAnterior[0][2]+fp[0][1]*covarianciaAnterior[1][2]+fp[0][2]*covarianciaAnterior[2][2];

	incertezaPosicaoAnterior[1][0] = fp[1][0]*covarianciaAnterior[0][0]+fp[1][1]*covarianciaAnterior[1][0]+fp[1][2]*covarianciaAnterior[2][0];
	incertezaPosicaoAnterior[1][1] = fp[1][0]*covarianciaAnterior[0][1]+fp[1][1]*covarianciaAnterior[1][1]+fp[1][2]*covarianciaAnterior[2][1];
	incertezaPosicaoAnterior[1][2] = fp[1][0]*covarianciaAnterior[0][2]+fp[1][1]*covarianciaAnterior[1][2]+fp[1][2]*covarianciaAnterior[2][2];
	
	incertezaPosicaoAnterior[2][0] = fp[2][0]*covarianciaAnterior[0][0]+fp[2][1]*covarianciaAnterior[1][0]+fp[2][2]*covarianciaAnterior[2][0];
	incertezaPosicaoAnterior[2][1] = fp[2][0]*covarianciaAnterior[0][1]+fp[2][1]*covarianciaAnterior[1][1]+fp[2][2]*covarianciaAnterior[2][1];
	incertezaPosicaoAnterior[2][2] = fp[2][0]*covarianciaAnterior[0][2]+fp[2][1]*covarianciaAnterior[1][2]+fp[2][2]*covarianciaAnterior[2][2];

	// fp * Covaranancia * fDeltaRLt
	incertezaPosicaoAnterior[0][0] = incertezaPosicaoAnterior[0][0]*fdeltaRlt[0][0] + incertezaPosicaoAnterior[0][1]*fdeltaRlt[1][0] + incertezaPosicaoAnterior[0][2]*fdeltaRlt[2][0]; 
	incertezaPosicaoAnterior[0][1] = incertezaPosicaoAnterior[0][0]*fdeltaRlt[0][1] + incertezaPosicaoAnterior[0][1]*fdeltaRlt[1][1] + incertezaPosicaoAnterior[0][2]*fdeltaRlt[2][1]; 
	incertezaPosicaoAnterior[0][2] = incertezaPosicaoAnterior[0][0]*fdeltaRlt[0][2] + incertezaPosicaoAnterior[0][1]*fdeltaRlt[1][2] + incertezaPosicaoAnterior[0][2]*fdeltaRlt[2][2]; 

	incertezaPosicaoAnterior[1][0] = incertezaPosicaoAnterior[1][0]*fdeltaRlt[0][0] + incertezaPosicaoAnterior[1][1]*fdeltaRlt[1][0] + incertezaPosicaoAnterior[1][2]*fdeltaRlt[2][0]; 
	incertezaPosicaoAnterior[1][1] = incertezaPosicaoAnterior[1][0]*fdeltaRlt[0][1] + incertezaPosicaoAnterior[1][1]*fdeltaRlt[1][1] + incertezaPosicaoAnterior[1][2]*fdeltaRlt[2][1]; 
	incertezaPosicaoAnterior[1][2] = incertezaPosicaoAnterior[1][0]*fdeltaRlt[0][2] + incertezaPosicaoAnterior[1][1]*fdeltaRlt[1][2] + incertezaPosicaoAnterior[1][2]*fdeltaRlt[2][2];	
	
	incertezaPosicaoAnterior[2][0] = incertezaPosicaoAnterior[2][0]*fdeltaRlt[0][0] + incertezaPosicaoAnterior[2][1]*fdeltaRlt[1][0] + incertezaPosicaoAnterior[2][2]*fdeltaRlt[2][0]; 
	incertezaPosicaoAnterior[2][1] = incertezaPosicaoAnterior[2][0]*fdeltaRlt[0][1] + incertezaPosicaoAnterior[2][1]*fdeltaRlt[1][1] + incertezaPosicaoAnterior[2][2]*fdeltaRlt[2][1]; 
	incertezaPosicaoAnterior[2][2] = incertezaPosicaoAnterior[2][0]*fdeltaRlt[0][2] + incertezaPosicaoAnterior[2][1]*fdeltaRlt[1][2] + incertezaPosicaoAnterior[2][2]*fdeltaRlt[2][2]; 
	*/

	incertezaPosicaoAnterior = matrixMultiplication(3,3,3,3,fp,covarianciaAnterior);
	incertezaPosicaoAnterior = matrixMultiplication(3,3,3,3,incertezaPosicaoAnterior,fpt);

	// IncertezaNoMovimento
	float incertezaMovimento[3][3];

	float fDeltaRL[3][2];  
	float fDeltaRLt[2][3];

	incertezaMovimento = matrixMultiplication(3,2,2,2,fDeltaRL,covar);
	incertezaMovimento = matrixMultiplication(3,2,2,3,incertezaMovimento,fDeltaRLt);

	/// Fim da atualização da ação
	/// Proximo passo checar todas as equações acima
	/// Partir para atualização na percepção

	float P[3][3];
	P = matrixSum(3,3,3,3,incertezaPosicaoAnterior,incertezaMovimento);

	float R[3][3];
	R[0][0] = R[1][1] = R[2][2] = 0.2;
	R[0][1] = R[0][2] = R[1][0] = R[1][2] = R[2][0] = R[2][1] = 0; 
	
	// Ganho de Kalman = Kt
	float Kt[3][3];
	float sumPR[3][3];
	sumPR = matrixSum(3,3,3,3,P,R);	
	Kt = matrixMultiplication(3,3,3,3,P,sumPR);

	// vt = xzt - xt;
	// xt = xt + kt*vt;
	// Pt = Pt - Kt*Ev*Ktt
	// Ev = Pt + Rt 


	//Por enquanto usando posição da API (remover depois)
	//UpdatePositionWithAPI();
}



void Robot::UpdatePositionWithSensorsAndMap(EnvMap testmap)
{
	/*//Encontrando estimativa segundo sensores e mapa
	simxFloat zpos[3];
	simxFloat zVariance[3];
	
	//------------------------------------------------------------
	//TESTE
	//Assumindo que eu poderia encontrar o zpos e zVariance 
	//fazendo um teste do melhor caso onde a estimativa Z 
	//é a posicao real + um errinho aleatório
	//para ver se kalman funciona
	zVariance[0] = 0.1 * rand_beetween_0_and_1();
	zVariance[1] = 0.1 * rand_beetween_0_and_1();
	zVariance[2] = (1.8*(PI/180.0)) * rand_beetween_0_and_1();
	
	//Um ponto proximo da posição real dentro da variancia acima
	zpos[0] = realpos[0] + zVariance[0] * rand_beetween_0_and_1()*rand_signal();
	zpos[1] = realpos[1] + zVariance[1] * rand_beetween_0_and_1()*rand_signal();
	zpos[2] = realpos[2] + zVariance[2] * rand_beetween_0_and_1()*rand_signal();
	//------------------------------------------------------------
	
	//Utilizando filtro de Kalman
	pos[0] = pos[0] + (posVariance[0] / (posVariance[0] + zVariance[0])) * (zpos[0] - pos[0]);
	pos[1] = pos[1] + (posVariance[1] / (posVariance[1] + zVariance[1])) * (zpos[1] - pos[1]);
	pos[2] = pos[2] + (posVariance[2] / (posVariance[2] + zVariance[2])) * (zpos[2] - pos[2]); 

	posVariance[0] = posVariance[0] - ((posVariance[0] * posVariance[0]) / (posVariance[0] + zVariance[0])); 
	posVariance[1] = posVariance[1] - ((posVariance[1] * posVariance[1]) / (posVariance[1] + zVariance[1])); 
	posVariance[2] = posVariance[2] - ((posVariance[2] * posVariance[2]) / (posVariance[2] + zVariance[2]));*/
	
	//Por enquanto usando API (remover depois)
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

void Robot::SetTargetSpeed(simxFloat phiL, simxFloat phiR)
{
    simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, simx_opmode_oneshot);  
}
