#include "Robot.h"

Robot::Robot(EnvMap envmap)
{
	//Guarda o mapa do ambiente
	this->envmap = envmap;

	//Inicializa as constantes de controle de movimento
	K_RHO = 0.1;
	K_ALPHA = 2.0;
	K_BETA = -0.4;
	WHEEL_R = 0.0325;
	WHEEL_L = 0.075;
	
	//Inicializa variáveis de odometria
	kl = 0.1;
	kr = 0.1;
	acumulatedDistance = 0.0;
	
	//Variáveis do Filtro de Kalman
	R.Resize(3, 3);
	R.mat[0][0] = 0.1;
	R.mat[0][1] = 0.0;
	R.mat[0][2] = 0.0;
	R.mat[1][0] = 0.0;
	R.mat[1][1] = 0.1;
	R.mat[1][2] = 0.0;
	R.mat[2][0] = 0.0;
	R.mat[2][1] = 0.0;
	R.mat[2][2] = 5.0*PI_DIV_180; 
	
	//Variáveis do caminho
	current_goal = -1;
	reached_goal = true;
	finishedTask = false;
	num_voltas = 0;
	
	//Inicializa a posição estimada e a incerteza do robô
	pos.ResizeAndNulify(3, 1);
	realpos.ResizeAndNulify(3, 1);
	sigmapos.ResizeAndNulify(3, 3);
	posdeviation[0] = 0.0; posdeviation[1] = 0.0; posdeviation[2] = 0.0;
}

bool Robot::LoadPath(const char* PATH_FILENAME)
{
	FILE* file_path = fopen(PATH_FILENAME, "r");

	if(file_path != NULL)
	{
		float x, y, theta;

		//Se o path é em loop
		int loop_aux = 0;
		fscanf(file_path, "%*[^:] %*c %d", &loop_aux);
		loop_path = (loop_aux > 0);

		//Posição inicial
		fscanf(file_path, "%*[^:] %*c %f %f %f", &x, &y, &theta);
		pos.mat[0][0] = x; pos.mat[1][0] = y; pos.mat[2][0] = to_pi_range(theta*PI_DIV_180);

		//Pontos do path
		while(!feof(file_path))
		{
			fscanf(file_path, "%*[^:] %*c %f %f %f", &x, &y, &theta);
			std::vector<float> path_point;
			path_point.push_back(x);
			path_point.push_back(y);
			path_point.push_back(theta*PI_DIV_180);
			path.push_back(path_point);
		}

		fclose(file_path);
		return true;
	}

	else
	{
		printf("\rError. Could not open ´%s´.\n", PATH_FILENAME);
		return false;
	}
}

bool Robot::FinishedWork()
{
	return finishedTask;
}

void Robot::Log()
{
	printf("\r----------------------------------------------------\n");
	printf("\rPosição Real:        [%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", realpos.mat[0][0], realpos.mat[1][0], realpos.mat[2][0]);
	printf("\rPosição Estimada:    [%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", pos.mat[0][0], pos.mat[1][0], pos.mat[2][0]);
	printf("\rErro de Posição:     [%.4f, %.4f, %.4f°]\n", fabs(realpos.mat[0][0]-pos.mat[0][0]), fabs(realpos.mat[1][0]-pos.mat[1][0]), to_deg(fabs(smallestAngleDiff(realpos.mat[2][0], pos.mat[2][0]))));
	printf("\rDesvio de Posição:   [%.4f, %.4f, %.4f°]\n", posdeviation[0], posdeviation[1], to_deg(posdeviation[2]));
	printf("\rVariância de Posição:[%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", sigmapos.mat[0][0], sigmapos.mat[1][1], sigmapos.mat[2][2]);
	printf("\rLeitura dos Sonares: [%f, %f, %f]  //[L, F, R]\n", sonarReading[LEFT], sonarReading[FRONT], sonarReading[RIGHT]);
	printf("\rNumero de voltas:    [%d]\n", num_voltas);
	float t_time = APIGetSimulationTimeInSecs();
	printf("\rTempo de simulação:  [%.2dm %.2ds]\n", (int) t_time/60,(int) t_time%60);
}

void Robot::Update()
{	
	//Gerencia os objetivos do robô
	ManageObjectives();

	//Olha posição real do robô para comparação
	APIGetTrueRobotPosition(&realpos);

	//Faz a leitura dos sonares
	UpdateSonarReadings();

	//Passo de Atualização de Ação
	ActionUpdate();
		
	//Passo de Atualização de Percepção
	if(PerceptionUpdateCondition())
	{
		//APIStopRobot();
		PerceptionUpdate();
		acumulatedDistance = 0.0;
	}

	//Executa o controle de movimento
	MotionControl();
}

void Robot::UpdateSonarReadings()
{
	sonarReading[LEFT]  = APIReadSonarLeft();
	sonarReading[FRONT] = APIReadSonarFront();
	sonarReading[RIGHT] = APIReadSonarRight();
}

void Robot::UpdatePositionWithAPI()
{
	//Lê posição real e guarda em realpos
	APIGetTrueRobotPosition(&realpos);

	//Utiliza a posição de referência com realpos
	pos.mat[0][0] = realpos.mat[0][0];
	pos.mat[1][0] = realpos.mat[1][0];
	pos.mat[2][0] = realpos.mat[2][0];

	//A incerteza é 0 já que a posição é precisa
	sigmapos.ResizeAndNulify(3, 3);
}

void Robot::ManageObjectives()
{
	if(reached_goal)
	{
		if(current_goal >= (int) path.size()-1)
		{
			//Se tiver loop volta pro goal inicial
			if(loop_path)
			{
				current_goal = -1;
				num_voltas++;
			}

			//Se não para o robô
			else
			{
				printf("\rUltimo objetivo alcançado.\n");
				APIStopRobot();
				finishedTask = true;
				return;
			}
		}
		
		current_goal++;	
		goal[0] = path[current_goal][0];
		goal[1] = path[current_goal][1];
		goal[2] = path[current_goal][2];
		reached_goal = false;
	}
}

void Robot::MotionControl()
{	
    float theta = pos.mat[2][0];
    float dtheta = smallestAngleDiff(goal[2], theta);
 
    float deltax = goal[0] - pos.mat[0][0];
    float deltay = goal[1] - pos.mat[1][0];
    float rho = sqrt(deltax*deltax + deltay*deltay);
 
    float atg = to_pi_range(atan2(deltay, deltax));
    float alpha = to_pi_range(smallestAngleDiff(atg, theta));
    float beta = to_pi_range(goal[2] - theta - alpha);
 
    float v = K_RHO * rho;
    
    //printf("\rv: %f", v);

    static float MIN_SPEED = 0.07, MAX_SPEED = 0.15;
    if(v < MIN_SPEED)
    	v = MIN_SPEED;
    else if(v > MAX_SPEED)
    	v = MAX_SPEED;

    float w = (K_ALPHA * alpha + K_BETA * beta);
 
 	float wR = v + WHEEL_L*w;	//2*v + WHEEL_L*w;
    float wL = v - WHEEL_L*w;	//wR - 2*WHEEL_L*w;

    float phiL = wL / WHEEL_R;
    float phiR = wR / WHEEL_R;

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
    
    APISetRobotSpeed(phiL, phiR);

    //printf("\tWheel Speed: %f \t %f\n", phiL, phiR);
}

void Robot::ActionUpdate()
{
	//Leitura do odometro para saber as variações dPhiL e dPhiR
	float dPhiL, dPhiR;
	APIReadOdometers(&dPhiL, &dPhiR);
	
	//Cálculo de deltaX, deltaY e deltaTheta
	float deltaSl = WHEEL_R * dPhiL;
	float deltaSr = WHEEL_R * dPhiR;
	static float b = (2.0*WHEEL_L);

	float deltaS = (deltaSl + deltaSr) / 2.0;
	float deltaTheta = (deltaSr - deltaSl) / b;

	float argumento = pos.mat[2][0] + (deltaTheta/2.0);
	float sinargumento = sin(argumento);
	float cosargumento = cos(argumento);

	float deltaX = deltaS * cosargumento;
	float deltaY = deltaS * sinargumento;

	//Atualiza distância acumulada desde a ultima atualização de percepção
	acumulatedDistance += deltaS;

	//Atualiza a média da posição	
	pos.mat[0][0] += deltaX;
	pos.mat[1][0] += deltaY;
	pos.mat[2][0] = to_pi_range(pos.mat[2][0] + deltaTheta);

	//Calcula o sigmaDelta
	Matrix sigmaDelta(2, 2);
	sigmaDelta.mat[0][0] = kr * fabs(deltaSr);
	sigmaDelta.mat[0][1] = 0.0;
	sigmaDelta.mat[1][0] = 0.0;
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
	fDeltaRl.mat[0][0] = cosargumento/2.0 - (deltaS*sinargumento) / (2.0*b);
	fDeltaRl.mat[0][1] = cosargumento/2.0 + (deltaS*sinargumento) / (2.0*b);
	fDeltaRl.mat[1][0] = sinargumento/2.0 + (deltaS*cosargumento) / (2.0*b);
	fDeltaRl.mat[1][1] = sinargumento/2.0 - (deltaS*cosargumento) / (2.0*b);
	fDeltaRl.mat[2][0] = 1.0/b;
	fDeltaRl.mat[2][1] = -1.0/b;
	
	//Atualiza matriz de covariancias da posição estimada
	sigmapos = ((fp * sigmapos) * Transpose(fp)) + ((fDeltaRl * sigmaDelta) * Transpose(fDeltaRl));

	//Atualiza o desvio padrão da posição do robô	
	posdeviation[0] = fmin(envmap.GetSizeX(), sqrt(sigmapos.mat[0][0]));
	posdeviation[1] = fmin(envmap.GetSizeY(), sqrt(sigmapos.mat[1][1]));
	posdeviation[2] = fmin(PI_TIMES_2, sqrt(sigmapos.mat[2][2]));

	//Cheat desvio igual ao erro real
	posdeviation[0] = fabs(realpos.mat[0][0]-pos.mat[0][0]);//fmin(envmap.GetSizeX(), sqrt(sigmapos.mat[0][0]));
	posdeviation[1] = fabs(realpos.mat[1][0]-pos.mat[1][0]);//fmin(envmap.GetSizeY(), sqrt(sigmapos.mat[1][1]));
	posdeviation[2] = fabs(smallestAngleDiff(realpos.mat[2][0], pos.mat[2][0]));//fmin(PI_TIMES_2, sqrt(sigmapos.mat[2][2]));
}

bool Robot::PerceptionUpdateCondition()
{
	return
	(
		//Todos os sonares estão com leituras válidas
		(sonarReading[LEFT] >= 0 && sonarReading[FRONT] >= 0 && sonarReading[RIGHT] >= 0)
		
		&& 
			//O robô andou uma certa distancia ou o desvio em alguma variável passou de um limiar
			(
				acumulatedDistance > 0.5
				|| (posdeviation[0] >= 0.3 || posdeviation[1] >= 0.3 || posdeviation[2] >= 5.0*PI_DIV_180)
			)
	);
}

void Robot::PerceptionUpdate()
{
	//Calcula sigmav
	Matrix sigmav = sigmapos + R;

	//Calcula Kt
	Matrix K = sigmapos * Invert3x3(sigmav);

	//Calcula xz
	Matrix xz = EstimateXz();

	//Calcula Vt  
	Matrix v = xz - pos; 

	//Atualiza posição
	pos = pos + (K * v);
	
	//Atualiza modelo de incerteza de posição
	sigmapos = sigmapos - ((K * sigmav) * Transpose(K));
}

float Robot::RobotToSensorPointDist(float Rx, float Ry, float Rtheta, float Sx, float Sy, float Stheta, float sensorDist)
{
	float point[2];
	float pSR[2];

	//Sensor to robot
    pSR[0] = Sx + sensorDist * cos(Stheta);
    pSR[1] = Sy + sensorDist * sin(Stheta);
    
    //Robot to world
    float sintheta = sin(Rtheta);
    float costheta = cos(Rtheta);
    point[0] = Rx + (pSR[0] * costheta - pSR[1] * sintheta);
    point[1] = Ry + (pSR[0] * sintheta + pSR[1] * costheta);

	return envmap.DistanceToNearestWall(point[0], point[1]);
}

Matrix Robot::EstimateXz()
{	
	printf("\rCalculando XZ...\n");

	//Estimativa de posição
	static Matrix xz(3, 1);

	//Posicionamento dos sensores em relação ao robô
	static float sensorLeftPos[3]  = {0.03, 0, PI/2.0};
	static float sensorFrontPos[3] = {0.03, 0, 0};
	static float sensorRightPos[3] = {0.03, 0, -PI/2.0};

	static float sensorDeviation = 0.05;
	static float SONAR_ANGLE = 7.5*PI_DIV_180;

	static float stepX = envmap.GetSizeX() / 80.0;
	static float stepY = envmap.GetSizeY() / 80.0;
	static float stepTheta = 1.0*PI_DIV_180;
	static float stepDiffAngle = 1.5*PI_DIV_180;

	float minX = pos.mat[0][0] - posdeviation[0];
	float maxX = pos.mat[0][0] + posdeviation[0];
	float minY = pos.mat[1][0] - posdeviation[1];
	float maxY = pos.mat[1][0] + posdeviation[1];
	float minTheta = pos.mat[2][0] - posdeviation[2];
	float maxTheta = pos.mat[2][0] + posdeviation[2];

	float maxCompat = 0.0;

	for(float x = minX; x <= maxX; x += stepX)
	{
		for(float y = minY; y <= maxY; y += stepY)
		{
			for(float theta = minTheta; theta <= maxTheta; theta += stepTheta)
			{
				float dL = INFINITE_DISTANCE;
				float dF = INFINITE_DISTANCE;
				float dR = INFINITE_DISTANCE;

				float diffAngleMin = sensorLeftPos[2] - SONAR_ANGLE;
				float diffAngleMax = sensorLeftPos[2] + SONAR_ANGLE;	
				for(float diffAngle = diffAngleMin; diffAngle <= diffAngleMax; diffAngle += stepDiffAngle)
					dL = fmin(dL, RobotToSensorPointDist(x, y, theta, sensorLeftPos[0], sensorLeftPos[1], diffAngle, sonarReading[LEFT]));

				diffAngleMin = sensorFrontPos[2] - SONAR_ANGLE;
				diffAngleMax = sensorFrontPos[2] + SONAR_ANGLE;	
				for(float diffAngle = diffAngleMin; diffAngle <= diffAngleMax; diffAngle += stepDiffAngle)
					dF = fmin(dF, RobotToSensorPointDist(x, y, theta, sensorFrontPos[0], sensorFrontPos[1], diffAngle, sonarReading[FRONT]));

				diffAngleMin = sensorRightPos[2] - SONAR_ANGLE;
				diffAngleMax = sensorRightPos[2] + SONAR_ANGLE;	
				for(float diffAngle = diffAngleMin; diffAngle <= diffAngleMax; diffAngle += stepDiffAngle)
					dR = fmin(dR, RobotToSensorPointDist(x, y, theta, sensorRightPos[0], sensorRightPos[1], diffAngle, sonarReading[RIGHT]));

				//float compat = GaussianCompatibility(0.0, dL, sensorDeviation) * GaussianCompatibility(0.0, dF, sensorDeviation) * GaussianCompatibility(0.0, dR, sensorDeviation);
				//float b = GaussianCompatibility(pos.mat[0][0], x, posdeviation[0])*GaussianCompatibility(pos.mat[1][0], y, posdeviation[1])*GaussianCompatibility(pos.mat[2][0], theta, posdeviation[2]);
				float compat = HansGaussian(dL, sensorDeviation, stepX)*HansGaussian(dF, sensorDeviation, stepX)*HansGaussian(dR, sensorDeviation, stepX);	

				if(compat > maxCompat)
				{
					maxCompat = compat;
					xz.mat[0][0] = x;
					xz.mat[1][0] = y;
					xz.mat[2][0] = to_pi_range(theta);
				}	
			}
		}
	}

	printf("\rERRO DA ESTIMATIVA XZ: [%.2fm, %.2fm, %.2f°]\n", fabs(realpos.mat[0][0]-xz.mat[0][0]), fabs(realpos.mat[1][0]-xz.mat[1][0]), to_deg(fabs(smallestAngleDiff(realpos.mat[2][0], xz.mat[2][0]))));

	//Simula a melhor estimativa possível
	//xz = realpos;
	
	return xz;
}