#include "Robot.h"

Robot::Robot()
{
	//Inicializa as constantes de controle de movimento
	K_RHO = 0.07;
	K_ALPHA = 2.0;
	K_BETA = -0.25;
	WHEEL1_R = 0.0325;
	WHEEL2_R = 0.0325;
	WHEEL_L = 0.075;
	
	//Inicializa variáveis de odometria
	kl = 0.001;
	kr = 0.001;
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
	num_voltas = 0;
	
	//Inicializa a posição estimada e a incerteza do robô
	Stop();
	pos.ResizeAndNulify(3, 1);
	realpos.ResizeAndNulify(3, 1);
	sigmapos.ResizeAndNulify(3, 3);
}

bool Robot::LoadPath(const char* PATH_FILENAME)
{
	FILE* file_path = fopen(PATH_FILENAME, "r");

	if(file_path != NULL)
	{
		float x, y, theta;

		int loop_aux = 0;
		fscanf(file_path, "%*[^:] %*c %d", &loop_aux);
		loop_path = (loop_aux > 0);
		fscanf(file_path, "%*[^:] %*c %f %f %f", &x, &y, &theta);
		pos.mat[0][0] = x; pos.mat[1][0] = y; pos.mat[2][0] = theta*PI_DIV_180;

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
		printf("Error. Could not open ´%s´.\n", PATH_FILENAME);
		return false;
	}
}

void Robot::Stop()
{
	APISetRobotSpeed(0, 0);
}

void Robot::Log(EnvMap envmap)
{
	printf("----------------------------------------------------\n");
	printf("Posição Real:        [%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", realpos.mat[0][0], realpos.mat[1][0], realpos.mat[2][0]);
	printf("Posição Estimada:    [%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", pos.mat[0][0], pos.mat[1][0], pos.mat[2][0]);
	printf("Erro de Posição:     [%.4f, %.4f, %.4f]\n", fabs(realpos.mat[0][0]-pos.mat[0][0]), fabs(realpos.mat[1][0]-pos.mat[1][0]), fabs(realpos.mat[2][0]-pos.mat[2][0]));
	printf("Variância de Posição:[%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", sigmapos.mat[0][0], sigmapos.mat[1][1], sigmapos.mat[2][2]);
	printf("Leitura dos Sonares: [%f, %f, %f]  //[F, L, R]\n", sonar_reading[2], sonar_reading[0], sonar_reading[1]);
	printf("Numero de voltas:    [%d]\n", num_voltas);
	float t_time = APIGetSimulationTimeInSecs();
	printf("Tempo de simulação:  [%.2dm %.2ds]\n", (int) t_time/60,(int) t_time%60);
}

void Robot::Update(EnvMap envmap)
{		
	APIGetTrueRobotPosition(&realpos);

	//Verifica se já chegou no objetivo
	if(reached_goal)
	{
		current_goal++;
		
		if(current_goal >= (int) path.size())
		{
			if(loop_path)
			{
				current_goal = 0;
				num_voltas++;
			}

			else
				current_goal--;	
		}
			
		goal[0] = path[current_goal][0];
		goal[1] = path[current_goal][1];
		goal[2] = path[current_goal][2];
		reached_goal = false;
	}

	//Faz a leitura dos sonares
	UpdateSonarReadings();

	//Passo de Atualização de Ação
	UpdatePositionWithOdometry();
	
	//Passo de Atualização de Percepção
	if(
		(sonar_reading[0] >= 0 && sonar_reading[1] >= 0 && sonar_reading[2] >= 0)
		&& (acumulatedDistance > 0.25 || (sigmapos.mat[0][0] >= 0.25 || sigmapos.mat[1][1] >= 0.25 || sigmapos.mat[2][2] >= 5*PI_DIV_180))
	  )
	{
		Stop();
		UpdatePositionWithSensorsAndMap(envmap);
		acumulatedDistance = 0.0;
	}

	//Executa o controle de movimento
	ExecuteMotionControl();
}

void Robot::ExecuteMotionControl()
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
    
    if(v < 0.2) //velocidade mínima
    	v = 0.2;
    else if(v > 0.8) //velocidade máxima
    	v = 1.0;

    float w = (K_ALPHA * alpha + K_BETA * beta);
 
    float wR = 2*v + WHEEL_L*w;
    float wL = wR - 2*WHEEL_L*w;
 
    float phiL = wL / WHEEL1_R;
    float phiR = wR / WHEEL2_R;

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
}

void Robot::UpdateSonarReadings()
{
	sonar_reading[0] = APIReadSonarLeft();
	sonar_reading[1] = APIReadSonarRight();
	sonar_reading[2] = APIReadSonarFront();
}

void Robot::UpdatePositionWithAPI()
{
	//Lê posição real e guarda em realpos
	APIGetTrueRobotPosition(&realpos);

	//Utiliza a posição de referência com realpos
	pos.mat[0][0] = realpos.mat[0][0];
	pos.mat[1][0] = realpos.mat[1][0];
	pos.mat[2][0] = to_2pi_range(realpos.mat[2][0]);

	//A incerteza é 0 já que a posição é precisa
	sigmapos.ResizeAndNulify(3, 3);
}

void Robot::UpdatePositionWithOdometry()
{
	//Leitura do odometro para saber as variações dPhiL e dPhiR
	float dPhiL, dPhiR;
	APIReadOdometers(&dPhiL, &dPhiR);
	
	//Cálculo de deltaX, deltaY e deltaTheta
	float deltaSl = WHEEL1_R * dPhiL;
	float deltaSr = WHEEL1_R * dPhiR;
	static float b = (2.0*WHEEL_L);

	float deltaS = (deltaSl + deltaSr) / 2.0;
	float deltaTheta = (deltaSr - deltaSl) / b;
	float argumento = pos.mat[2][0] + (deltaTheta/2.0);
	float deltaX = deltaS * cos(argumento);
	float deltaY = deltaS * sin(argumento);

	acumulatedDistance += deltaS;

	//Atualiza a média da posição	
	pos.mat[0][0] += deltaX;
	pos.mat[1][0] += deltaY;
	pos.mat[2][0] = to_2pi_range(pos.mat[2][0] + deltaTheta);
	
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
	fp.mat[0][2] = fabs(deltaY);//-deltaY;
	fp.mat[1][0] = 0;
	fp.mat[1][1] = 1;
	fp.mat[1][2] = fabs(deltaX);//deltaX;
	fp.mat[2][0] = 0;
	fp.mat[2][1] = 0;
	fp.mat[2][2] = 1;
	
	//Calcula fDeltaRl
	static Matrix fDeltaRl(3, 2);
	fDeltaRl.mat[0][0] = cos(argumento)/2.0 - (deltaS*sin(argumento)) / (2.0*b);
	fDeltaRl.mat[0][1] = cos(argumento)/2.0 + (deltaS*sin(argumento)) / (2.0*b);
	fDeltaRl.mat[1][0] = sin(argumento)/2.0 + (deltaS*cos(argumento)) / (2.0*b);
	fDeltaRl.mat[1][1] = sin(argumento)/2.0 - (deltaS*cos(argumento)) / (2.0*b);
	fDeltaRl.mat[2][0] = 1.0/b;
	fDeltaRl.mat[2][1] = -1.0/b;
	
	//Calcula fDeltaRl transposta
	Matrix fDeltaRlT = Transpose(fDeltaRl);

	//Calcula a transposta de fp
	Matrix fpT = Transpose(fp);
	
	//Atualiza matriz de covariancias da posição estimada
	Matrix result = ((fp * sigmapos) * fpT) + ((fDeltaRl * sigmaDelta) * fDeltaRlT);
	sigmapos = result;
	
	//Limite máxio para as variâncias
	sigmapos.mat[0][0] = fmin(4.0, sigmapos.mat[0][0]);
	sigmapos.mat[1][1] = fmin(4.0, sigmapos.mat[1][1]);
	sigmapos.mat[2][2] = fmin(2*PI, sigmapos.mat[2][2]);
}

void Robot::UpdatePositionWithSensorsAndMap(EnvMap envmap)
{
	//Calcula sigmav
	Matrix sigmav = sigmapos + R;

	//Calcula Kt
	Matrix invSigmav = Invert3x3(sigmav);
	Matrix K = sigmapos * invSigmav;

	//Calcula xz
	Matrix xz = EstimateXz(envmap);

	//Calcula Vt  
	Matrix v = xz - pos; 

	//Atualiza posição
	Matrix result = pos + (K * v);
	pos = result;
	
	//Atualiza modelo de incerteza de posição
	Matrix result2 = sigmapos - ((K * sigmav) * Transpose(K));
	sigmapos = result2;
}

Matrix Robot::EstimateXz(EnvMap envmap)
{
	Matrix xz = pos;
	
	float deviationX = 2*sqrt(sigmapos.mat[0][0]);
	float deviationY = 2*sqrt(sigmapos.mat[1][1]);
	float deviationTheta = 2*sqrt(sigmapos.mat[2][2]);

	float diffX = deviationX;
	float diffY = deviationY;
	float diffTheta = deviationTheta;

	static float stepX = diffX / 5.0;
	static float stepY = diffY / 5.0;
	static float stepTheta =  diffTheta / 5.0;
	static float sensorDeviation = 0.05;

	float minX = pos.mat[0][0] - diffX;
	float maxX = pos.mat[0][0] + diffX;
	float minY = pos.mat[1][0] - diffY;
	float maxY = pos.mat[1][0] + diffY;
	float minTheta = pos.mat[2][0] - diffTheta;
	float maxTheta = pos.mat[2][0] + diffTheta;
	
	float bestCompat = 0.0;
	
	for(float x = minX; x <= maxX; x += stepX)
	{
		for(float y = minY; y <= maxY; y += stepY)
		{
			for(float theta = minTheta; theta <= maxTheta; theta += stepTheta)
			{
				//O que eu deveria estar vendo se estivesse nessa postura
				float desiredL = envmap.MapDistance2(x, y, theta+PI_DIV_2);
				float desiredR = envmap.MapDistance2(x, y, theta-PI_DIV_2);
				float desiredF = envmap.MapDistance2(x, y, theta);
				
				//Calcula compatibilidade com a medição real
				float compatL = GaussianCompatibility(desiredL, sonar_reading[0], sensorDeviation);
				float compatR = GaussianCompatibility(desiredR, sonar_reading[1], sensorDeviation);
				float compatF = GaussianCompatibility(desiredF, sonar_reading[2], sensorDeviation);
				
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

	printf("ERRO ESTIMATIVA: [%.4f, %.4f, %.4f]\n", fabs(realpos.mat[0][0]-xz.mat[0][0]), fabs(realpos.mat[1][0]-xz.mat[1][0]), fabs(realpos.mat[2][0]-xz.mat[2][0]));
	
	//Simula a melhor estimativa possível
	xz = realpos;
	
	return xz;
}