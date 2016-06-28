#include "Robot.h"

Robot::Robot()
{
	//Inicializa as constantes de controle de movimento
	K_RHO = 0.07;
	K_ALPHA = 2.0;
	K_BETA = -0.25;
	WHEEL_R = 0.0325;
	WHEEL_L = 0.075;
	
	//Inicializa variáveis de odometria
	kl = 0.01;
	kr = 0.01;
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

void Robot::Log(EnvMap envmap)
{
	printf("\r----------------------------------------------------\n");
	printf("\rPosição Real:        [%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", realpos.mat[0][0], realpos.mat[1][0], realpos.mat[2][0]);
	printf("\rPosição Estimada:    [%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", pos.mat[0][0], pos.mat[1][0], pos.mat[2][0]);
	printf("\rErro de Posição:     [%.4f, %.4f, %.4f]\n", fabs(realpos.mat[0][0]-pos.mat[0][0]), fabs(realpos.mat[1][0]-pos.mat[1][0]), fabs(smallestAngleDiff(realpos.mat[2][0], pos.mat[2][0])));
	printf("\rVariância de Posição:[%.4f, %.4f, %.4f]  //[X, Y, THETA]\n", sigmapos.mat[0][0], sigmapos.mat[1][1], sigmapos.mat[2][2]);
	printf("\rLeitura dos Sonares: [%f, %f, %f]  //[F, L, R]\n", sonar_reading[2], sonar_reading[0], sonar_reading[1]);
	printf("\rNumero de voltas:    [%d]\n", num_voltas);
	float t_time = APIGetSimulationTimeInSecs();
	printf("\rTempo de simulação:  [%.2dm %.2ds]\n", (int) t_time/60,(int) t_time%60);
}

void Robot::Update(EnvMap envmap)
{		
	APIGetTrueRobotPosition(&realpos);

	//Verifica se já chegou no objetivo
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
				APIStopRobot();
				return;
			}
		}
		
		current_goal++;	
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
		&& (acumulatedDistance > 1.0 /*|| (sigmapos.mat[0][0] >= 1.0 || sigmapos.mat[1][1] >= 1.0 || sigmapos.mat[2][2] >= 20.0*PI_DIV_180)*/)
	  )
	{
		//APIStopRobot();
		//UpdatePositionWithSensorsAndMap(envmap);
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
    
    if(v < 0.15) //velocidade mínima
    	v = 0.15;
    else if(v > 0.8) //velocidade máxima
    	v = 0.8;

    float w = (K_ALPHA * alpha + K_BETA * beta);
 
    float wR = 2*v + WHEEL_L*w;
    float wL = wR - 2*WHEEL_L*w;
 
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
	//printf("\rV: %f \tWheel Speed: %f \t %f\n", v, phiL, phiR);
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
	pos.mat[2][0] = realpos.mat[2][0];

	//A incerteza é 0 já que a posição é precisa
	sigmapos.ResizeAndNulify(3, 3);
}

void Robot::UpdatePositionWithOdometry()
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
	float deltaX = deltaS * cos(argumento);
	float deltaY = deltaS * sin(argumento);

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
	fp.mat[0][2] = -fabs(deltaY);
	fp.mat[1][0] = 0;
	fp.mat[1][1] = 1;
	fp.mat[1][2] = fabs(deltaX);
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
	
	//Atualiza matriz de covariancias da posição estimada
	sigmapos = ((fp * sigmapos) * Transpose(fp)) + ((fDeltaRl * sigmaDelta) * Transpose(fDeltaRl));

sigmapos.Print();
//printf("\rtheta %f \tdPhiR %f\n", pos.mat[2][0], dPhiR);	

	//Limita as variâncias para um valor máximo
	//sigmapos.mat[0][0] = fmin(4.0, sigmapos.mat[0][0]);
	//sigmapos.mat[1][1] = fmin(4.0, sigmapos.mat[1][1]);
	//sigmapos.mat[2][2] = fmin(2*PI, sigmapos.mat[2][2]);
}

void Robot::UpdatePositionWithSensorsAndMap(EnvMap envmap)
{
	//Calcula sigmav
	Matrix sigmav = sigmapos + R;

	//Calcula Kt
	Matrix K = sigmapos * Invert3x3(sigmav);

	//Calcula xz
	Matrix xz = EstimateXz(envmap);

	//Calcula Vt  
	Matrix v = xz - pos; 

	//Atualiza posição
	pos = pos + (K * v);
	
	//Atualiza modelo de incerteza de posição
	sigmapos = sigmapos - ((K * sigmav) * Transpose(K));
}

Matrix Robot::EstimateXz(EnvMap envmap)
{
	static Matrix xz(3, 1);

	printf("\rERRO DA ESTIMATIVA XZ: [%.2fcm, %.2fcm, %.2f°]\n", 100.0*fabs(realpos.mat[0][0]-xz.mat[0][0]), 100.0*fabs(realpos.mat[1][0]-xz.mat[1][0]), to_deg(fabs(smallestAngleDiff(realpos.mat[2][0], xz.mat[2][0]))));

	//Simula a melhor estimativa possível
	xz = realpos;
	
	return xz;
}

/*Matrix Robot::EstimateXz(EnvMap envmap)
{
	static Matrix xz(3, 1);
	
	float deviationX = 2*sqrt(sigmapos.mat[0][0]);
	float deviationY = 2*sqrt(sigmapos.mat[1][1]);
	float deviationTheta = 2*sqrt(sigmapos.mat[2][2]);
	float stepX = 0.0;
	float stepY = 0.0;
	float stepTheta = 0.0;
	float diffX = 0.00001;
	float diffY = 0.00001;
	float diffTheta = 0.0001;
	float bestCompat = 0.0;
	static float stepIncX = 0.001;
	static float stepIncY = 0.001;
	static float stepIncTheta = 0.001 * PI_DIV_180;
	static float sensorDeviation = 0.8;

	while(diffX < deviationX)
	{
		while(diffY < deviationY)
		{
			while(diffTheta < deviationTheta)
			{
				float x = pos.mat[0][0] + diffX;
				float y = pos.mat[1][0] + diffY;
				float theta = pos.mat[2][0] + diffTheta;

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

				//Inclrementa o diffY
				if(diffTheta >= 0)
					diffTheta *= -1;
				else
				{
					diffTheta *= -1;
					diffTheta += stepTheta;
					stepTheta += stepIncTheta;
				}
			}

			//Inclrementa o diffY
			if(diffY >= 0)
				diffY *= -1;
			else
			{
				diffY *= -1;
				diffY += stepY;
				stepY += stepIncY;
			}
		}

		//Incrementa o diffX
		if(diffX >= 0)
			diffX *= -1;
		else
		{
			diffX *= -1;
			diffX += stepX;
			stepX += stepIncX;
		}
	}

	printf("\rERRO DA ESTIMATIVA XZ: [%.2fcm, %.2fcm, %.2f°]\n", 100.0*fabs(realpos.mat[0][0]-xz.mat[0][0]), 100.0*fabs(realpos.mat[1][0]-xz.mat[1][0]), to_deg(fabs(smallestAngleDiff(realpos.mat[2][0], xz.mat[2][0]))));

	//Simula a melhor estimativa possível
	//xz = realpos;
	
	return xz;
}*/

/*Matrix Robot::EstimateXz(EnvMap envmap)
{
	static Matrix xz(3, 1);

	static float MAP_SIZE_X = 4.0;
	static float MAP_SIZE_Y = 4.0;

	//float diffX = 1.8;
	//float diffY = 1.8;
	//float diffTheta = PI;

	static float stepX = 0.025;//diffX / 81.0;
	static float stepY = 0.025;//diffY / 81.0;
	static float stepTheta = 2.5*PI_DIV_180;//diffTheta / 81.0;
	static float sensorDeviation = 0.075;

	float minX = -MAP_SIZE_X/2.0;//pos.mat[0][0] - diffX;
	float maxX = MAP_SIZE_X/2.0;//pos.mat[0][0] + diffX;
	float minY = -MAP_SIZE_Y/2.0;//pos.mat[1][0] - diffY;
	float maxY = MAP_SIZE_Y/2.0;//pos.mat[1][0] + diffY;
	float minTheta = 0.0;//pos.mat[2][0] - diffTheta;
	float maxTheta = 359.0*PI_DIV_180;//pos.mat[2][0] + diffTheta;
	
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
				//printf("\rCompatSum: %f\n", compatSum);
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

	printf("\rERRO DA ESTIMATIVA XZ: [%.4f, %.4f, %.4f]\n", fabs(realpos.mat[0][0]-xz.mat[0][0]), fabs(realpos.mat[1][0]-xz.mat[1][0]), fabs(smallestAngleDiff(realpos.mat[2][0], xz.mat[2][0])));
	
	//Simula a melhor estimativa possível
	xz = realpos;
	
	return xz;
}*/