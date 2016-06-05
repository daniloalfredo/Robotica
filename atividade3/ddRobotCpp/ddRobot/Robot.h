#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string>
#include <unistd.h>
 
extern "C" {
#include "extApi.h"
/* #include "extApiCustom.h" if you wanna use custom remote API functions! */
}

#include "Utils.h"
#include "EnvMap.h"

class Robot
{
	private:
		//Constantes do controle de movimento
		float K_RHO;
		float K_ALPHA;
		float K_BETA;
		float WHEEL1_R;
		float WHEEL2_R;
		float WHEEL_L;
		
		//Componentes do robô
		simxInt ddRobotHandle;
		simxInt leftMotorHandle;
		simxInt rightMotorHandle;
		simxInt sensorHandle;
		simxInt graphOdometryHandle;
		simxInt sonarL;
		simxInt sonarR;
		simxInt sonarF;
		
		//Posição atual e próximo objetivo
		simxFloat lastPos[3];
		simxFloat pos[3];
		simxFloat goal[3];
		
		//Leituras dos sonares e das odometrias
		simxFloat sonar_reading[3]; // [L, R, F]
		
		//Caminho do robô
		std::vector<simxFloat*> path;
		int current_goal;
		bool reached_goal;
		
		//Variáveis da odometria
		//...
		
		//Funções auxiliares
		float DistanceBeetweenPos(simxFloat pos1[3], simxFloat pos2[3]);
		void readOdometers(int clientID, simxFloat &dPhiL, simxFloat &dPhiR);
		simxFloat readSonar(int clientID, simxInt &sonar);
	
	public:
		void Init(simxInt clientID, std::vector<simxFloat*> path);
		void Log(EnvMap envmap);
		void Update(simxInt clientID, EnvMap testmap);
		int ExecuteMotionControl(simxInt clientID); //faz o robô andar em direção ao alvo
		void SetTargetSpeed(int clientID, simxFloat phiL, simxFloat phiR);
		void UpdateSonarReadings(simxInt clientID); // atualiza a leitura dos sonares
		void UpdatePositionWithAPI(simxInt clientID);
		void UpdatePositionWithOdometry(simxInt clientID);
		void UpdatePositionWithSensorsAndMap(simxInt clientID, EnvMap testmap);
		
};

#endif
