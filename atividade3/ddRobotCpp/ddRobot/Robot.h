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
		simxFloat pos[3];
		simxFloat goal[3];
		
		//Leituras dos sonares e das odometrias
		simxFloat sonar_reading[3]; // [L, R, F]
		
		//Funções auxiliares
		void readOdometers(int clientID, simxFloat &dPhiL, simxFloat &dPhiR);
		simxFloat readSonar(int clientID, simxInt &sonar);
	
	public:
		void Init(simxInt clientID);
		void Log(EnvMap envmap);
		void SetNextGoal(simxFloat x, simxFloat y, simxFloat theta);
		void GetAPIPosition(simxInt clientID); //seta ´pos´ usando a leitura precisa da API
		void GetSonarReadings(simxInt clientID); // atualiza a leitura dos sonares
		int ExecuteMotionControl(simxInt clientID); //faz o robô andar em direção ao alvo
		void SetTargetSpeed(int clientID, simxFloat phiL, simxFloat phiR);
		//bool HasReachedGoal();
		
		simxFloat* GetPos() { return pos; } //posição atual do robô
		float GetSensorFReading() { return sonar_reading[2]; }
};

#endif
