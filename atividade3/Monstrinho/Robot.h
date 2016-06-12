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
		//Client ID da API
		simxFloat clientID;
		
		//Componentes do robô
		simxInt ddRobotHandle;
		simxInt leftMotorHandle;
		simxInt rightMotorHandle;
		simxInt sonarL;
		simxInt sonarR;
		simxInt sonarF;
		
		//Constantes do controle de movimento
		float K_RHO;
		float K_ALPHA;
		float K_BETA;
		float WHEEL1_R;
		float WHEEL2_R;
		float WHEEL_L;
		
		//Posição do robô
		Matrix realpos;			//posição verdadeira de referencia da API
		Matrix pos;				//posição estimada pelo robô
		Matrix sigmapos;		//Matriz de covariância da posição estimada (modelo de incerteza)
		
		//Leituras dos sonares e das odometrias
		simxFloat sonar_reading[3]; 	//[L, R, F]
		
		//Caminho do robô
		simxFloat goal[3];				//Objetivo atual
		std::vector<simxFloat*> path;	//Vetor de objetivos do robô
		int current_goal;				//Índice do objetivo atual no path
		bool reached_goal;				//Se chegou ao objetivo atual
		int num_voltas;					//Quantidade de voltas que o robô deu no percurso
		
		//Variáveis da odometria
		simxFloat dPhiL;				//Diferenca de ângulo em PhiL desde o último comando de movimento 
		simxFloat dPhiR;				//Diferenca de ângulo em PhiR desde o último comando de movimento
		float kl, kr;
		float acumulated_distance;

		//Variáveis do Filtro de Kalman
		Matrix R;

		//Funções auxiliares
		void GetAPIPosition();			//Lê a posição verdadeira do robô e guarda no vetor realpos[]
		void ExecuteMotionControl(); 	//Faz o robô andar em direção ao objetivo atual
		void UpdateSonarReadings(); 	//Atualiza a leitura dos sonares do robô e armazena em sonar_reading[]
		void UpdatePositionWithAPI();	//Atualiza pos[] e posVariance[] usando precisão perfeita da API (apenas para testes)
		void UpdatePositionWithOdometry();	//Atualiza pos[] e posVariance[] após movimento do robô usando a odometria
		void UpdatePositionWithSensorsAndMap(EnvMap testmap); //Melhora a estimativa de posição do robô com os sensores e o mapa
		Matrix EstimateXz(EnvMap testmap);
		float Compatibility(float desiredMeasure, float realMeasure, float sensorDeviation);
		void readOdometers();	
		simxFloat readSonar(simxInt &sonar);
		void SetTargetSpeed(simxFloat phiL, simxFloat phiR);
	
	public:
		//Funções principais de interface do robô
		void Init(simxInt clientID, std::vector<simxFloat*> path);
		void Stop();
		void Log(EnvMap envmap);
		void Update(EnvMap testmap);
};

#endif
