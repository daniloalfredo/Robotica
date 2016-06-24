#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string>
#include <unistd.h>

#include "RobotAPI.h"
#include "EnvMap.h"
#include "Utils.h"

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
		
		//Posição do robô
		Matrix realpos;					//posição verdadeira de referencia da API do VREP
		Matrix pos;						//posição estimada pelo robô
		Matrix sigmapos;				//Matriz de covariância da posição estimada (modelo de incerteza)
		
		//Leituras dos sonares e das odometrias
		float sonar_reading[3]; 		//[L, R, F]
		
		//Caminho do robô
		std::vector< std::vector<float> > path;	//Vetor de objetivos do robô
		float goal[3];						//Objetivo atual
		int current_goal;					//Índice do objetivo atual no path
		bool reached_goal;					//Se chegou ao objetivo atual
		int num_voltas;						//Quantidade de voltas que o robô deu no percurso
		bool loop_path;						//Se é pra repetir o caminho quando terminar
		
		//Variáveis da odometria
		float kl, kr;
		float acumulatedDistance;

		//Variáveis do Filtro de Kalman
		Matrix R;

		//Funções auxiliares
		void ExecuteMotionControl(); 							//Faz o robô andar em direção ao objetivo atual
		void UpdateSonarReadings(); 						//Atualiza a leitura dos sonares do robô e armazena em sonar_reading[]
		void UpdatePositionWithAPI();							//Atualiza pos e sigmapos usando precisão perfeita da API (apenas no VREP)
		void UpdatePositionWithOdometry();						//Atualiza pos e sigmapos após movimento do robô usando a odometria
		void UpdatePositionWithSensorsAndMap(EnvMap envmap);	//Melhora a estimativa de posição do robô com os sensores e o mapa
		Matrix EstimateXz(EnvMap envmap); 						//Faz uma estimativa da posição do robô com base nos sensores
	
	public:
		//Funções principais de interface do robô
		void Init();
		bool LoadPath(const char* PATH_FILENAME);
		void Stop();
		void Log(EnvMap envmap);
		void Update(EnvMap envmap);
};

#endif
