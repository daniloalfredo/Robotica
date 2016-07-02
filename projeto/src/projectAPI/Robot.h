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
		//Mapa do ambiente
		EnvMap envmap;	

		//Caminho do robô
		std::vector< std::vector<float> > path;	//Vetor de objetivos do robô
		float goal[3];							//Objetivo atual
		int current_goal;						//Índice do objetivo atual no path
		bool reached_goal;						//Se chegou ao objetivo atual
		bool loop_path;							//Se é pra repetir o caminho quando terminar
		bool finishedTask;						//Se cumpriu todos os objetivos
		int num_voltas;							//Quantidade de voltas que o robô deu no percurso

		//Posição do robô
		Matrix realpos;							//posição verdadeira de referencia da API do VREP
		Matrix pos;								//posição estimada pelo robô

		//Leituras dos sonares
		enum SONAR_ID { LEFT = 0, FRONT = 1, RIGHT = 2 };
		float sonarReading[3];

		//Constantes do controle de movimento
		float K_RHO;
		float K_ALPHA;
		float K_BETA;
		float WHEEL_R;
		float WHEEL_L;				
		
		//Variáveis da odometria
		float kl, kr;
		float acumulatedDistance;

		//Variáveis do Filtro de Kalman
		Matrix sigmapos;					//Matriz de covariância da posição estimada (modelo de incerteza)
		Matrix R;							//Matriz de covariância do sensor

		//Funções auxiliares
		void UpdateSonarReadings(); 		//Atualiza a leitura dos sonares do robô e armazena em sonarReading[]
		void UpdatePositionWithAPI();		//Atualiza pos e sigmapos usando precisão perfeita da API (apenas no VREP)
		void ManageObjectives();			//Gerencia os objetivos do robô
		void MotionControl();		 		//Faz o robô andar em direção ao objetivo atual
		void ActionUpdate();				//Atualiza pos e sigmapos após movimento do robô usando a odometria
		bool PerceptionUpdateCondition();	//Decide quando fazer atualização de percepção	
		void PerceptionUpdate();			//Melhora a estimativa de posição do robô com os sensores e o mapa
		Matrix EstimateXz(); 				//Faz uma estimativa da posição do robô com base nos sensores
	
	public:
		//Funções principais de interface do robô
		Robot(EnvMap envmap);
		bool LoadPath(const char* PATH_FILENAME);
		bool FinishedWork();
		void Log();
		void Update();
};

#endif
