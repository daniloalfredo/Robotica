#include "RobotAPI.h"
#include "Robot.h"
           
int main(int argc, char* argv[])
{   
	//Se conseguiu conectar com a API
    if (APIInitConection())
    {
        printf("Conexão efetuada.\n");
        
        //Define o mapa do experimento
        EnvMap testmap;
        testmap.AddWall(-2.0, 2.0, 2.0, 2.0); //Parede superior
        testmap.AddWall(-2.0, -2.0, 2.0, -2.0); //Parede inferior
        testmap.AddWall(-2.0, 2.0, -2.0, -2.0); //Parede esquerda
        testmap.AddWall(2.0, 2.0, 2.0, -2.0); //Parede direita
        testmap.AddWall(-2.0, 0.0, 0.0, 0.0); //Parede central esquerda
        testmap.AddWall(0.0, 2.0, 0.0, 1.0); //Parede central cima
        testmap.AddWall(0.0, -1.0, 0.0, -2.0); //Parede central baixo
        testmap.PrintMap();
        
        //Define caminho para o robô seguir
        std::vector<float*> path;        
       	float goal1[3] = {-1.0, 1.5, PI/2};
        float goal2[3] = {0.0, 0.5, -PI/4};
        float goal3[3] = {1.0, -1.5, -PI/2};
        float goal4[3] = {1.0, 1.5, PI/2};
        float goal5[3] = {0.0, -0.5, -3*PI/4};
        float goal6[3] = {-1.0, -1.5, -PI/2};
        float goal7[3] = {0.25, 0.0, PI/2};
        path.push_back(goal1);
        path.push_back(goal2);
        path.push_back(goal3);
        path.push_back(goal4);
        path.push_back(goal5);
        path.push_back(goal6);
        path.push_back(goal7);
        
        //Inicializa o Robô
        Robot monstrinho;
        monstrinho.Init(path);
       
       	//Começa a simulação
        if(APIStartSimulation())
        {
        	printf("Simulação iniciada.\n");
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {      		    
		    	//Atualiza o robô
		    	monstrinho.Update(testmap);
		    	
		    	//Printa o log do robô
		    	monstrinho.Log(testmap);
	 
		        //Espera um tempo para o V-REP
		        APIWaitMsecs(2);
		    }
		    //---------------------------------------------------------
		   
		   	printf("Fim da simulação.\n");
		   
		    //Para o robô e desconecta do V-Rep;
		    monstrinho.Stop();
		    APIFinishSimulation();
        } 
        
        else
        	printf("Não foi possível iniciar a simulação.\n");
    } 
    
    else
        printf("Não foi possível conectar.\n");
   
    return 0;
}