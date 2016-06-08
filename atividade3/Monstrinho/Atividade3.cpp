#include "Robot.h"
 
#define V_REP_IP_ADDRESS "127.0.0.1"
#define V_REP_PORT 19997	//1999; 
           
int main(int argc, char* argv[])
{
    std::string ipAddr = V_REP_IP_ADDRESS;
    int portNb = V_REP_PORT;
    int goalCount = 0;
 
    if (argc > 1)
        ipAddr = argv[1];

    printf("Iniciando conexao com: %s...\n", ipAddr.c_str());
 
 	//Pega o ID do cliente
    simxInt clientID = simxStart((simxChar*) (simxChar*) ipAddr.c_str(), portNb, true, true, 2000, 5);
   
	//Se conseguiu conectar executa o programa
    if (clientID != -1)
    {
        printf("Conexao efetuada\n");
        
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
        std::vector<simxFloat*> path;        
       	simxFloat goal1[3] = {-1.0, 1.5, PI/2};
        simxFloat goal2[3] = {0.0, 0.5, -PI/4};
        simxFloat goal3[3] = {1.0, -1.5, -PI/2};
        simxFloat goal4[3] = {1.0, 1.5, PI/2};
        simxFloat goal5[3] = {0.0, -0.5, -3*PI/4};
        simxFloat goal6[3] = {-1.0, -1.5, -PI/2};
        simxFloat goal7[3] = {0.25, 0.0, PI/2};
        path.push_back(goal1);
        path.push_back(goal2);
        path.push_back(goal3);
        path.push_back(goal4);
        path.push_back(goal5);
        path.push_back(goal6);
        path.push_back(goal7);
        
        //Inicializa o Robô
        Robot monstrinho;
        monstrinho.Init(clientID, path);
       
       	//Começa a simulação
        if(simxStartSimulation(clientID, simx_opmode_oneshot_wait) != -1)
        {
        	printf("Simulação iniciada.\n");
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while (simxGetConnectionId(clientID) != -1)
		    {      
		        //Encerra o loop se a simulacao acabou
		        if (simxGetLastCmdTime(clientID) == 0) //Retorna o tempo de simulação em ms ou 0 se o simulador não está rodando
		        {
		        	printf("Parando a simulação...\n");
		            break;  
				}
		    
		    	//Atualiza o robô
		    	monstrinho.Update(testmap);
		    	
		    	//Printa o log do robô
		    	monstrinho.Log(testmap);
	 
		        //Espera um tempo para o V-REP
		        extApi_sleepMs(2);
		    }
		    //---------------------------------------------------------
		   
		   	printf("Fim da simulação.\n");
		   
		    //Para o robô e desconecta do V-Rep;
		    monstrinho.Stop();
		    simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
		    simxFinish(clientID);
        } 
        
        else
        	printf("Não foi possível iniciar a simulação.\n");
    } 
    
    else
        printf("Não foi possível conectar.\n");
   
    return 0;
}
