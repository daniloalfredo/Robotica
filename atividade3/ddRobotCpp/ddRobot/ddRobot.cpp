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
        
        //Define caminho para o robô seguir
        int current_goal = 0;

        std::vector<simxFloat*> path;
        //Landmark Amarelo
        simxFloat goal1[3]; goal1[0] = -1.0; goal1[1] = 1.5; goal1[2] = PI/2;
        path.push_back(goal1);

        simxFloat goal2[3]; goal2[0] = 0; goal2[1] = 0.5; goal2[2] = -PI/4;
        path.push_back(goal2);
        //Landmark Verde
        simxFloat goal3[3]; goal3[0] = 1.0; goal3[1] = -1.5; goal3[2] = -PI/2;
        path.push_back(goal3);
        //Landmark Azul
        simxFloat goal4[3]; goal4[0] = 1.0; goal4[1] = 1.5; goal4[2] = PI/2;
        path.push_back(goal4);

        simxFloat goal5[3]; goal5[0] = 0; goal5[1] = -0.5; goal5[2] = -3*PI/4;
        path.push_back(goal5);
        //Landmark Vermelho
        simxFloat goal6[3]; goal6[0] = -1.0; goal6[1] = -1.5; goal6[2] = -PI/2;
        path.push_back(goal6);

        simxFloat goal7[3]; goal7[0] = 0.25; goal7[1] = 0; goal7[2] = PI/2;
        path.push_back(goal7);

        
        //Inicializa o Robô
        Robot monstrinho;
        monstrinho.Init(clientID);
		monstrinho.SetNextGoal(path[current_goal][0], path[current_goal][1], path[current_goal][2]); //[m, m, rads]
       
       	//Começa a simulação
        if(simxStartSimulation(clientID, simx_opmode_oneshot_wait) != -1)
        {
        	printf("Simulação iniciada.\n");
        	
        	//Enquanto está conectado
		    while (simxGetConnectionId(clientID) != -1)
		    {    
		    	//Lê tempo de simulação do ultimo comando
		        simxInt time = getSimTimeMs(clientID); //Simulation time in ms or 0 if sim is not running
		       
		        //Encerra o loop se a simulcao acabou
		        if (time == 0){
		        	printf("Parando Simulação...\n");
		            break;  
				}
		    
		        //Lê a posição atual do robô
		        monstrinho.GetAPIPosition(clientID);
		       
				//Faz a leitura dos sonares
		        monstrinho.GetSonarReadings(clientID);

                //Se chegou no objetivo seta um novo
		        if(monstrinho.ExecuteMotionControl(clientID))
                {
	 				current_goal++;
	 			
	 				if(current_goal < path.size())
	 				{
	 					printf("Setando Proximo Objetivo...\n");
	 					monstrinho.SetNextGoal(path[current_goal][0], path[current_goal][1], path[current_goal][2]);
	 				}
                    else
                    {
                        current_goal = 0;
                        monstrinho.SetNextGoal(path[current_goal][0], path[current_goal][1], path[current_goal][2]);
                        goalCount++;
                        printf("Vezes que o objetivo total foi alcançado: %d\n", goalCount);

                    }
	 			}
	 
		        //Espera um tempo para o V-REP
		        extApi_sleepMs(2);
		    }
		   
		   	printf("Fim da simulação.\n");
		   
		    //Para o robô e desconecta do V-Rep;
		    monstrinho.SetTargetSpeed(clientID, 0, 0);
		    simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
		    simxFinish(clientID);
        } 
        
        else
        	printf("Nao foi possível iniciar a simulação.\n");
    } 
    
    else
        printf("Nao foi possivel conectar.\n");
   
    return 0;
}
