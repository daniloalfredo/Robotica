#include "RobotAPI.h"
#include "Robot.h"

#define MAP_FILENAME "ini/envmap.ini"
#define PATH_FILENAME "ini/path.ini"
           
int main(int argc, char* argv[])
{   
	//Se conseguiu conectar com a API
    if (APIInitConection())
    {
        printf("Conexão efetuada.\n");
        
        //Define o mapa do experimento
        EnvMap testmap(MAP_FILENAME);
        testmap.PrintMap();
        
        //Inicializa o Robô
        Robot monstrinho;
        monstrinho.Init();
        monstrinho.LoadPath(PATH_FILENAME);
       
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
	 
		        //Espera um tempo antes da próxima atualização
		        APIWait();
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