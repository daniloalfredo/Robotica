#include "RobotAPI.h"
#include "Robot.h"

#define MAP_FILENAME "ini/envmapBig.ini"
#define PATH_FILENAME "ini/pathBig.ini"
           
int main(int argc, char* argv[])
{   
	//Se conseguiu conectar com a API
    if (APIInitConection())
    {
        printf("\rConexão efetuada.\n");
        
        //Define o mapa do experimento
        EnvMap testmap(MAP_FILENAME);
        testmap.PrintMap();
        
        //Inicializa o Robô
        Robot monstrinho;
        monstrinho.LoadPath(PATH_FILENAME);
       
       	//Começa a simulação
        if(APIStartSimulation())
        {
        	printf("\rSimulação iniciada.\n");
        	
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
		   
		   	printf("\rFim da simulação.\n");
		    APIFinishSimulation();
        } 
        
        else
        	printf("\rNão foi possível iniciar a simulação.\n");
    } 
    
    else
        printf("\rNão foi possível conectar.\n");
   
    return 0;
}