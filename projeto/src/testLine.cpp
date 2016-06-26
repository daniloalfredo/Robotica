#include "RobotAPI.h"
#include "Utils.h"
           
int main(int argc, char* argv[])
{   
	//Se conseguiu conectar com a API
    if (APIInitConection())
    {
        printf("\rConexão efetuada.\n");
       
       	//Começa a simulação
        if(APIStartSimulation())
        {
        	printf("\rSimulação iniciada.\n");

        	float leftSpeed = 5.0;
        	float rightSpeed = 5.0;
        	float stepSpeed = 0.5;
        	bool startTest = false;
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {    
		    	printf("\rSpace to begin; Q to quit. Speed [Left | Right]: [%.2f | %.2f]\n", leftSpeed, rightSpeed);

		    	if(APIGetKey() == 'i')
		    		leftSpeed += 0.5;
		    	else if(APIGetKey() == 'k')
		    		leftSpeed -= 0.5;
		    	else if(APIGetKey() == 'o')
		    		rightSpeed += 0.5;
		    	else if(APIGetKey() == 'l')
		    		rightSpeed -= 0.5;
		    	else if(APIGetKey() == ' ')
		    		startTest = true;

		    	if(startTest)
		    	{
		    		float beginTime = APIGetSimulationTimeInSecs();
		    		APISetRobotSpeed(leftSpeed, rightSpeed);

		    		while(APISimulationIsRunning() && (APIGetSimulationTimeInSecs() - beginTime < 2.0)){
		    			APIWait();
		    		}

		    		APIStopRobot();
		    		startTest = false;
		    	}

		    	APIWait();
		    }
		    //---------------------------------------------------------
		   
		   	printf("\rFim da simulação.\n\r");
		   
		    //Para o robô e desconecta
		    APIFinishSimulation();
        } 
        
        else
        	printf("\rNão foi possível iniciar a simulação.\n");
    } 
    
    else
        printf("\rNão foi possível conectar.\n");
   
    return 0;
}