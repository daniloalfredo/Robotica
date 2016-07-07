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

        	float leftSpeed = 15.0;
        	float rightSpeed = 15.0;
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
		    		float prevpos[3];
		    		float afterpos[3];
		    		APIGetTrueRobotPosition(prevpos);

		    		float beginTime = APIGetSimulationTimeInSecs();
		    		APISetRobotSpeed(leftSpeed, rightSpeed);

		    		while(APISimulationIsRunning() && (APIGetSimulationTimeInSecs() - beginTime < 2.0))
		    		{
		    			APIWait();
		    		}

		    		APIStopRobot();
		    		startTest = false;

		    		APIGetTrueRobotPosition(afterpos);
		    		printf("\rΔX: %f ΔY: %f Δθ: %f\n", afterpos[0]-prevpos[0], afterpos[1]-prevpos[1], to_deg(smallestAngleDiff(afterpos[2], prevpos[2])));
		    	}

		    	APIWait();
		    }
		    //---------------------------------------------------------
		   
		   	printf("\rFim da simulação.\n\r");
		   
		    //Para o robô e desconecta
		    APIFinishSimulation();
        } 
        
        else
        	printf("\rNão foi possível iniciar a simulação.\n\r");
    } 
    
    else
        printf("\rNão foi possível conectar.\n\r");
   
    return 0;
}