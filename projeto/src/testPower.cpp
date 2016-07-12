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

        	float leftSpeed = 60.0;
        	float rightSpeed = 60.0;
        	float stepSpeed = 1.0;
        	float experimentTime = 2.0;
        	float stepTime = 0.1;
        	bool startTest = false;
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {    
		    	printf("\rTime: %.2fs  |  Power: [%.2f | %.2f]\n", experimentTime, leftSpeed, rightSpeed);

		    	if(APIGetKey() == 'u')
		    		experimentTime += stepTime;
		    	else if(APIGetKey() == 'j')
		    		experimentTime -= stepTime;
		    	else if(APIGetKey() == 'i')
		    		leftSpeed += stepSpeed;
		    	else if(APIGetKey() == 'k')
		    		leftSpeed -= stepSpeed;
		    	else if(APIGetKey() == 'o')
		    		rightSpeed += stepSpeed;
		    	else if(APIGetKey() == 'l')
		    		rightSpeed -= stepSpeed;
		    	else if(APIGetKey() == ' ')
		    		startTest = true;

		    	if(startTest)
		    	{
		    		float deltaSl = 0.0, deltaSr = 0.0, deltaS = 0.0;
		    		float WHEEL_R = 0.0375;
		    		float beginTime = APIGetSimulationTimeInSecs();

		    		while(APISimulationIsRunning() && (APIGetSimulationTimeInSecs() - beginTime < experimentTime))
		    		{
		    			deltaS += APIReadOdometers(WHEEL_R);
		    			float dl, dr;
		    			APIReadOdometers(WHEEL_R, &dl, &dr);
		    			deltaSl += dl;
		    			deltaSr += dr;

		    			APISetMotorPower(leftSpeed, rightSpeed);
		    			APIWait();
		    		}

		    		APIStopRobot();
		    		startTest = false;

		    		printf("\n\rΔSl: %f ΔSr: %f ΔS: %f\n\n", deltaSl, deltaSr, deltaS);
		    		APIWaitMsecs(3000);
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