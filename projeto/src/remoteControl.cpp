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

        	float leftSpeed = 0.0;
        	float rightSpeed = 0.0;
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {    
		    	//Controle de movimentos
		    	if(APIGetKey() == 'o')
		    	{
		    		leftSpeed = 15.0;
		    		rightSpeed = 15.0;
		    	}

		    	else if(APIGetKey() == 'l')
		    	{
		    		leftSpeed = -15.0;
		    		rightSpeed = -15.0;
		    	}

		    	else if(APIGetKey() == 'k')
		    	{
		    		leftSpeed = 10.0*fsignal(leftSpeed);
		    		rightSpeed = 15.0*fsignal(rightSpeed);
		    	}

		    	else if(APIGetKey() == ';')
		    	{
		    		leftSpeed = 15.0*fsignal(leftSpeed);
		    		rightSpeed = 10.0*fsignal(rightSpeed);
		    	}

		    	//Parar o robô
		    	else if(APIGetKey() == ' ')
		    	{
		    		leftSpeed = 0.0;
		    		rightSpeed = 0.0;
		    		APIStopRobot();
		    	}

		    	//Tirar Foto
		    	else if(APIGetKey() == 'p')
		    	{
		    		printf("\rTaking picture...\n");
		    		APISavePicture(APIReadCamera());
		    	}

		    	printf("\rSonars (L, F, R): [%.4f, %.4f, %.4f]\n", APIReadSonarLeft(), APIReadSonarFront(), APIReadSonarRight());

		    	APISetRobotSpeed(leftSpeed, rightSpeed);

		        //Espera um tempo antes da próxima atualização
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