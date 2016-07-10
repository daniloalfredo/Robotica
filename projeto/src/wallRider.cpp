#include "ObjectDetector.h"
#include "RobotAPI.h"
#include "Utils.h"

#define FILE_DICTIONARY "ini/dictionary.yml"
#define FILE_DATABASE "ini/objects.ini"
#define FILE_PARAMS "ini/detector.ini"
#define FILE_SVM "ini/svm.ini" 

#define INFINITE_DISTANCE 2.2

enum STATES { FIND_MIN_DIST, ADJUSTING };
enum SONARS { LEFT = 0, FRONT = 1, RIGHT = 2 };
           
void UpdateSonarReadings(float* sonarReadings)
{
	sonarReadings[LEFT] = APIReadSonarLeft();
	sonarReadings[FRONT] = APIReadSonarFront();
	sonarReadings[RIGHT] = APIReadSonarRight();

	if(sonarReadings[LEFT] < 0.0)
		sonarReadings[LEFT] = INFINITE_DISTANCE;
	if(sonarReadings[FRONT] < 0.0)
		sonarReadings[FRONT] = INFINITE_DISTANCE;
	if(sonarReadings[RIGHT] < 0.0)
		sonarReadings[RIGHT] = INFINITE_DISTANCE;
}

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

        	int state = FIND_MIN_DIST;
        	float sonarReadings[3] = { -1.0, -1.0, -1.0 };
        	float motionSpeed = 30.0;
        	float leftSpeed = 0.0;
        	float rightSpeed = 0.0;
        	float timeStamp = APIGetSimulationTimeInSecs();
        	float turningDuration;

        	float PROXIMITY_THRESHOLD = 0.15;
        	float distToWall = INFINITE_DISTANCE;
        	float minDist = INFINITE_DISTANCE;
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {   
		    	//Atualiza sonares
		    	UpdateSonarReadings(sonarReadings); 

		    	//Máquina de estados
		    	if(state == FIND_MIN_DIST)
		    	{
		    		leftSpeed = motionSpeed;
		    		rightSpeed = -motionSpeed;

		    		minDist = fmin(minDist, sonarReadings[RIGHT]);

		    		if(APIGetSimulationTimeInSecs() - timeStamp >= 1.5)
		    			state = ADJUSTING;
		    	}

		    	if(state == ADJUSTING)
		    	{
		    		leftSpeed = motionSpeed;
		    		rightSpeed = -motionSpeed;

		    		if(sonarReadings[RIGHT] <= minDist+0.001
		    		&& sonarReadings[RIGHT] >= minDist-0.001)
		    		{
		    			leftSpeed = 0.0;
						rightSpeed = 0.0;
		    			state = 2;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}
		    	}

		    	//APISetRobotSpeed(leftSpeed, rightSpeed);
		    	APISetMotorPower(leftSpeed, rightSpeed);

		    	//Printa Informações
		    	printf("\rSonars:[%.2f, %.2f, %.2f]\n", sonarReadings[0], sonarReadings[1], sonarReadings[2]);

		        //Espera um tempo antes da próxima atualização
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