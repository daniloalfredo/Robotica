#include "ObjectDetector.h"
#include "RobotAPI.h"
#include "Utils.h"

#define FILE_DICTIONARY "ini/dictionary.yml"
#define FILE_DATABASE "ini/objects.ini"
#define FILE_PARAMS "ini/detector.ini"
#define FILE_SVM "ini/svm.ini" 

#define INFINITE_DISTANCE 2.2

enum STATES { WALKING, STOPPING, DETECTING, TURNING };
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

float RandomTimeSecs(float maxTimeSecs)
{
	return fmax(0.3, (rand()%((int)(maxTimeSecs*1000))) / 1000.0);
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

        	//Inicializa detector de objetos
        	/*ObjectDetector objectDetector;
			objectDetector.LoadParams(FILE_PARAMS);
			objectDetector.LoadObjects(FILE_DATABASE);

			if(objectDetector.LoadDictionary(FILE_DICTIONARY))
			{
				objectDetector.LoadSVM(FILE_SVM);
			}
			
			else
			{
				objectDetector.Train();
				objectDetector.SaveDictionary(FILE_DICTIONARY);
				objectDetector.SaveSVM(FILE_SVM);
			}*/
            cv::Mat frame(1, 1, CV_32FC1);
            char objectName[100];

        	int state = WALKING;
        	float sonarReadings[3] = { -1.0, -1.0, -1.0 };
        	float motionSpeed = 60.0;
        	float leftSpeed = 0.0;
        	float rightSpeed = 0.0;
        	float timeStamp;
        	float turningDuration;

        	float PROXIMITY_THRESHOLD = 0.15;
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {   
		    	//Atualiza sonares
		    	UpdateSonarReadings(sonarReadings); 

		    	//Máquina de estados
		    	if(state == WALKING)
		    	{
		    		leftSpeed = motionSpeed;
		    		rightSpeed = motionSpeed;

		    		if(sonarReadings[LEFT] <= PROXIMITY_THRESHOLD
		    		|| sonarReadings[FRONT] <= PROXIMITY_THRESHOLD
		    		|| sonarReadings[RIGHT] <= PROXIMITY_THRESHOLD	)
		    		{
		    			state = STOPPING;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}
		    	}

		    	else if(state == STOPPING)
		    	{
		    		leftSpeed = 0.0;
		    		rightSpeed = 0.0;

		    		if(APIGetSimulationTimeInSecs() - timeStamp >= 0.2)
		    			state = DETECTING;
		    	}

		    	else if(state == DETECTING)
		    	{
		    		printf("\n\rDetecting object...\n");
                	frame = APIReadCamera();
					APIWaitMsecs(2000);
					//objectDetector.Detect(frame, objectName);
					printf("\rObject: %s\n\n", objectName);

					turningDuration = RandomTimeSecs(1.1);
		    		timeStamp = APIGetSimulationTimeInSecs();
		    		state = TURNING;
		    	}

		    	else if(state == TURNING)
		    	{
		    		leftSpeed = motionSpeed;
		    		rightSpeed = -motionSpeed;

		    		if(APIGetSimulationTimeInSecs() - timeStamp >= turningDuration)
		    		{
		    			if(sonarReadings[LEFT] > PROXIMITY_THRESHOLD
		    			|| sonarReadings[FRONT] > PROXIMITY_THRESHOLD
		    			|| sonarReadings[RIGHT] > PROXIMITY_THRESHOLD)
		    				state = WALKING;
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