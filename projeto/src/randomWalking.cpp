#include "ColorDetector.h"
#include "RobotAPI.h"
#include "Utils.h"

#define INFINITE_DISTANCE 2.2

enum STATES { WALKING, STOPPING, DETECTING, TURNING, EMERGENCY_REAR };
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

        	//Inicializa detector de objetos
        	ColorDetector objectDetector;

            std::string objectName;

        	int state = WALKING;
        	float sonarReadings[3] = { -1.0, -1.0, -1.0 };
        	float motionSpeed = 60.0;
        	float leftSpeed = 0.0;
        	float rightSpeed = 0.0;
        	float timeStamp = APIGetSimulationTimeInSecs();
        	float turningDuration;
        	float turingDirection = 1.0;
        	float deltaS = 0.0;

        	float WHEEL_R = 0.0375;
        	float PROXIMITY_THRESHOLD = 0.15;
        	float DELTA_S_THRESHOLD = 0.12;
        	int pic_counter = 0;

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

		    		//Odometria para checar se bateu
		    		if(APIGetSimulationTimeInSecs() - timeStamp < 3.0)
		    			deltaS += APIReadOdometers(WHEEL_R);
		    		else
		    		{
		    			if(deltaS < DELTA_S_THRESHOLD)
		    				state = EMERGENCY_REAR;

		    			deltaS = 0.0;	
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}

		    		if(sonarReadings[LEFT] <= PROXIMITY_THRESHOLD
		    		|| sonarReadings[FRONT] <= PROXIMITY_THRESHOLD
		    		|| sonarReadings[RIGHT] <= PROXIMITY_THRESHOLD	)
		    		{
		    			deltaS = 0.0;
		    			state = STOPPING;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}
		    	}

		    	else if(state == EMERGENCY_REAR)
		    	{
		    		leftSpeed = -motionSpeed;
		    		rightSpeed = -motionSpeed;

		    		if(APIGetSimulationTimeInSecs() - timeStamp > 0.5)
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
	                
	                cv::Mat frame;
	                for(int i = 0; i < 15; i++)
	                {
	                 	frame = APIReadCamera();
	                 	APIWaitMsecs(10);
	                }

	                objectName = objectDetector.Detect(frame);
   					char pic_path[50];
    				sprintf(pic_path, "img/pics/%d_%s.jpg", pic_counter, objectName.c_str());
    				pic_counter++;
					printf("\rObject: %s\n\n", objectName.c_str());
					APISavePicture(frame, pic_path);
					APIWaitMsecs(800);

					turningDuration = RandomValue(0.4, 1.2);
		    		timeStamp = APIGetSimulationTimeInSecs();
		    		state = TURNING;
		    		turingDirection = RandSignal();
		    	}

		    	else if(state == TURNING)
		    	{
		    		leftSpeed = motionSpeed*turingDirection;
		    		rightSpeed = -motionSpeed*turingDirection;

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
		    	printf("\rdeltaS: %f | Sonars:[%.2f, %.2f, %.2f]\n", deltaS, sonarReadings[0], sonarReadings[1], sonarReadings[2]);

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