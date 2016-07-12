#include "ColorDetector.h"
#include "RobotAPI.h"
#include "Utils.h"

#define INFINITE_DISTANCE 2.2

enum STATES { WALKING = 1, STOPPING = 2, DETECTING = 3, TURNING_A = 4, TURNING_B = 5, EMERGENCY_REAR = 6 };
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
            cv::Mat frame(1, 1, CV_32FC1);
            std::string objectName;

        	int state = WALKING;
        	float sonarReadings[3] = { -1.0, -1.0, -1.0 };
        	float leftSpeed = 0.0;
        	float rightSpeed = 0.0;
        	float deltaS = 0.0;
        	float stampDeltaSRear = 0.0;

        	float baseSpeed = 35.0;  

        	float WHEEL_R = 0.0375;
        	float WALL_THRESHOLD_FAR = 0.07;
        	float WALL_THRESHOLD_CLOSE = 0.05;
        	float WALL_THRESHOLD_FRONT = 0.08;
        	float DELTA_S_THRESHOLD = 0.025;

        	int substate = 0;
        	float timeStamp = APIGetSimulationTimeInSecs();
        	
        	//---------------------------------------------------------
        	//LOOP DA SIMULAÇÃO
        	//---------------------------------------------------------
		    while(APISimulationIsRunning())
		    {   
		    	//Atualiza sonares
		    	UpdateSonarReadings(sonarReadings); 

		    	//Atualiza odometria
		    	deltaS += APIReadOdometers(WHEEL_R);

		    	//Máquina de estados
		    	if(state == WALKING)
		    	{
		    		leftSpeed = baseSpeed;
		    		rightSpeed = baseSpeed;

		    		//Odometria para checar se bateu
		    		if(APIGetSimulationTimeInSecs() - timeStamp < 2.0)
		    		{
		    			stampDeltaSRear = deltaS - stampDeltaSRear;
		    		}
		    		else
		    		{
		    			if(stampDeltaSRear < DELTA_S_THRESHOLD){
		    				substate = 0;
		    				state = EMERGENCY_REAR;
		    			}

		    			stampDeltaSRear = deltaS;	
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}

		    		if(sonarReadings[RIGHT] > WALL_THRESHOLD_FAR
		    		&& sonarReadings[RIGHT] < WALL_THRESHOLD_FAR + 0.04)
		    		{
		    			leftSpeed = baseSpeed;
		    			rightSpeed = baseSpeed*0.60;
		    		}

		    		if(sonarReadings[RIGHT] < WALL_THRESHOLD_CLOSE
		    		&& sonarReadings[RIGHT] > WALL_THRESHOLD_CLOSE - 0.04)
		    		{
		    			leftSpeed = baseSpeed*0.60;
		    			rightSpeed = baseSpeed;
		    		}

		    		//Hora de tirar foto
		    		if(sonarReadings[FRONT] >= 0.18 && sonarReadings[FRONT] <= 0.20)
		    		{
		    			state = DETECTING;
		    			substate = 0.0;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}

		    		//Curva tipo A
		    		if(sonarReadings[FRONT] <= 0.10)
		    		{
		    			state = TURNING_A;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}

		    		//Curva tipo B
		    		if(sonarReadings[FRONT] > 0.30 && sonarReadings[RIGHT] > 0.30
		    		&& sonarReadings[FRONT] < 2.20 && sonarReadings[RIGHT] < 2.20)
		    		{
		    			state = TURNING_B;
		    			substate = 0;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}
		    	}

		    	else if(state == DETECTING)
		    	{
		    		//STOPPING
		    		if(substate == 0)
		    		{
			    		leftSpeed = 0.0;
			    		rightSpeed = 0.0;

			    		if(APIGetSimulationTimeInSecs() - timeStamp >= 0.5)
			    		{
			    			state = DETECTING;
			    			substate = 1;
			    		}
					}

					//DETECTING
					else if(substate == 1)
					{
						printf("\n\rDetecting object...\n");
	                	frame = APIReadCamera();
						objectName = objectDetector.Detect(frame);
						printf("\rObject: %s\n\n", objectName.c_str());
						APISavePicture(frame, objectName);
						APIWaitMsecs(1000);
						timeStamp = APIGetSimulationTimeInSecs();
						substate = 2;
					}

					//GO FOWARD
					else if(substate == 2)
		    		{
			    		leftSpeed = baseSpeed;
			    		rightSpeed = baseSpeed;

			    		if(APIGetSimulationTimeInSecs() - timeStamp >= 1.0)
			    		{
			    			state = WALKING;
			    			stampDeltaSRear = deltaS;
							timeStamp = APIGetSimulationTimeInSecs();
			    			substate = 0;
			    		}
					}
		    	}

		    	else if(state == TURNING_A)
		    	{
		    		leftSpeed = -baseSpeed*0.60;
		    		rightSpeed = baseSpeed;

		    		if(APIGetSimulationTimeInSecs() - timeStamp > 2.0
		    		/*&& sonarReadings[FRONT] >= 0.20
		    		&& sonarReadings[RIGHT] <= 0.10*/)
		    		{
		    			state = WALKING;
		    			stampDeltaSRear = deltaS;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}
		    	}

		    	else if(state == TURNING_B)
		    	{
		    		leftSpeed = 70.0;
		    		rightSpeed = 35.0;

		    		if(APIGetSimulationTimeInSecs() - timeStamp > 1.7)
		    		{
		    			substate = 1;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    			state = WALKING;
		    			stampDeltaSRear = deltaS;
		    			timeStamp = APIGetSimulationTimeInSecs();
		    		}
		    	}

		    	else if(state == EMERGENCY_REAR)
		    	{
		    		//Ré
		    		if(substate == 0)
		    		{
		    			leftSpeed = -60.0;
			    		rightSpeed = -60.0;

			    		if(APIGetSimulationTimeInSecs() - timeStamp > 0.7)
			    		{
			    			substate = 1;
			    			timeStamp = APIGetSimulationTimeInSecs();
			    		}
		    		}

		    		//Correção
		    		else if(substate == 1)
		    		{
		    			leftSpeed = 30.0;
			    		rightSpeed = 55.0;

			    		if(APIGetSimulationTimeInSecs() - timeStamp > 0.7)
			    		{
			    			stampDeltaSRear = deltaS;
			    			state = WALKING;
			    			timeStamp = APIGetSimulationTimeInSecs();
			    		}
		    		}
		    	}

		    	//APISetRobotSpeed(leftSpeed, rightSpeed);
		    	APISetMotorPower(leftSpeed, rightSpeed);

		    	//Printa Informações
		    	printf("\rState: %d  |  Substate: %d  |  DeltaS: %f  |  Sonars:[%.2f, %.2f, %.2f]\n", state, substate, deltaS, sonarReadings[0], sonarReadings[1], sonarReadings[2]);

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