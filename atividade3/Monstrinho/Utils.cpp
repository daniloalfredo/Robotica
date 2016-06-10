#include "Utils.h"

float fsignal(float v)
{
	if(v >= 0)
		return 1.0;
	return -1.0;
}

float rand_beetween_0_and_1()
{
	static bool seeded = false;
	
	if(!seeded)
		srand(time(NULL));
		
	return (rand()%1001) / 1000.0;
}

float rand_signal()
{
	static bool seeded = false;
	
	if(!seeded)
		srand(time(NULL));
		
	if(rand()%2 == 0)
		return 1.0;
	return -1.0;
}

float to_pi_range(float radians)
{
    radians = fmod(radians, 2*PI);
    if (radians < -PI)
        radians += 2*PI;
    else if (radians > PI)
        radians -= 2*PI;
 
    return radians;
}

float to_2pi_range(float radians)
{
    while(radians < 0.0)
		radians += 2*PI;
	
	if(radians > 2*PI)
		radians = fmod(radians, 2*PI);

    return radians;
}

float to_rad(float degrees)
{
	if(degrees < 0)
		degrees = 360 - degrees;
		
	return (degrees * (PI/180.0));
}

float to_deg(float radians)
{
    return radians * (180.0 / M_PI);
}

float to_pos_deg(float radians)
{
	while(radians < 0.0)
		radians += 2*PI;
	
	if(radians > 2*PI)
		radians = fmod(radians, 2*PI);
	
    return to_2pi_range(radians) * (180.0 / M_PI);
}
 
float smallestAngleDiff(float target, float source)
{
    float a;
    a = to_2pi_range(target) - to_2pi_range(source);
 
    if (a > M_PI)
        a = a - 2 * M_PI;
    else if (a < -M_PI)
        a = a + 2 * M_PI;
    
    return a;
}

float GetSimulationTimeInSecs(simxInt clientID)
{
	return (((float) simxGetLastCmdTime(clientID)) / 1000.0);
}

float GetTimeSinceLastCommandInSecs(simxInt clientID, float lastCommandTime)
{
	return GetSimulationTimeInSecs(clientID) - lastCommandTime;
}

double normalDistribution(double x)
{
	if(x < -10.0)return 0.0;
	if(x > 10.0)return 1.0;
	// number of steps
	int N=2000;
	// range of integration
	double a=0,b=x;
	// local variables
	double s,h,sum=0.;
	// inialise the variables
	h=(b-a)/N;
	// add in the first few terms
	sum = sum + exp(-a*a/2.) + 4.*exp(-(a+h)*(a+h)/2.);
	// and the last one
	sum = sum + exp(-b*b/2.);
	// loop over terms 2 up to N-1
	for(int i=1;i<N/2;i++)
	{
		s = a + 2*i*h;
		sum = sum + 2.*exp(-s*s/2.);
		s = s + h;
		sum = sum + 4.*exp(-s*s/2.);
	}
	// complete the integral
	sum = 0.5 + h*sum/3./sqrt(8.*atan(1.));
	// return result
	return sum;
}
