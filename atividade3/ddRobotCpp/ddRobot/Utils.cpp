#include "Utils.h"

float fsignal(float v)
{
	if(v >= 0)
		return 1.0;
	return -1.0;
}

float to180range(float angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle < -M_PI)
        angle = angle + 2 * M_PI;
    else if (angle > M_PI)
        angle = angle - 2 * M_PI;
 
    return angle;
}
 
float to_positive_angle(float angle)
{
    angle = fmod(angle, 2 * M_PI);
    while (angle < 0) {
        angle = angle + 2 * M_PI;
    }
    return angle;
}

float to_2pi_range(float angle)
{
    angle = to_positive_angle(angle);

    while(angle > 2*M_PI)
        angle -= 2*M_PI;

    return angle;
}
 
float smallestAngleDiff(float target, float source)
{
    float a;
    a = to_positive_angle(target) - to_positive_angle(source);
 
    if (a > M_PI) {
        a = a - 2 * M_PI;
    } else if (a < -M_PI) {
        a = a + 2 * M_PI;
    }
    return a;
}

double to_deg(double radians)
{
    return radians * (180.0 / M_PI);
}

simxInt getSimTimeMs(int clientID) //In Miliseconds
{
    return simxGetLastCmdTime(clientID);
}
