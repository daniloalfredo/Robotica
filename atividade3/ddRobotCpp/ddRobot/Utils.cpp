#include "Utils.h"

float to180range(float angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle<-M_PI)
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
