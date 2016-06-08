#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <math.h>
#include <unistd.h>
#include <sys/time.h>
 
extern "C" {
#include "extApi.h"
/* #include "extApiCustom.h" if you wanna use custom remote API functions! */
}

#define PI M_PI

//Funções de Ângulos
float fsignal(float v);
float to180range(float angle);
float to_2pi_range(float angle);
float to_positive_angle(float angle);
float smallestAngleDiff(float target, float source);
double to_deg(double radians);
double to_rad(double degrees);

//Funções de Tempo
float GetSimulationTimeInSecs(simxInt clientID);
float GetTimeSinceLastCommandInSecs(simxInt clientID, float lastCommandTime);

#endif
