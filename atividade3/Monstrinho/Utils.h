#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <cstdlib>
#include <ctime>
#include <cmath>
#include <unistd.h>
 
extern "C" {
#include "extApi.h"
/* #include "extApiCustom.h" if you wanna use custom remote API functions! */
}

#define PI M_PI

//Obter sinal do número
float fsignal(float v);

//Números aleatórios
float rand_beetween_0_and_1();
float rand_signal();

//Funções de Ângulos
float to_pi_range(float radians);
float to_2pi_range(float radians);
float to_rad(float degrees);
float to_deg(float radians);
float to_pos_deg(float radians);
float smallestAngleDiff(float target, float source);

//Funções de Tempo
float GetSimulationTimeInSecs(simxInt clientID);
float GetTimeSinceLastCommandInSecs(simxInt clientID, float lastCommandTime);

//Distribuição de probabilidade
double normalDistribution(double x);

#endif
