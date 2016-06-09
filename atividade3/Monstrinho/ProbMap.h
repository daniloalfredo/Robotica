#ifndef PROBMAP_H_INCLUDED
#define PROBMAP_H_INCLUDED

#include <cstdio>
#include <cmath>
#include <cstdlib>

#include "Utils.h"

#define X_DELTA 
#define Y_DELTA
#define DEG_DELTA 1

#define MAP_SIZE_X 
#define MAP_SIZE_Y
#define MAP_SIZE_DEG 360

#define CELLS_X MAP_SIZE_X/X_DELTA
#define CELLS_Y MAP_SIZE_Y/Y_DELTA
#define CELLS_DEG MAP_SIZE_DEG/DEG_DELTA

//Mapa de probabilidades
//float Map[CELLS_X][CELLS_Y][CELLS_DEG];
//float covar[3][3];

class ProbMap
{
	private:
		float Map[CELLS_X][CELLS_Y][CELLS_DEG];
		float covar[3][3];

	public:
		ProbMap();
		void UpdateMap();
		void ActionUpdate(int x, int y, int deg);
		void PerceptionUpdate(int x, int y, int deg);
		void ErrorPropagationOdometry(simxFloat dPhiL, simxFloat dPhiR); //calcula as variâncias em x e y para montar a Gaussiana 2-D
}

#endif
