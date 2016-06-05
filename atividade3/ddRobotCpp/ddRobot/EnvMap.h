#ifndef ENVMAP_H_INCLUDED
#define ENVMAP_H_INCLUDED

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>

#include "Utils.h"

typedef std::pair<float, float> Point;
typedef std::pair< Point, Point > Segment;

class EnvMap
{
	private:
		std::vector< Segment > segments;
	
		//Funções Auxiliares
		float IntersectionPointToSegment(float x, float y, float theta, Segment seg);
		
	public:
		EnvMap();
		void PrintMap();
		void AddWall(float x0, float y0, float x1, float y1);
		float MapDistance(float x, float y, float theta);
};

#endif
