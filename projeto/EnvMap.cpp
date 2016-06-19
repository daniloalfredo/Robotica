#include "EnvMap.h"

EnvMap::EnvMap()
{	
}

void EnvMap::PrintMap()
{
	printf("Mapa do Ambiente:\n");
	for(int i = 0; i < (int) segments.size(); i++)
		printf("\tSegmento de (%.2f, %.2f) para (%.2f, %.2f)\n", segments[i].first.first, segments[i].first.second, segments[i].second.first, segments[i].second.second);
}

void EnvMap::AddWall(float x0, float y0, float x1, float y1)
{
	Point a(x0, y0);
	Point b(x1, y1);
	Segment s(a, b);
	segments.push_back(s);
}

float EnvMap::MapDistance(float x, float y, float theta)
{	
	//Verifica quais segmentos o vetor intersecta e guarda
	float min_dist = INFINITE_DISTANCE;
	for(int i = 0; i < (int) segments.size(); i++)
	{
		float dist = IntersectionPointToSegment(x, y, theta, segments[i]);
		
		if(dist < min_dist)
			min_dist = dist;
	}
	
	return min_dist;
}

float EnvMap::MapDistance2(float x, float y, float theta)
{
	static float sensorOpening = 15.0*(PI/180.0); 
	return fmin(MapDistance(x, y, theta-sensorOpening), MapDistance(x, y, theta+sensorOpening));
}

//--------------------------------------------------------------
//Funções Auxiliares
//--------------------------------------------------------------

//Retorna a distância do vetor ao segmento de reta (se não houver intersecção retorna distancia infinita)
float EnvMap::IntersectionPointToSegment(float x, float y, float theta, Segment seg)
{
	float angle = to_2pi_range(theta);

	//Encontrando a reta do vetor
	float a1 = tan(angle); //Possível crash em 90 graus (improvável)
	float b1 = y - a1*x;
	
	float increment = 10.0*fsignal(a1);
	
	if(angle > PI)
		increment *= -1;
	
	float p0_x = x;
	float p0_y = y;
	float p1_x = x + increment;
	float p1_y = a1*p1_x + b1;

	float p2_x = seg.first.first;
	float p2_y = seg.first.second;
	float p3_x = seg.second.first+0.001;
	float p3_y = seg.second.second;
	
	//-------------------------------------------
	//Colisão entre 2 segmentos de reta (p0, p1) e (p2, p3)

	float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
    	float xp = p0_x + (t * s1_x);
        float yp = p0_y + (t * s1_y);
    
    	return sqrt((x-xp)*(x-xp) + (y-yp)*(y-yp));
    }
        
    return INFINITE_DISTANCE;
}
