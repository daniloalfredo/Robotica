#include "timer.h"

void Timer::Start()
{
	gettimeofday(&inicio, 0);
}

int Timer::GetTimeMsec()
{
	int tempo;
	
	gettimeofday(&final, 0);
	tempo = (int) ( 1000 * (final.tv_sec - inicio.tv_sec) + (final.tv_usec - inicio.tv_usec) / 1000);

	return tempo;
}
