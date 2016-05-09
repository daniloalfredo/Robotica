#include "timer.h"

void Timer::Start()
{
	#ifdef _WIN32
	QueryPerformanceFrequency(&frequencia);
    QueryPerformanceCounter(&inicio);
	#else
	gettimeofday(&inicio, 0);
	#endif
}

int Timer::GetTimeMsec()
{
	int tempo;
	
	#ifdef _WIN32
    QueryPerformanceCounter(&final);
    #else
	gettimeofday(&final, 0);
	#endif

	#ifdef _WIN32
    tempo = (int) fabs(((1000LL * final.QuadPart) - (1000LL * inicio.QuadPart)) / frequencia.QuadPart);
	#else
	tempo = (int) ( 1000 * (final.tv_sec - inicio.tv_sec) + (final.tv_usec - inicio.tv_usec) / 1000);
	#endif

	return tempo;
}
