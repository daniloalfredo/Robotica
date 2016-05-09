#ifndef TEMPO_H_INCLUDED
#define TEMPO_H_INCLUDED

#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#endif

#include <cmath>

class Timer
{
	private:

		#ifdef _WIN32
		LARGE_INTEGER frequencia, inicio, final;
		#else
		struct timeval inicio, final;
		#endif

	public:

		void Start();
		int GetTimeMsec();
};

#endif
