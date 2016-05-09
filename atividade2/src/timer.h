#ifndef TIMER_H_INCLUDED
#define TIMER_H_INCLUDED

#include <sys/time.h>
#include <cmath>

class Timer
{
	private:
		struct timeval inicio, final;

	public:

		void Start();
		int GetTimeMsec();
};

#endif
