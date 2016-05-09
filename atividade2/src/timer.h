#ifndef TIMER_H_INCLUDED
#define TIMER_H_INCLUDED

#include <sys/time.h>
#include <ctime>
#include <cmath>

class Timer
{
	private:
		struct timeval begin, end;
		long int secs;

	public:

		void Start();
		int GetTimeMsec();
		int GetTimeSec();
};

#endif
