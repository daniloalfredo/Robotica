#include "timer.h"

void Timer::Start()
{
	gettimeofday(&begin, NULL);
	secs = time(NULL);
}

int Timer::GetTimeMsec()
{
	int time_diff;
	
	gettimeofday(&end, NULL);
	time_diff = (int) (end.tv_sec - begin.tv_sec)*1000 + (end.tv_sec - begin.tv_usec)/1000; 

	return time_diff;
}

int Timer::GetTimeSec()
{
	return (int)time(NULL)-secs;
}
