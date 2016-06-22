#include "robotAPI/RobotTimer.h"
#include <unistd.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv) {

	for (int i=0; i<20; i++) {
	   long start = RobotTimer::getTime_us();
		//RobotTimer::delay_us(1000000);
		usleep(1000000);
		cout << "Time: " <<	RobotTimer::getTime_us() - start << "us" << endl;	
	}
}
