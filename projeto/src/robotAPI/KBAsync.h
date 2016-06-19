#ifndef KBAsync_H
#define KBAsync_H

#include <termios.h>

class KBAsync {
public:

	KBAsync();
	~KBAsync();
	int kbhit();
	int getch();
	int getKey();
};

#endif
