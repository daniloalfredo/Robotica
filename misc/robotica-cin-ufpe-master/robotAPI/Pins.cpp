#include "Pins.h"
#include <sstream>
#include <stdlib.h>
#include <iostream>

using namespace std;

int sysconfgGPIOEdge(int pinBCM, std::string mode) {
    cout << "Configuring pin " << pinBCM << " in mode '" << mode << "'...";
    std::stringstream ss;
    ss << "gpio edge " << pinBCM << " " << mode;
    if (system(ss.str().c_str())!=0)
        return -1;

    cout << "...ok" << endl;
    return 0;
}
