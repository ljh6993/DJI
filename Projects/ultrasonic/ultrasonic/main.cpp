#include <iostream>
#include <wiringPi.h>
#include "libSonar.h"

using namespace std;

int trigger = 2;
int echo = 3;

int main()
{
	if (wiringPiSetupGpio() == -1)
		return -1;

	Sonar sonar;
	sonar.init(trigger, echo);
	while (1) {
		cout << "Distance is " << sonar.distance(30000) << " cm." << endl; // 30000 is a timeout in microseconds
	}
}