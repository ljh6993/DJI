#include <wiringPi.h>

// LED Pin - wiringPi pin 0 is BCM_GPIO 17.
// we have to use BCM numbering when initializing with wiringPiSetupSys
// when choosing a different pin number please use the BCM numbering, also
// update the Property Pages - Build Events - Remote Post-Build Event command 
// which uses gpio export for setup for wiringPiSetupSys

#include <iostream> 
#include <unistd.h>
int main(void)
{
	//wiringPiSetupSys();

	//pinMode(LED, OUTPUT);

	while (true)
	{
		//digitalWrite(LED, HIGH);  // On
		sleep(100);
		std::cout << "time0" << std::endl;
		//digitalWrite(LED, LOW);	  // Off
		sleep(100);
		std::cout << "time1" << std::endl;
	}
	return 0;
}