//
//  test.cpp
//  control flight sample
//
//  Created by jianhui Li on 2018-06-27.
//  Copyright © 2018 jianhui Li. All rights reserved.
//



#include <djiosdk/dji_telemetry.hpp>
#include <djiosdk/dji_vehicle.hpp>
#include <iostream>
#include <djiosdk/dji_broadcast.hpp>



using namespace DJI::OSDK;

using namespace DJI::OSDK::Telemetry;



int main()
{

GlobalPosition currentPosition;


Vehicle drone("/dev/ttyAMA0", 115200, true, false);


DataBroadcast* tmp = drone.broadcast;


currentPosition = tmp->getGlobalPosition();



//currentposition = drone->broadcast->getGlobalPosition();





cout << currentPosition <<endl;

return 0;

}