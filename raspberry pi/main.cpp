#include <djiosdk/dji_vehicle.hpp>
#include <iostream>
#include <djiosdk/dji_control.hpp>


using namespace std;
using namespace DJI::OSDK;

int main() {

    Vehicle v("/dev/ttyAMA0", 115200, true, false);

    ACK::DroneVersion version = v.getDroneVersion(1);

    cout << version.data.hwVersion << endl;
    return 0;
}