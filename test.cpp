#include "garmingps.hpp"

#include <iostream>
#include <unistd.h>
#include <ctime>

using namespace kybernetes;

int main ()
{
    // Get the main dispatch queeu
    kybernetes::GarminGPS gps("/dev/ttyUSB0");
    gps.RegisterHandler([] (GarminGPS::State& state)
    {
        // Print out the location
        std::cout << "location = " << state.latitude << "," << state.longitude << ";" << state.altitude << "m @ " << asctime(localtime((time_t*)&state.timestamp));
    });
    gps.Join();

    return 0;
}
