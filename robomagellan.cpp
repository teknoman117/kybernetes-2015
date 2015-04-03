#include <iostream>
#include <unistd.h>
#include <ctime>

#include "kybernetes.hpp"
#include "garmingps.hpp"

using namespace kybernetes;
using namespace std;

class Application
{
    GarminGPS *gps;

    // Called when the GPS has been updated
    void GarminGPSHandler (GarminGPS::State& state)
    {
        // Print out the location
        cout << "location = " << state.latitude << "," << state.longitude << ";" << state.altitude << "m @ " << asctime(localtime((time_t*)&state.timestamp));
    }

public:
    Application()
    {
        // Initialize the GPS
        auto gpsCallback = bind(&Application::GarminGPSHandler, this, std::placeholders::_1);
        gps = new GarminGPS(GPSPath);
        gps->RegisterHandler(gpsCallback);
    }

    void Join()
    {
        gps->Join();
    }
};

int main (int argc, char **argv)
{
    // Process any input path information


    Application app;
    app.Join();
    return 0;
}
