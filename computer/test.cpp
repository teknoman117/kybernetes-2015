#include <iostream>
#include <thread>
#include <chrono>

#include "kybernetes.hpp"
#include "garmingps.hpp"

using namespace std;
using namespace kybernetes;
using namespace kybernetes::sensor;
using namespace kybernetes::constants;

class Application;
static Application *instance;

class Application
{
    GarminGPS *gps;

public:
    Application()
    {
        gps = new GarminGPS(GPSPath, dispatch_get_main_queue());
        gps->SetHandler([] (const GarminGPS::State& state)
        {
            cout << "Received GPS Packet @ " << asctime(localtime((time_t *) &state.timestamp)) << endl;
        });
    }

    static void run()
    {
        dispatch_async(dispatch_get_main_queue(), ^
        {
            instance = new Application();
        });
        dispatch_main();
    }
};

int main (int argc, char** argv)
{
    Application::run();
    return 0;
}
