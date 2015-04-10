#include <iostream>
#include <thread>
#include <chrono>

#include "kybernetes.hpp"
#include "application.hpp"
#include "garmingps.hpp"

using namespace std;
using namespace kybernetes;
using namespace kybernetes::sensor;
using namespace kybernetes::constants;

class TestApplication : public Application::Delegate
{
    GarminGPS *gps;

public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {
        gps = new GarminGPS(GPSPath, dispatch_get_main_queue());
        gps->SetHandler([] (const GarminGPS::State& state)
        {
            cout << "Received GPS Packet @ " << asctime(localtime((time_t *) &state.timestamp)) << endl;
        });
    }
    void ApplicationWillTerminate()
    {
        delete gps;
    }
};

int main (int argc, char** argv)
{
    Application::Run<TestApplication>(argc, argv);
    return 0;
}
