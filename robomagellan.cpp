#include <iostream>
#include <unistd.h>
#include <ctime>
#include <csignal>

#include "kybernetes.hpp"
#include "utility.hpp"
#include "garmingps.hpp"
#include "sensor_controller.hpp"

using namespace kybernetes;
using namespace std;

class Application
{
    GarminGPS        *gps;
    SensorController *sensorController;

    // Called when the GPS has been updated
    void GarminGPSHandler (GarminGPS::State& state)
    {
        // Print out the location
        //cout << "location = " << state.latitude << "," << state.longitude << ";" << state.altitude << "m @ " << asctime(localtime((time_t*)&state.timestamp));
    }

    // Called when the Sensor controller has posted a packet
    void SensorControllerHandler (SensorController::State& state)
    {
        //std::cout << "Sonars = " << state.sonar[0] << ", " << state.sonar[1] << ", " << state.sonar[2] << std::endl;
        //std::cout << "Bumpers = " << state.bumper[0] << ", " << state.bumper[1] << std::endl;
        //std::cout << "IMU = " << state.rotation[0] << ", " << state.rotation[1] << ", " << state.rotation[2] << std::endl;
    }

public:
    Application()
    {
        // Initialize the GPS
        auto gpsCallback = bind(&Application::GarminGPSHandler, this, std::placeholders::_1);
        gps = new GarminGPS(GPSPath);
        gps->RegisterHandler(gpsCallback);

        // Initialize the Sensor controller
        auto sensorCallback = bind(&Application::SensorControllerHandler, this, std::placeholders::_1);
        sensorController = new SensorController(SensorControllerPath);
        sensorController->RegisterHandler(sensorCallback);
    }

    void Join()
    {
        gps->Join();
        sensorController->Join();
    }

    void Close()
    {
        // TODO - do stuff to shutdown robot

        gps->Close();
        sensorController->Close();
    }
};

static Application *application;

void killHandler(int signal)
{
    // Just close
    cout << "---- Termination Requested ----" << endl;
    application->Close();
}

int main (int argc, char **argv)
{
    // Process any input path information
    signal(SIGINT, killHandler);

    application = new Application();
    application->Join();
    return 0;
}
