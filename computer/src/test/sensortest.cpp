#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

#include <kybernetes/constants/constants.hpp>
#include <kybernetes/utility/application.hpp>
#include <kybernetes/controller/sensor_controller.hpp>

using namespace std;

using namespace kybernetes::controller;
using namespace kybernetes::constants;
using namespace kybernetes::utility;

class SensorTestApplication : public Application::Delegate
{
    SensorController *sensorController;

public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {
        // Open the sensor controller
        sensorController = new SensorController(SensorControllerPath, dispatch_get_main_queue());

        // Parse command line options (for sensors to monitor)
        char option;
        bool monitorSonars = false, monitorIMU = false, monitorBumpers = false;
        while ((option = getopt (argc, argv, "sib")) != -1)
        switch (option)
        {
            case 's':
                monitorSonars = true;
                break;
            case 'i':
                monitorIMU = true;
                break;
            case 'b':
                monitorBumpers = true;
                break;
            case '?':
                 if (isprint (optopt))
                    cerr << "Unknown option \'-" << optopt << "\'." << endl;
                else
                    cerr << "Unknown option character \'\\x" << hex << optopt << "\'." << endl;
                return;
            default:
                abort ();
        };

        // Register handlers
        if(monitorSonars)
        {
            sensorController->SetSonarHandler([] (SensorController::SonarState& state)
            {
                // Print it out
                cout << "Sonars ==> " << state[0] << " cm, " << state[1] << " cm, " << state[2] << " cm" << endl;
            });
        }

        // Register handlers
        if(monitorIMU)
        {
            sensorController->SetIMUHandler([] (SensorController::IMUState& state)
            {
                // Print it out
                cout << "IMU ==> " << state[0] << " degrees, " << state[1] << " degrees, " << state[2] << " degrees" << endl;
            });
        }

        // Register handlers
        if(monitorBumpers)
        {
            sensorController->SetBumperHandler([] (SensorController::BumperState& state)
            {
                // Print it out
                cout << "Bumpers ==> " << state[0] << ", " << state[1] << endl;
            });
        }
    }
    void ApplicationWillTerminate()
    {
        cout << "Termination Requested" << endl;
        delete sensorController;
    }
};

int main (int argc, char** argv)
{
    Application::Run<SensorTestApplication>(argc, argv);
    return 0;
}
