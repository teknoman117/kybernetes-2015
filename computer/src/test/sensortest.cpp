#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <memory>

#include <kybernetes/constants/constants.hpp>
#include <kybernetes/utility/application.hpp>
#include <kybernetes/controller/sensor_controller.hpp>

using namespace std;

using namespace kybernetes::controller;
using namespace kybernetes::constants;
using namespace kybernetes::utility;

class SensorTestApplication : public Application::Delegate
{
    unique_ptr<SensorController> sensorController;

public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {
        // Parse command line options (for sensors to monitor)
        char option;
        short cutoff = 400;
        bool monitorSonars = false, monitorIMU = false, monitorBumpers = false;
        string path = SensorControllerPath;
        while ((option = getopt (argc, argv, "p:sibc:")) != 255)
        switch (option)
        {
            case 'p':
                path = optarg;
                break;
            case 's':
                monitorSonars = true;
                break;
            case 'i':
                monitorIMU = true;
                break;
            case 'b':
                monitorBumpers = true;
                break;
            case 'c':
                cutoff = atoi(optarg);
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

        // Open the sensor controller
        sensorController = make_unique<SensorController>(path, dispatch_get_main_queue());

        // Register handlers
        if(monitorSonars)
        {
            sensorController->SetSonarHandler([cutoff] (SensorController::SonarState& state)
            {
                // Print it out
                cout << "Sonars ==> ";
                if(state[0] > 0 && state[0] < cutoff)
                    cout << state[0] << " cm, ";
                else
                    cout << "clear, ";

                if(state[1] > 0 && state[1] < cutoff)
                    cout << state[1] << " cm, ";
                else
                    cout << "clear, ";
    
                if(state[2] > 0 && state[2] < cutoff)
                    cout << state[2] << " cm";
                else
                    cout << "clear ";
 
                cout << endl;
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
    }
};

int main (int argc, char** argv)
{
    Application::Run<SensorTestApplication>(argc, argv);
    return 0;
}
