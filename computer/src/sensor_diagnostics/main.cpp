#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <memory>

#include <kybernetes/constants/constants.hpp>
#include <kybernetes/utility/application.hpp>
#include <kybernetes/controller/sensor_controller.hpp>
#include <kybernetes/sensor/garmingps.hpp>

using namespace std;

using namespace kybernetes::controller;
using namespace kybernetes::constants;
using namespace kybernetes::utility;
using namespace kybernetes::sensor;

class SensorTestApplication : public Application::Delegate
{
    unique_ptr<SensorController> sensorController;
    unique_ptr<GarminGPS>        gps;

public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {
        // Parse command line options (for sensors to monitor)
        char option;
        short cutoff = 400;
        bool monitorSonars = false, monitorIMU = false, monitorBumpers = false, monitorGPS = false;
        string path = SensorControllerPath;
        while ((option = getopt (argc, argv, "p:sigbc:")) != 255)
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
            case 'g':
                monitorGPS = true;
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
        cout << "Attempting to open the sensor controller.  Please allow a few moments for the IMU Gyro Bias to stabilize" << endl;
        sensorController = make_unique<SensorController>(path, dispatch_get_main_queue(), 57600, [this, monitorBumpers, monitorSonars, monitorIMU, cutoff] (bool success)
        {
            if(!success)
            {
                cout << "An error occurred in opening the sensor controller." << endl;
                Application::Instance()->Exit();
                return;
            }

            // Register handlers
            cout << "Sensor controller opened successfully" << endl;
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
        });

        // Open the GPS
        gps = std::make_unique<GarminGPS>(GPSPath, dispatch_get_main_queue(), 9600, [monitorGPS] (GarminGPS *gps, bool success)
        {
            // An error occurred
            if(!success)
            {
                cout << "An error occurred opening the GPS." << endl << endl;
                Application::Instance()->Exit();

                return;
            }

            // Register the gps handler
            if(monitorGPS)
            {
                gps->SetHandler([] (const GarminGPS::State& state)
                {
                    cout << "Received GPS Packet @ " << asctime(localtime((time_t *) &state.timestamp));
                    cout << setprecision(10) << "    Latitude  = " << state.latitude << " degrees" << endl;
                    cout << setprecision(10) <<  "    Longitude = " << state.longitude << " degrees" << endl;
                    cout << setprecision(10) <<  "    Altitude  = " << state.altitude << " meters" << endl;
                    cout << setprecision(10) <<  "    Error     = " << state.precision << " meters" << endl;
                    cout << endl;
                });
            }
        });
    }
    void ApplicationWillTerminate()
    {
    }
};

int main (int argc, char** argv)
{
    Application::Run<SensorTestApplication>(argc, argv);
    return 0;
}
