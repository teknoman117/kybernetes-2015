#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <memory>
#include <limits>

#include <kybernetes/constants/constants.hpp>
#include <kybernetes/utility/application.hpp>
#include <kybernetes/controller/motion_controller.hpp>
#include <kybernetes/controller/sensor_controller.hpp>

#include <dispatch/dispatch.h>

using namespace std;

using namespace kybernetes::controller;
using namespace kybernetes::constants;
using namespace kybernetes::utility;

class MotionTestApplication : public Application::Delegate
{
    unique_ptr<SensorController> sensorController;
    unique_ptr<MotionController> motionController;

    bool                         motionControllerInitialized;
    bool                         motionControllerArmed;     
    bool                         collisionEmergency;  

public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {
        motionControllerInitialized = false;
        motionControllerArmed       = false;
        collisionEmergency          = false;

        // Open the motion controller
        motionController = make_unique<MotionController>(MotionControllerPath, dispatch_get_main_queue(), 57600, [] (MotionController *controller, bool success) 
        {
            if(!success)
            {
                cout << "An error occurred in opening the motion controller." << endl;
                Application::Instance()->Exit();
                return;
            }
        });

        sensorController = make_unique<SensorController>(SensorControllerPath, dispatch_get_main_queue(), 57600, [] (SensorController *controller, bool success)
        {
            if(!success)
            {
                cout << "An error occurred in opening the sensor controller." << endl;
                Application::Instance()->Exit();
                return;
            }
        });

        // Set the alert handler for the motion controller
        motionController->SetAlertHandler([this] (MotionController::Alert alert)
        {
            switch(alert)
            {
                case MotionController::AlertReady:
                    motionControllerInitialized = true;
                    cout << "[HEY LISTEN] I am ready to go!" << endl;
                    break;

                // Arm the motion controller if the motion controller is not armed
                case MotionController::AlertHeartbeat:
                    if(motionControllerInitialized && !motionControllerArmed && !collisionEmergency)
                    {
                        motionController->RequestArm([this] (bool success)
                        {
                            cout << "[HEY LISTEN] I am currently disabled by my operator!" << endl;
                            motionControllerArmed = success;
                        });
                    }
                    break;

                // Check for a disarming notification
                case MotionController::AlertStatusIdle:
                    // do something about being idle

                case MotionController::AlertStatusKilled:
                case MotionController::AlertStatusDisarming:
                    motionControllerArmed = false;
                    cout << "[HEY LISTEN] I have been disabled by my operator!" << endl;
                    break;

                // If we become armed, sent a set velocity request
                case MotionController::AlertStatusArmed:
                    motionController->SetVelocity(20, [] (bool success)
                    {
                        if(!success)
                            cout << "[HEY LISTEN] I failed to set my velocity" << endl;
                        else
                            cout << "[HEY LISTEN] I AM MOVING!!" << endl;
                    });

                default:
                    break;
            };
        });


    }

    void ApplicationWillTerminate()
    {
    }
};

int main (int argc, char** argv)
{
    Application::Run<MotionTestApplication>(argc, argv);
    return 0;
}
