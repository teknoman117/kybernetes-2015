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
        // Open the motion controller
        motionController = make_unique<MotionController>(MotionControllerPath, dispatch_get_main_queue(), 57600, [] (bool) {});
        sensorController = make_unique<SensorController>(SensorControllerPath, dispatch_get_main_queue(), 57600, [] (SensorController *, bool) {});
        motionControllerInitialized = false;
        motionControllerArmed       = false;
        collisionEmergency          = false;


        // Set the alert handler for the motion controller
        motionController->SetAlertHandler([this] (MotionController::Alert alert)
        {
            switch(alert)
            {
                //case MotionController::AlertReady:
                //    motionControllerInitialized = true;
                //    cout << "[HEY LISTEN] I am ready to go!" << endl;
                //    break;

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
                    motionController->SetVelocity(50, [] (bool success)
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

        // Setup the sensor controller event
        sensorController->SetSonarHandler([this] (SensorController::SonarState& state)
        {
            const float minimumTrigger = 10.f;
            const float emergencyTrigger = 55.f;
            const float maxResponseTrigger = 70.f;
            const float maximumTrigger = 140.f;

            if(state[0] < minimumTrigger) state.distance[0] = numeric_limits<float>::max();
            if(state[1] < minimumTrigger) state.distance[1] = numeric_limits<float>::max();
            if(state[2] < minimumTrigger) state.distance[2] = numeric_limits<float>::max();

            // Clear mode
            if((state[0] > maximumTrigger) &&
               (state[1] > maximumTrigger) &&
               (state[2] > maximumTrigger))
            {
                //cout << "[DERP] sending steering request" << endl;
                collisionEmergency = false;
                motionController->SetSteering(0, [] (bool success) { /*cout << "[DERP] steering response came back" << endl;*/ });
            }

            // Are we in a state of emergency
            else if((state[0] < emergencyTrigger) ||
                    (state[1] < emergencyTrigger) ||
                    (state[2] < emergencyTrigger))
            {
                cout << "[ALERT] COLLISION EMERGENCY!!!!!!>!>!>" << endl;
                collisionEmergency = true;
                motionController->RequestDisarm([this] (bool) {});
            }

            // Evasion mode
            else 
            {
                collisionEmergency = false;

                float responses[3] = {0.f, 0.f, 0.0f};
                if(state[0] <= maximumTrigger)
                    responses[0] = (maximumTrigger - max(state[0], maxResponseTrigger)) / (maximumTrigger - maxResponseTrigger);
                if(state[1] <= maximumTrigger)
                    responses[1] = (maximumTrigger - max(state[1], maxResponseTrigger)) / (maximumTrigger - maxResponseTrigger);
                if(state[2] <= maximumTrigger)
                    responses[2] = (maximumTrigger - max(state[2], maxResponseTrigger)) / (maximumTrigger - maxResponseTrigger);

                float direction = (responses[0] * -450.f) + (responses[2] * 450.f);

                cout << direction << " :: " << responses[0] << " (" << state[0] << "), " << responses[1] << " (" << state[1] << "), " << responses[2] << " (" << state[2] << ")"  << endl;
                motionController->SetSteering(direction, [] (bool success) { /*cout << "[DERP] steering response came back" << endl;*/ });
            }
        });
    }

    void ApplicationWillTerminate()
    {
        cout << "Termination Requested" << endl;
    }
};

int main (int argc, char** argv)
{
    Application::Run<MotionTestApplication>(argc, argv);
    return 0;
}
