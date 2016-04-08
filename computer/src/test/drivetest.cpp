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

namespace
{
    const float kp = 0.05;
    const float ki = 0.09;
    const float kd = 0.04;

    const float headingKp = (450.0 / 45.0);
    const short steeringExtreme = 450;

    template<typename T>
    inline T clamp(T x, T a, T b)
    {
        return x < a ? a : (x > b ? b : x);
    }

    inline double fixheading(double heading)
    {
        while(heading > 180.0) heading -= 360.0;
        while(heading < -180.0) heading += 360.0;
        return heading;
    }
}

class MotionTestApplication : public Application::Delegate
{
    unique_ptr<SensorController> sensorController;
    unique_ptr<MotionController> motionController;

    bool                         motionControllerInitialized;
    bool                         motionControllerArmed;     
    bool                         imuReady;

    SensorController::IMUState   previousState;
    float                        targetHeading;

public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {
        motionControllerInitialized = false;
        motionControllerArmed       = false;
        imuReady                    = false;
        targetHeading               = 0.f;

        // Open the motion controller
        motionController = make_unique<MotionController>(MotionControllerPath, dispatch_get_main_queue(), 57600, [this] (bool success) 
        {
            // Something went wrong
            if(!success)
            {
                cout << "An error occurred in opening the motion controller." << endl;
                Application::Instance()->Exit();
                return;
            }

            // Motion controller has become ready
            cout << "[HEY LISTEN] I am ready to go!" << endl;
            motionControllerInitialized = true;
            motionController->SetPID(kp, ki, kd);
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
                // Arm the motion controller if the motion controller is not armed
                case MotionController::AlertHeartbeat:
                    if(motionControllerInitialized && !motionControllerArmed && imuReady)
                    {
                        motionController->RequestArm([this] (bool success)
                        {
                            cout << "[HEY LISTEN] I am currently disabled by my operator!" << endl;
                            motionControllerArmed = success;
                            targetHeading = previousState[SensorController::IMUState::Yaw];
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
                    motionController->SetVelocity(900, [] (bool success)
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

        // Set the IMU handler
        sensorController->SetIMUHandler([this] (SensorController::IMUState& state)
        {
            // Based on our current heading to target, set the direction
            float error = state[2];
            float response = headingKp * error;
            short requestedSteering = clamp<short>((short) response, -steeringExtreme, steeringExtreme);
            imuReady = true;
            //previousState = state;

            cout << "heading: " << state[2] << ", error: " << error << endl;

            motionController->SetSteering(requestedSteering, [] (bool success)
            {

            });
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
