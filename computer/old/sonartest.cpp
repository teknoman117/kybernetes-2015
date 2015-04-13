#include <iostream>
#include <thread>
#include <chrono>

#include "kybernetes.hpp"
#include "motion_controller.hpp"
#include "sensor_controller.hpp"

using namespace std;
using namespace kybernetes;
using namespace kybernetes::constants;

MotionController *controller = NULL;
SensorController *sensorController = NULL;

int main (int argc, char** argv)
{
    // Get the main dispatch queue
    controller = new MotionController(MotionControllerPath);
    sensorController = new SensorController(SensorControllerPath);

    // Register a sensor handler
    sensorController->RegisterHandler([] (SensorController::State& state)
    {
        cout << "Sonars = {" << state.sonar[0] << ", " << state.sonar[1] << ", " << state.sonar[2] << "}" << endl;
    });

    // Block until we receive a heartbeat from the motion controller
    promise<void> p;
    bool done = false;
    cout << "Waiting for motion controller to be ready" << endl;
    controller->RegisterAlertHandler([&p, &done] (string s)
    {
        if(s == "HEARTBEAT" && !done)
        {
            p.set_value();
            done = true;
        }
    });
    p.get_future().wait();

    // main loop
    while(1)
    {
        // Ensure the controller is armed
        future<string> requestStatus = controller->RequestArmStatus();
        requestStatus.wait();
        if(requestStatus.get() != "ARMED")
        {
            future<string> requestArm = controller->RequestArm();
            requestArm.wait();
            if(requestArm.get() != "OK")
                continue;

            cout << "Controller is ARMED" << endl;
        }

        // Set the velocity
        controller->SetVelocity(65);
        controller->SetSteering(0);
    }

    return 0;
}
