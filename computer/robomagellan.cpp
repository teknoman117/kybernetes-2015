#include <iostream>

#include <unistd.h>
#include <ctime>
#include <csignal>
#include <thread>
#include <mutex>
#include <future>
#include <chrono>

#include "kybernetes.hpp"
#include "utility.hpp"
#include "garmingps.hpp"
#include "sensor_controller.hpp"
#include "motion_controller.hpp"

using namespace kybernetes;
using namespace std;

const string kMotionControllerAlertHeartbeat = "HEARTBEAT";
const string kMotionControllerStateArmed = "ARMED";
const string kMotionControllerStateDisarming = "DISARMING";
const string kMotionControllerStateIdle = "IDLE";
const string kMotionControllerStateKilled = "KILLED";

class Application
{
    GarminGPS        *gps;
    SensorController *sensorController;
    MotionController *motionController;

    // Possible arming states
    typedef enum _arming_state : unsigned char
    {
        Unknown,
        Killed,
        Idle,
        Armed,
        Disarming
    } ArmingState;

    // Arming stuff
    ArmingState    motionState;
    future<string> motionStateRequest;

    // Called when the GPS has been updated
    void GarminGPSHandler (GarminGPS::State& state)
    {
        // Print out the location
        //cout << "location = " << state.latitude << "," << state.longitude << ";" << state.altitude << "m @ " << asctime(localtime((time_t*)&state.timestamp));

        // Compute
    }

    // Called when the Sensor controller has posted a packet
    void SensorControllerHandler (SensorController::State& state)
    {
        // Controller is disengaged but armable.  Attempt to do so.
        if(motionState == Idle)
        {
            // Submit a request to arm the controller
            future<string> request = motionController->RequestArm();
            request.wait();
            string result = request.get();
            if(result == "OK")
            {
                motionState = Armed;
            }
            else return;
        }

        // If the state is killed do nothing
        else if(motionState != Armed)
            return;

        //std::cout << "Sonars = " << state.sonar[0] << ", " << state.sonar[1] << ", " << state.sonar[2] << std::endl;
        //std::cout << "Bumpers = " << state.bumper[0] << ", " << state.bumper[1] << std::endl;
        //std::cout << "IMU = " << state.rotation[0] << ", " << state.rotation[1] << ", " << state.rotation[2] << std::endl;
    }

    // Called when the motion controller posts an alert
    void MotionControllerAlertHandler (string alert)
    {
        // If this is a heartbeat tick
        if(alert == kMotionControllerAlertHeartbeat)
        {
            // If the current state of the controller is unknown, submit a stat request
            if(motionState == Unknown)
            {
                // If we haven't submitted a request
                if(!motionStateRequest.valid())
                    motionStateRequest = motionController->RequestArmStatus();

                // Check on the request
                if(motionStateRequest.wait_for(chrono::seconds(0)) == future_status::ready)
                {
                    string stateStr = motionStateRequest.get();
                    if(stateStr == kMotionControllerStateKilled)
                        motionState = Killed;
                    else if(stateStr == kMotionControllerStateIdle)
                        motionState = Idle;
                    else if(stateStr == kMotionControllerStateArmed)
                        motionState = Armed;
                    else if(stateStr == kMotionControllerStateDisarming)
                        motionState = Disarming;

                    cout << "Upstart State = " << stateStr << endl;
                }
            }

            // Do other things
        }

        // Detect activation state changes for the controller
        else if(alert == kMotionControllerStateKilled)
            motionState = Killed;
        else if(alert == kMotionControllerStateIdle)
            motionState = Idle;
        else if(alert == kMotionControllerStateArmed)
            motionState = Armed;
        else if(alert == kMotionControllerStateDisarming)
            motionState = Disarming;

        // Do other things
    }

public:
    Application()
        : motionState(Unknown)
    {
        // Initialize the GPS
        auto gpsCallback = bind(&Application::GarminGPSHandler, this, std::placeholders::_1);
        gps = new GarminGPS(GPSPath);
        gps->RegisterHandler(gpsCallback);

        // Initialize the Sensor controller
        auto sensorCallback = bind(&Application::SensorControllerHandler, this, std::placeholders::_1);
        sensorController = new SensorController(SensorControllerPath);
        sensorController->RegisterHandler(sensorCallback);

        // Initialize the motion controller
        auto motionAlertCallback = bind(&Application::MotionControllerAlertHandler, this, std::placeholders::_1);
        motionController = new MotionController(MotionControllerPath);
        motionController->RegisterAlertHandler(motionAlertCallback);
    }

    void Join()
    {
        gps->Join();
        sensorController->Join();
        motionController->Join();
    }

    void Close()
    {
        // TODO - do stuff to shutdown robot

        gps->Close();
        sensorController->Close();
        motionController->Close();
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
