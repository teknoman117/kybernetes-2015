#include <iostream>
#include "findconetask.hpp"

using namespace std;
using namespace kybernetes::controller;
using namespace kybernetes::sensor;

// Read the manifest
FindConeTask::FindConeTask(ifstream& manifest)
	: Task(), armed(false)
{
	/*manifest >> waypoint.latitude;
    manifest >> waypoint.longitude;
    manifest >> highspeed;
    manifest >> lowspeed;

    cout << "[MOVETOTASK] INFO: Created moveto " << waypoint.latitude << " degrees north"
         << ", " << waypoint.longitude << " degrees east "
         << " @ low = " << lowspeed << ", high = " << highspeed << endl; */
         cout << "[MOVETOTASK] INFO: Created findcone " << endl;
}


void FindConeTask::HandleSonarMessage(const kybernetes::controller::SensorController::SonarState& state)
{

}

void FindConeTask::HandleBumperMessage(const kybernetes::controller::SensorController::BumperState& state)
{

}

void FindConeTask::HandleIMUMessage(const kybernetes::controller::SensorController::IMUState& state)
{

}

void FindConeTask::HandleGPSMessage(const kybernetes::sensor::GarminGPS::State& state)
{

}

void FindConeTask::HandleMotionControllerAlert(const kybernetes::controller::MotionController::Alert& alert)
{
	if(!motionController || !taskEnabled)
		return;

	switch(alert)
    {
        // If the motion controller is not armed, attempt to arm it
        case MotionController::AlertHeartbeat:
            if(!armed)
            {
                motionController->RequestArm([this] (bool success)
                {
                    if(!success)
                    	cerr << "[MOVETOTASK: MOTION] ALERT: I am currently disabled by my operator!" << endl;
                    else
                    	cout << "[MOVETOTASK: MOTION] ALERT: Witness the power of the fully armed and operational battlestation (/robot)!" << endl;

                    armed = success;
                });
                motionController->SetVelocity(30, [] (bool) {});
                motionController->SetSteering(450, [] (bool) {});
            } 
            else
            {

            }
            break;

        // Check for a disarming notification
        case MotionController::AlertStatusIdle:
        case MotionController::AlertStatusKilled:
        case MotionController::AlertStatusDisarming:
            armed = false;
            cout << "[MOVETOTASK: MOTION] ALERT: I have been disabled by my operator!" << endl;
            break;

        // If we become armed, sent a set velocity request
        case MotionController::AlertStatusArmed:
            break;

        default:
            break;
    };
}

void FindConeTask::HandleCameraFrame(const kybernetes::sensor::V4L2Camera::FrameBuffer&)
{

}

