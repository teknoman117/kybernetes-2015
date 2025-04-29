#include <iostream>
#include "movetotask.hpp"

using namespace std;
using namespace kybernetes::controller;
using namespace kybernetes::sensor;

// Unexported symbols
namespace
{
    const float headingKp = (450.0 / 45.0);
    const short steeringExtreme = 450;
    const double closeDistanceThreshold = 5.0;    // 10 meters

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

// Read the manifest
MoveToTask::MoveToTask(ifstream& manifest)
	: Task(), armed(false), avoiding(false), debug(0)
{
	manifest >> waypoint.latitude;
    manifest >> waypoint.longitude;
    manifest >> highspeed;
    manifest >> lowspeed;

    cout << "[MOVETOTASK] INFO: Created moveto " << waypoint.latitude << " degrees north"
         << ", " << waypoint.longitude << " degrees east "
         << " @ low = " << lowspeed << ", high = " << highspeed << endl; 

    flags[0] = flags[1] = flags[2] = false;
}


void MoveToTask::HandleSonarMessage(const kybernetes::controller::SensorController::SonarState& state)
{
    const float collisionThreshold = 100.f;

    if(!taskEnabled)
        return;

    // Check if there are obstacles we need to avoid
    bool currentFlags[3] = {
        (state[0] > 0.0f && state[0] <= collisionThreshold),
        (state[1] > 0.0f && state[1] <= collisionThreshold),
        (state[2] > 0.0f && state[2] <= collisionThreshold)
    };

    // Find obstables
    cout << "warning " << (flags[0] && currentFlags[0]) << " " << (flags[1] && currentFlags[1]) << " " << (flags[2] && currentFlags[1]) << endl;

    // Store the previous flags
    flags[0] = currentFlags[0];
    flags[1] = currentFlags[1];
    flags[2] = currentFlags[2];
}

void MoveToTask::HandleBumperMessage(const kybernetes::controller::SensorController::BumperState& state)
{
    // ignore bumper for moveto
}

void MoveToTask::HandleIMUMessage(const kybernetes::controller::SensorController::IMUState& state)
{
    // Only update the steering due to the IMU if we aren't avoiding obstacles
    if(!taskEnabled && !avoiding)
        return;



	// Based on our current heading to target, set the direction
	float error = -fixheading(headingToObjective - state[SensorController::IMUState::Yaw]);
	float response = headingKp * error;

	short requestedSteering = clamp<short>((short) response, -steeringExtreme, steeringExtreme);
	motionController->SetSteering(requestedSteering, [] (bool success)
	{
	});

	short requestedVelocity = (distanceToObjective > closeDistanceThreshold) ? highspeed : lowspeed;
	motionController->SetVelocity(requestedVelocity, [] (bool success)
	{
	});

    //cout << "Received IMU error = " << error << ", steering = " << requestedSteering << ", velocity = " << requestedVelocity << endl;
}

void MoveToTask::HandleGPSMessage(const kybernetes::sensor::GarminGPS::State& state)
{
	if(!taskEnabled)
		return;

	// Compute heading and distance to target
	headingToObjective = state.HeadingTo(waypoint);
	distanceToObjective = state.DistanceTo(waypoint);

	// If we are within the error distance of our target, choose the next objective
	if(distanceToObjective <= state.precision)
	{
		taskEnabled = false;
		if(taskCompletedCallback)
        {
			taskCompletedCallback(*this);
        }
	}

	cout << "Received GPS Packet @" << state.timestamp << " {" << distanceToObjective << " meters, " << headingToObjective << " degrees}" << endl;
}

void MoveToTask::HandleMotionControllerAlert(const kybernetes::controller::MotionController::Alert& alert)
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

void MoveToTask::HandleCameraFrame(const kybernetes::sensor::V4L2Camera::FrameBuffer&)
{
    // Ignore camera in moveto
}

