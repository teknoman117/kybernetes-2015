#include <iostream>
#include <fstream>
#include <iomanip>
#include <unistd.h>
#include <ctime>
#include <csignal>
#include <thread>
#include <mutex>
#include <future>
#include <chrono>
#include <list>
#include <algorithm>

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

// Navigation constants on speed and distance
const double closeDistanceThreshold = 10.0;    // 10 meters
const short  farSpeed = 135;
const short  nearSpeed = 70;
const double headingKp = (450.0 / 45.0);
const short  steeringExtreme = 450;

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

class Application
{
public:
    // Waypoint along the goal path
    struct Waypoint
    {
        GarminGPS::State coordinate;
        bool             isCone;
    };

    // Load the goal manifest from a file
    static void LoadFromFile(string path, list<Waypoint>& waypoints)
    {
        // Load the coordinates from the manifest file
        std::ifstream manifest(path);
        waypoints.clear();
        while(!manifest.eof())
        {
            // Load the coordinate from the file
            Waypoint point;
            manifest >> point.coordinate.latitude;
            manifest >> point.coordinate.longitude;
            manifest >> point.isCone;

            // Check if the coordinate is valid
            if(manifest.eof()) break;

            // Otherwise store the coordinate
            waypoints.push_back(point);
        }
        manifest.close();
    }

private:
    GarminGPS                *gps;
    SensorController         *sensorController;
    MotionController         *motionController;

    // Route information
    list<Waypoint>&           route;
    list<Waypoint>::iterator  currentObjective;
    volatile double           headingToObjective;
    volatile double           distanceToObjective;

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
        // If there are no more objective, dont do shit
        if(currentObjective == route.end())
            return;

        // Compute heading and distance to target
        headingToObjective = state.HeadingTo(currentObjective->coordinate);
        distanceToObjective = state.DistanceTo(currentObjective->coordinate);

        // If we are within the error distance of our target, choose the next objective
        if(distanceToObjective <= state.precision)
        {
            // Is the current objective a cone (switch to visual guidance)


            // Otherwise choose the next target
            if(++currentObjective == route.end())
            {
                Close();
            } else
            {
                headingToObjective = state.HeadingTo(currentObjective->coordinate);
                distanceToObjective = state.DistanceTo(currentObjective->coordinate);
            }
        }

        cout << "Received GPS Packet @" << state.timestamp << " {" << distanceToObjective << ", " << headingToObjective << "}" << endl;
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

        // Based on our current heading to target, set the direction
        double error = -fixheading(headingToObjective - state.rotation[2]);
        double response = headingKp * error;
        short requestedSteering = clamp<short>((short) response, -steeringExtreme, steeringExtreme);
        motionController->SetSteering(requestedSteering);

        // Based on the distance to target, set the speed. in future apply a logistic function: 1 / (exp(-(distance - kHalfspeedistance)) + 1)
        short requestedVelocity = (distanceToObjective > closeDistanceThreshold) ? farSpeed : nearSpeed;
        motionController->SetVelocity(requestedVelocity);

        cout << "Heading = " << state.rotation[2] << ", Error = " << error << ", response = " << requestedSteering << ", distance = " << distanceToObjective << endl;
        //cout << "Sonars = {" << state.sonar[0] << ", " << state.sonar[1] << ", " << state.sonar[2] << "}" << endl;
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
    Application(list<Waypoint>& route)
        : route(route), currentObjective(route.begin()), motionState(Unknown)
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
        // Shutdown the robot
        cout << "INFO: Disarming" << endl;
        motionController->RequestDisarm().wait();

        // Kill off our sensors
        cout << "INFO: Shutting Down" << endl;
        gps->Close();
        sensorController->Close();
        motionController->Close();
    }
};

static Application *application;

void killHandler(int signal)
{
    // Just close
    cout << "INFO: Termination Requested" << endl;
    application->Close();
}

int main (int argc, char **argv)
{
    // Verify parameter count
    if(argc < 2)
    {
        cerr << "Usage: " << argv[0] << " <path to waypoint list>" << endl;
        return 1;
    }

    // Process any input path information
    signal(SIGINT, killHandler);

    // Load the objective from a file
    list<Application::Waypoint> route;
    Application::LoadFromFile(argv[1], route);
    for_each(route.begin(), route.end(), [] (Application::Waypoint& waypoint)
    {
        cout << setprecision(10) << "Waypoint: ";
        cout << setprecision(10) << waypoint.coordinate.latitude << ", ";
        cout << setprecision(10) << waypoint.coordinate.longitude << ", ";
        cout << setprecision(10) << "is cone = " << waypoint.isCone << endl;
    });

    // Verify
    if(route.empty())
    {
        cerr << "No waypoints specified, exiting..." << endl;
        return 1;
    }

    // Launch robomagellan
    application = new Application(route);
    application->Join();
    return 0;
}
