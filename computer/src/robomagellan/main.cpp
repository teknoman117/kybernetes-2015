#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <memory>
#include <limits>
#include <algorithm>

#include <kybernetes/constants/constants.hpp>
#include <kybernetes/utility/application.hpp>
#include <kybernetes/controller/motion_controller.hpp>
#include <kybernetes/controller/sensor_controller.hpp>
#include <kybernetes/sensor/garmingps.hpp>
#include <kybernetes/sensor/v4l2camera.hpp>

#include <dispatch/dispatch.h>

#include "movetotask.hpp"
#include "findconetask.hpp"

using namespace std;

using namespace kybernetes::controller;
using namespace kybernetes::constants;
using namespace kybernetes::utility;
using namespace kybernetes::sensor;

class MotionTestApplication : public Application::Delegate
{
    unique_ptr<SensorController>  sensorController;
    unique_ptr<MotionController>  motionController;
    unique_ptr<GarminGPS>         gps;
    unique_ptr<V4L2Camera>        camera;

    // Initialization stuff
    bool motionControllerInitialized = false;
    bool sensorControllerInitialized = false;
    bool gpsInitialized              = false;
    bool cameraInitialized           = false;

    vector<unique_ptr<Task>>           tasks;
    vector<unique_ptr<Task>>::iterator currentTask;

public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {
    	// Config stuff
		float Kc = 0.10;
		float tI = 1.00;
		float tD = 0.01;

    	// Process configuration file arguments
        if(argc < 2)
        {
        	cerr << "[APPLICATION] FATAL: no manifest file specified" << endl;
            Application::Instance()->Exit();
            return;
        }

        // Load the configuration
        std::ifstream manifest(argv[1]);
        if(!manifest.is_open())
        {
        	cerr << "[APPLICATION] FATAL: manifest file not found" << endl;
            Application::Instance()->Exit();
            return;
        }

        // Read the manifest
        while(!manifest.eof())
        {
        	string type;
        	manifest >> type;

        	if(type == "PID")
        	{
        		manifest >> Kc;
        		manifest >> tI;
        		manifest >> tD;
            	cout << "PID CONSTANTS = " << Kc << " " << tI << " " << tD << endl;
        	}

        	else if(type == "MOVETO")
        	{
        		tasks.push_back(make_unique<MoveToTask>(manifest));
        	}

            else if(type == "FINDCONE")
            {
                tasks.push_back(make_unique<FindConeTask>(manifest));
            }
        }
        currentTask = tasks.end();

        // Open the motion controller
        motionController = make_unique<MotionController>(MotionControllerPath, dispatch_get_main_queue(), 57600, [this, Kc, tI, tD] (bool success) 
        {
            // Something went wrong
            motionControllerInitialized = success;
            if(!success)
            {
                cerr << "[APPLICATION] FATAL: Motion controller failed to initialize" << endl;
                Application::Instance()->Exit();
                return;
            }

            // Motion controller has become ready
            cout << "[APPLICATION] INFO: Motion controller becomes ready" << endl;
            motionController->SetPID(Kc, tI, tD);

            // Turn on the debug mode
            /*motionController->SetDebug();
            motionController->SetDebugHandler([] (const vector<string>& messages)
            {
                std::cout << "debug: ";
                for_each(messages.begin(), messages.end(), [] (const string& m)
                {
                    std::cout << m << " ";
                });
                std::cout << std::endl;
            });*/

            // Register the alert handler to feed into the current task
            motionController->SetAlertHandler([&] (MotionController::Alert alert)
            {
            	if(currentTask != tasks.end())
            		(*currentTask)->HandleMotionControllerAlert(alert);
            });

            BeginTasks();
        });

        // Open the sensor controller
        sensorController = make_unique<SensorController>(SensorControllerPath, dispatch_get_main_queue(), 57600, [&] (bool success)
        {
        	// Something went wrong
        	sensorControllerInitialized = success;
            if(!success)
            {
                cerr << "[APPLICATION] FATAL: Sensor controller failed to initialize" << endl;
                Application::Instance()->Exit();
                return;
            }

            // Set the IMU handler
            cout << "[APPLICATION] INFO: Sensor controller becomes ready" << endl;
            sensorController->SetIMUHandler([this] (SensorController::IMUState& state)
            {
            	if(currentTask != tasks.end())
            		(*currentTask)->HandleIMUMessage(state);
            });
            sensorController->SetBumperHandler([this] (SensorController::BumperState& state)
            {
            	if(currentTask != tasks.end())
            		(*currentTask)->HandleBumperMessage(state);
            });
            sensorController->SetSonarHandler([this] (SensorController::SonarState& state)
            {
            	if(currentTask != tasks.end())
            		(*currentTask)->HandleSonarMessage(state);
            });

            BeginTasks();
        });

        // Open the GPS
        gps = std::make_unique<GarminGPS>(GPSPath, dispatch_get_main_queue(), 9600, [&] (GarminGPS *gps, bool success)
        {
            // Something went wrong
            if(!success)
            {
                cerr << "[APPLICATION] FATAL: GPS failed to initialize" << endl;
                Application::Instance()->Exit();
                return;
            }

            // Register the gps handler
            gps->SetHandler([&] (const GarminGPS::State& state)
            {
            	if(state.status == GarminGPS::State::Invalid || state.status == GarminGPS::State::Simulated)
            	{
            		cerr << "[APPLICATION] WARN: Waiting for GPS fix" << endl;
            		return;
            	}

                if(!gpsInitialized)
                {
                	cout << "[APPLICATION] INFO: GPS becomes ready" << endl;
                	cout << "[APPLICATION] INFO: Fix @ " << asctime(localtime((time_t *) &state.timestamp));
                    cout << setprecision(10) << "    Latitude  = " << state.latitude << " degrees" << endl;
                    cout << setprecision(10) <<  "    Longitude = " << state.longitude << " degrees" << endl;
                    cout << setprecision(10) <<  "    Altitude  = " << state.altitude << " meters" << endl;
                    cout << setprecision(10) <<  "    Error     = " << state.precision << " meters" << endl;
                    cout << endl;

                	gpsInitialized = true;
                	dispatch_async(dispatch_get_main_queue(), ^{BeginTasks();});	
                }

            	if(currentTask != tasks.end())
            		(*currentTask)->HandleGPSMessage(state);
            });
        });

	    // Open the camera
	    camera = make_unique<V4L2Camera>("/dev/video0", 320, 240, V4L2_FIELD_ANY, V4L2_PIX_FMT_YUYV, 30, dispatch_get_main_queue(), [&] (bool success)
	    {
            // Something went wrong
            if(!success)
            {
                cerr << "[APPLICATION] FATAL: Camera failed to initialize" << endl;
	            Application::Instance()->Exit();
	            return;
	        }

	        // Frame callback
	        //camera->SetStreaming(true);
	        camera->SetFrameCaptureCallback([&] (const V4L2Camera::FrameBuffer &image)
	        {
	        	if(!cameraInitialized)
	        	{
	        		cout << "[APPLICATION] INFO: Camera becomes ready" << endl;
                	cameraInitialized = true;
                	dispatch_async(dispatch_get_main_queue(), ^{BeginTasks();});	
                }

            	if(currentTask != tasks.end())
            		(*currentTask)->HandleCameraFrame(image);
	        });
	    });
    }

    void BeginTasks()
    {
    	// Start the first task
    	if(!motionControllerInitialized || !sensorControllerInitialized || !gpsInitialized /*|| !cameraInitialized*/ )
    	{
    		return;
    	}

    	// Assign stuff
    	for_each(tasks.begin(), tasks.end(), [&] (unique_ptr<Task>& t)
    	{
    		// Setup the pointers to the robot controllers
    		t->motionController = motionController.get();
    		t->sensorController = sensorController.get();
    		t->gps              = gps.get();
    		t->camera           = camera.get();

    		t->SetTaskCompletedCallback([&] (Task& task)
    		{
    			// Disable the current task
    			task.taskEnabled = false;

    			// Select the next task or stop
    			currentTask++;
    			if(currentTask == tasks.end())
    			{
    				Shutdown();
    			}
    			else
    			{
    				(*currentTask)->taskEnabled = true;
    				cout << "[APPLICATION] INFO: Executing next task" << endl;
    			}
    		});
    	});

    	// Fire off the first task
    	cout << "[APPLICATION] INFO: Queuing first task" << endl;
    	currentTask = tasks.begin();
    	if(currentTask != tasks.end())
    	{
    		(*currentTask)->taskEnabled = true;
    	}
    }

    void Shutdown()
    {
		motionController->RequestDisarm([&] (bool success)
		{
			if(success)
			{
				cout << "[APPLICATION] INFO: Robot successfully shut down.  Run complete." << endl;
			}
			else
			{
				cout << "[APPLICATION] ALERT: Robot failed to shutdown. Run complete." << endl;
			}
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
