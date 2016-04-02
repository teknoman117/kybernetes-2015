#include <kybernetes/controller/motion_controller.hpp>
#include <kybernetes/utility/utility.hpp>

#include <algorithm>
#include <cmath>
#include <ctime>

using namespace std;

namespace
{
	static const string commandNames[4]    = {"ARM", "DISARM", "ARMSTAT", "PING"};
	static const size_t commandNamesLength = 4;
}


namespace kybernetes
{
    namespace controller
    {
        MotionController::MotionController(const std::string& path, dispatch_queue_t queue, uint32_t baudrate)
        	: queue(queue)
        {
            // Open the sensor controller device
            device = make_unique<io::SerialDispatchDevice>(path, queue, baudrate, [] (int error)
            {
                if(error)
                {
                    // do something about it
                }
            });

            // Register the handler for receipt of a message
            device->SetHandler([this] (const string& message)
            {
                // Decipher the packet from the controller
                vector<string> commands;
                utility::tokenize(message, ":", commands);
                if(commands.size() != 2)
                {
                    return;
                }

                // Get the parameters sent to the computer
                vector<string> parameters;
                utility::tokenize(commands[1], ";", parameters);

                // Process the command
			    if(commands[0] == "ALERT")
			    {
			        if(parameters.size() != 1)
			            return;

			        // Alert the caller
			        alertHandler(parameters[0]);
			    }

			    // Is this a known "one response" command?
				else if(find(commandNames, commandNames + commandNamesLength, commands[0]) != commandNames + commandNamesLength)
				{
					if(parameters.size() != 2)
						return;

					// Process the result of the arm command
					int code = atoi(parameters[1].c_str());
					/*{
						lock_guard<mutex> lock(requestsMutex);
						auto request = requests.find(make_pair(code, commands[0]));
						if(request != requests.end())
						{
							request->second.set_value(parameters[0]);
							requests.erase(request);
						}
					}*/
				}
            });
        }

        MotionController::~MotionController()
        {
        }

        void MotionController::SetAlertHandler(std::function<void (const std::string&)>&& handler)
        {
        	alertHandler = std::move(handler);
        }

        void MotionController::RequestArm(std::function<void (bool)> handler)
        {

        }

        void MotionController::RequestDisarm(std::function<void (bool)> handler)
        {

        }

        void MotionController::RequestArmStatus(std::function<void (ArmingStatus)> handler)
        {

        }

        void MotionController::RequestPing(std::function<void (bool)> handler)
        {

        }

    	void MotionController::SetVelocity(short velocity)
    	{

    	}

        void MotionController::SetSteering(short steering)
        {

        }
    }
}
