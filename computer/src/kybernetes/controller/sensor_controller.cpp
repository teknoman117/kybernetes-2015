#include <kybernetes/controller/sensor_controller.hpp>
#include <kybernetes/utility/utility.hpp>

#include <algorithm>
#include <cmath>
#include <ctime>

using namespace std;

namespace kybernetes
{
    namespace controller
    {
        // Open the GPS
        SensorController::SensorController(std::string path, dispatch_queue_t queue_, const uint32_t baudrate, SuccessCallback&& callback)
            : queue(queue_), readyCallback(move(callback))
        {
            // Open the sensor controller device
            device = make_unique<io::SerialDispatchDevice>(path, queue, baudrate, [this] (bool success)
            {
                // An error occurred
                if(!success)
                {
                    dispatch_async(queue, ^{readyCallback(false);});
                    return;
                }

                // Register the handler for receipt of a message
                device->SetHandler(bind(&SensorController::ReceiveMessageHandler, this, placeholders::_1));
            });
        }

        void SensorController::SetSonarHandler(std::function<void (SensorController::SonarState&)>&& handler)
        {
            sonarCallback = move(handler);
        }

        void SensorController::SetIMUHandler(std::function<void (SensorController::IMUState&)>&& handler)
        {
            imuCallback = move(handler);
        }

        void SensorController::SetBumperHandler(std::function<void (SensorController::BumperState&)>&& handler)
        {
            bumperCallback = move(handler);
        }

        void SensorController::ReceiveMessageHandler(const std::string& message)
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
            if(commands[0] == "STATUS")
            {
                if(parameters.size() < 1) 
                    return;

                if(parameters[0] == "READY")
                {
                    readyCallback(true);
                }
                else if(parameters[0] == "ERROR")
                {
                    readyCallback(false);
                    device.reset(nullptr);
                }
            }
            else if(commands[0] == "IMU")
            {
                SensorController::IMUState state;
                if(parameters.size() != 3)
                    return;
                for(int i = 0; i < 3; i++)
                    state.angles[i] = atof(parameters[i].c_str());
                if(imuCallback)
                    imuCallback(state);
            }
            else if(commands[0] == "SONAR")
            {
                SensorController::SonarState state;
                if(parameters.size() != 3)
                    return;
                for(int i = 0; i < 3; i++)
                    state.distance[i] = atof(parameters[i].c_str());
                if(sonarCallback)
                    sonarCallback(state);
            }
            else if(commands[0] == "BUMPER")
            {
                SensorController::BumperState state;
                if(parameters.size() != 2)
                    return;
                for(int i = 0; i < 2; i++)
                    state.value[i] = (atoi(parameters[i].c_str()) == 1);
                if(bumperCallback)
                    bumperCallback(state);
            }
        }
    }
}
