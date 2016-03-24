#include <kybernetes/controller/sensor_controller.hpp>
#include <kybernetes/utility/utility.hpp>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iostream>

using namespace std;

namespace kybernetes
{
    namespace controller
    {
        // Open the GPS
        SensorController::SensorController(std::string path, dispatch_queue_t queue, const uint32_t baudrate)
        {
            // Open the sensor controller device
            device = new io::SerialDispatchDevice(path, queue, baudrate, [] (int error)
            {
                if(error)
                {
                    // do somethign about it
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
                if(commands[0] == "IMU")
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
            });
        }

        SensorController::~SensorController()
        {
            std::cout << "closing sensor controller" << std::endl;
            delete device;
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
    }
}
