#include <kybernetes/controller/motion_controller.hpp>
#include <kybernetes/utility/utility.hpp>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <sstream>
//#include <iostream>

using namespace std;

namespace 
{
    static const string statusNames[8] = {"ARMED", "DISARMING", "IDLE", "KILLED", "HEARTBEAT", "TIMEOUT", "RESET", "READY"};
    static const size_t statusNamesLength = 8;
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
                // Parse the message and parameters
                vector<string> commands;
                vector<string> parameters;

                utility::tokenize(message, ":", commands);
                if(commands.size() != 2)
                {
                    return;
                }
                utility::tokenize(commands[1], ";", parameters);

                /*std::cout << commands[0] << " ";
                for_each(parameters.begin(), parameters.end(), [] (std::string s)
                {
                    std::cout << s << " ";
                });
                std::cout << std::endl;*/

                // Alert response
                if(commands[0] == "ALERT")
                {
                    if(parameters.size() != 1)
                        return;

                    auto command = find(statusNames, statusNames + statusNamesLength, parameters[0]);
                    MotionController::Alert alert = static_cast<MotionController::Alert>(distance(statusNames, command));
                    alertHandler(alert);
                }

                // Request response
                else if(parameters.size() == 2)
                {
                    uint8_t code = atoi(parameters[1].c_str());

                    if(commands[0] == "ARM")
                    {
                        lock_guard<mutex> lock(requestArmCallbacksMutex);
                        auto handlerIt = requestArmCallbacks.find(code);
                        handlerIt->second(parameters[0] == "OK");
                        requestArmCallbacks.erase(handlerIt);
                    }
                    else if(commands[0] == "DISARM")
                    {
                        lock_guard<mutex> lock(requestDisarmCallbacksMutex);
                        auto handlerIt = requestDisarmCallbacks.find(code);
                        handlerIt->second(parameters[0] == "OK");
                        requestDisarmCallbacks.erase(handlerIt);
                    }
                    else if(commands[0] == "PING")
                    {
                        lock_guard<mutex> lock(requestPingCallbacksMutex);
                        auto handlerIt = requestPingCallbacks.find(code);
                        handlerIt->second(parameters[0] == "OK");
                        requestPingCallbacks.erase(handlerIt);
                    }
                    else if(commands[0] == "STEER")
                    {
                        lock_guard<mutex> lock(setSteeringCallbacksMutex);
                        auto handlerIt = setSteeringCallbacks.find(code);
                        handlerIt->second(parameters[0] == "OK");
                        setSteeringCallbacks.erase(handlerIt);
                    }
                    else if(commands[0] == "VELOCITY")
                    {
                        lock_guard<mutex> lock(setVelocityCallbacksMutex);
                        auto handlerIt = setVelocityCallbacks.find(code);
                        handlerIt->second(parameters[0] == "OK");
                        setVelocityCallbacks.erase(handlerIt);
                    }
                    else if(commands[0] == "ARMSTAT")
                    {
                        lock_guard<mutex> lock(requestArmStatusCallbacksMutex);
                        auto handlerIt = requestArmStatusCallbacks.find(code);

                        // Get the enum entry of the response
                        auto command = find(statusNames, statusNames + statusNamesLength, parameters[0]);
                        MotionController::ArmingStatus status = static_cast<MotionController::ArmingStatus>(distance(statusNames, command));
                        handlerIt->second(status);
                        requestArmStatusCallbacks.erase(handlerIt);
                    }
                }
            });

            // setup index
            index = 0;
        }

        MotionController::~MotionController()
        {
            RequestDisarm([] (bool) {});
        }

        std::string MotionController::GetStringRepresentation(Alert alert)
        {
            if(alert >= statusNamesLength)
                return "";

            return statusNames[alert];
        }

        void MotionController::SetAlertHandler(const MotionController::AlertCallback& handler)
        {
            alertHandler = handler;
        }

        void MotionController::RequestArm(const MotionController::SuccessCallback& handler)
        {
            lock_guard<mutex> lock(requestArmCallbacksMutex);
            int code = index++;
            requestArmCallbacks[code] = handler;

            // Push out the command
            stringstream stream;
            stream << "ARM:" << code << "\r\n";
            device->Write(stream.str());
        }

        void MotionController::RequestDisarm(const MotionController::SuccessCallback& handler)
        {
            lock_guard<mutex> lock(requestDisarmCallbacksMutex);
            int code = index++;
            requestDisarmCallbacks[code] = handler;

            // Push out the command
            stringstream stream;
            stream << "DISARM:" << code << "\r\n";
            device->Write(stream.str());
        }

        void MotionController::RequestArmStatus(const MotionController::ArmingStatusCallback& handler)
        {
            lock_guard<mutex> lock(requestArmStatusCallbacksMutex);
            int code = index++;
            requestArmStatusCallbacks[code] = handler;

            // Push out the command
            stringstream stream;
            stream << "ARMSTAT:" << code << "\r\n";
            device->Write(stream.str());
        }

        void MotionController::RequestPing(const MotionController::SuccessCallback& handler)
        {
            lock_guard<mutex> lock(requestPingCallbacksMutex);
            int code = index++;
            requestPingCallbacks[code] = handler;

            // Push out the command
            stringstream stream;
            stream << "PING:" << code << "\r\n";
            device->Write(stream.str());
        }

        void MotionController::SetVelocity(short velocity, const MotionController::SuccessCallback& handler)
        {
            lock_guard<mutex> lock(setVelocityCallbacksMutex);
            int code = index++;
            setVelocityCallbacks[code] = handler;

            // Push out the command
            stringstream stream;
            stream << "VELOCITY:" << velocity << ";" << code << "\r\n";
            device->Write(stream.str());
        }

        void MotionController::SetSteering(short steering, const MotionController::SuccessCallback& handler)
        {
            lock_guard<mutex> lock(setSteeringCallbacksMutex);
            int code = index++;
            setSteeringCallbacks[code] = handler;

            // Push out the command
            stringstream stream;
            stream << "STEER:" << steering << ";" << code << "\r\n";
            device->Write(stream.str());
        }
    }
}
