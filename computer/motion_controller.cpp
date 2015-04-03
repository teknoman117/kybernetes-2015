#include "motion_controller.hpp"
#include "kybernetes.hpp"
#include "utility.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <sstream>
#include <iostream>

using namespace kybernetes;
using namespace std;

// Open the GPS
MotionController::MotionController(std::string path, const SerialPort::BaudRate baudRate)
    : SerialDevice(path, baudRate)
{

}

MotionController::~MotionController()
{
    this->Close();
}

// Process a message received from the GPS
void MotionController::processMessage(std::string& message)
{
    // Copy the message
    std::string mcopy = message;
    mcopy.erase(std::remove_if(mcopy.begin(),
                               mcopy.end(),
                               [] (char x) {return x == '\n' || x == '\r';}),
                mcopy.end());

    // Decipher the packet from the controller
    vector<string> commands;
    tokenize(mcopy, ":", commands);
    if(commands.size() != 2)
    {
        return;
    }

    // Get the parameters sent to the computer
    vector<string> parameters;
    tokenize(commands[1], ";", parameters);

    // Process the command
    if(commands[0] == "ALERT")
    {
        if(parameters.size() != 1)
            return;

        // Process various alerts (Killed, Reset, Ready, Timeout, Idle, Heartbeat)
        // Push out these events to the registered handlers
        lock_guard<mutex> lock(callbacksMutex);
        for_each(callbacks.begin(), callbacks.end(), [this, parameters] (request_handler_t handler)
        {
            handler(parameters[0]);
        });
    }
    else if(commands[0] == "ARM")
    {
        if(parameters.size() != 2)
            return;

        // Process the result of the arm command
        int code = atoi(parameters[1].c_str());
        {
            lock_guard<mutex> lock(requestsMutex);
            auto request = requests.find(make_pair(code, string("ARM")));
            if(request != requests.end())
            {
                request->second.set_value(parameters[0]);
                requests.erase(request);
            }
        }
    }
    else if(commands[0] == "DISARM")
    {
        if(parameters.size() != 2)
            return;

        // Process the result of the disarm command
    }
    else if(commands[0] == "ARMSTAT")
    {
        if(parameters.size() != 2)
            return;

        // Process the result of the armstat command
    }
    else if(commands[0] == "PING")
    {
        if(parameters.size() != 2)
            return;

        // Process the result of the ping command
    }
    else if(commands[0] == "VELOCITY")
    {
        if(parameters.size() != 2)
            return;

        // Process the result of the velocity command
    }
    else if(commands[0] == "STEER")
    {
        if(parameters.size() != 2)
            return;

        // Process the result of the steer command
    }

    // Push out these events to the registered handlers
    /*lock_guard<mutex> lock(callbacksMutex);
    for_each(callbacks.begin(), callbacks.end(), [this] (std::function<void (MotionController::State&)> handler)
    {
        handler(currentState);
    });*/
}

std::future<std::string> MotionController::RequestARM()
{
    // Push out a arming request command
    int code = rand() & 0xFFFF;
    lock_guard<mutex> lock(requestsMutex);
    auto r = requests.insert(make_pair(make_pair(code, string("ARM")), std::move(promise<string>())));

    // Push out the command
    stringstream stream;
    stream << "ARM:" << code << "\r\n";
    serialPort.Write(stream.str());

    // Return the future
    return r.first->second.get_future();
}

// Close the serial device
void MotionController::Close()
{
    SerialDevice::Close();
}

void MotionController::RegisterAlertHandler(request_handler_t handler)
{
    lock_guard<mutex> lock(callbacksMutex);
    callbacks.push_back(handler);
}
