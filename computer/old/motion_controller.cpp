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

static const string commandNames[4] = {"ARM", "DISARM", "ARMSTAT", "PING"};
static const size_t commandNamesLength = 4;

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
    //cout << "got: " << mcopy << endl;

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

    // Is this a known "one response" command?
    else if(find(commandNames, commandNames + commandNamesLength, commands[0]) != commandNames + commandNamesLength)
    {
        if(parameters.size() != 2)
            return;

        // Process the result of the arm command
        int code = atoi(parameters[1].c_str());
        {
            lock_guard<mutex> lock(requestsMutex);
            auto request = requests.find(make_pair(code, commands[0]));
            if(request != requests.end())
            {
                request->second.set_value(parameters[0]);
                requests.erase(request);
            }
        }
    }



    // Push out these events to the registered handlers
    /*lock_guard<mutex> lock(callbacksMutex);
    for_each(callbacks.begin(), callbacks.end(), [this] (std::function<void (MotionController::State&)> handler)
    {
        handler(currentState);
    });*/
}

MotionController::request_future_t MotionController::RequestArm()
{
    // Push out a arming request command
    int code = rand() % 32;
    pair<std::map<std::pair<int, std::string>, std::promise<std::string> >::iterator, bool> r;
    {
        lock_guard<mutex> lock(requestsMutex);
        r = requests.insert(make_pair(make_pair(code, string("ARM")), std::move(promise<string>())));
    }

    // Push out the command
    stringstream stream;
    stream << "ARM:" << code << "\r\n";
    serialPort.Write(stream.str());

    // Return the future
    return r.first->second.get_future();
}

MotionController::request_future_t MotionController::RequestDisarm()
{
    // Push out a arming request command
    int code = rand() % 32;
    pair<std::map<std::pair<int, std::string>, std::promise<std::string> >::iterator, bool> r;
    {
        lock_guard<mutex> lock(requestsMutex);
        r = requests.insert(make_pair(make_pair(code, string("DISARM")), std::move(promise<string>())));
    }

    // Push out the command
    stringstream stream;
    stream << "DISARM:" << code << "\r\n";
    serialPort.Write(stream.str());

    // Return the future
    return r.first->second.get_future();
}

MotionController::request_future_t MotionController::RequestArmStatus()
{
    // Push out a arming request command
    int code = rand() % 32;
    pair<std::map<std::pair<int, std::string>, std::promise<std::string> >::iterator, bool> r;
    {
        lock_guard<mutex> lock(requestsMutex);
        r = requests.insert(make_pair(make_pair(code, string("ARMSTAT")), std::move(promise<string>())));
    }

    // Push out the command
    stringstream stream;
    stream << "ARMSTAT:" << code << "\r\n";
    serialPort.Write(stream.str());

    // Return the future
    return r.first->second.get_future();
}

MotionController::request_future_t MotionController::RequestPing()
{
    // Push out a arming request command
    int code = rand() % 32;
    pair<std::map<std::pair<int, std::string>, std::promise<std::string> >::iterator, bool> r;
    {
        lock_guard<mutex> lock(requestsMutex);
        r = requests.insert(make_pair(make_pair(code, string("PING")), std::move(promise<string>())));
    }

    // Push out the command
    stringstream stream;
    stream << "PING:" << code << "\r\n";
    serialPort.Write(stream.str());

    // Return the future
    return r.first->second.get_future();
}

void MotionController::SetVelocity(short velocity)
{
    // Push out the command
    stringstream stream;
    stream << "VELOCITY:" << velocity << ";42" << "\r\n";
    serialPort.Write(stream.str());
}

void MotionController::SetSteering(short steering)
{
    // Push out the command
    stringstream stream;
    stream << "STEER:" << steering << ";42" << "\r\n";
    serialPort.Write(stream.str());
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
