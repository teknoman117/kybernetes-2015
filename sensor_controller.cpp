#include "sensor_controller.hpp"
#include "kybernetes.hpp"
#include "utility.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>

using namespace kybernetes;
using namespace std;

// Open the GPS
SensorController::SensorController(std::string path, const SerialPort::BaudRate baudRate)
    : SerialDevice(path, baudRate)
{

}

SensorController::~SensorController()
{
    this->Close();
}

// Process a message received from the GPS
void SensorController::processMessage(std::string& message)
{
    // Decipher the packet from the controller
    vector<string> commands;
    tokenize(message, ":", commands);
    if(commands.size() != 2)
    {
        return;
    }

    // Get the parameters sent to the computer
    vector<string> parameters;
    tokenize(commands[1], ";", parameters);

    // Process the command
    if(commands[0] == "IMU")
    {
        if(parameters.size() != 3)
            return;
        for(int i = 0; i < 3; i++)
            currentState.rotation[i] = atof(parameters[i].c_str());
    }
    else if(commands[0] == "SONAR")
    {
        if(parameters.size() != 3)
            return;
        for(int i = 0; i < 3; i++)
            currentState.sonar[i] = atof(parameters[i].c_str());
    }
    else if(commands[0] == "BUMPER")
    {
        if(parameters.size() != 2)
            return;
        for(int i = 0; i < 2; i++)
            currentState.bumper[i] = (atoi(parameters[i].c_str()) == 1);
    }

    // Push out these events to the registered handlers
    lock_guard<mutex> lock(callbacksMutex);
    for_each(callbacks.begin(), callbacks.end(), [this] (std::function<void (SensorController::State&)> handler)
    {
        handler(currentState);
    });
}

// Close the serial device
void SensorController::Close()
{
    SerialDevice::Close();
}

void SensorController::RegisterHandler(std::function<void (SensorController::State& state)> handler)
{
    lock_guard<mutex> lock(callbacksMutex);
    callbacks.push_back(handler);
}
