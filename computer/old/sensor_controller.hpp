#ifndef __SENSOR_CONTROLLER_HPP__
#define __SENSOR_CONTROLLER_HPP__

#include "serialdevice.hpp"

namespace kybernetes
{
    class SensorController : public SerialDevice
    {
    public:
        struct State
        {
            float sonar[3];
            double rotation[3];
            bool  bumper[2];

            // Initializing constructor
            State()
            {
                sonar[0] = sonar[1] = sonar[2] = 0.0f;
                rotation[0] = rotation[1] = rotation[2] = 0.0;
                bumper[0] = bumper[1] = false;
            }
        };
    private:
        std::mutex callbacksMutex;
        std::vector<std::function<void (SensorController::State&)> > callbacks;
        State currentState;

    public:
        SensorController(std::string path, const SerialPort::BaudRate baudRate = SerialPort::BAUD_57600);
        ~SensorController();
        void processMessage(std::string& message);
        void Close();

        void RegisterHandler(std::function<void (SensorController::State&)> handler);
    };
}

#endif
