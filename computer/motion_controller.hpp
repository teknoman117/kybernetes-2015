#ifndef __MOTION_CONTROLLER_HPP__
#define __MOTION_CONTROLLER_HPP__

#include "serialdevice.hpp"
#include <map>
#include <future>

namespace kybernetes
{
    class MotionController : public SerialDevice
    {
    public:
        typedef std::function<void (std::string)> request_handler_t;
        typedef std::future<std::string>          request_future_t;
    private:
        std::mutex callbacksMutex;
        std::mutex requestsMutex;

        // Alert callbacks
        std::vector<request_handler_t> callbacks;
        std::map<std::pair<int, std::string>, std::promise<std::string> > requests;

    public:
        MotionController(std::string path, const SerialPort::BaudRate baudRate = SerialPort::BAUD_57600);
        ~MotionController();
        void processMessage(std::string& message);
        void Close();

        // Requests from the motion controller
        request_future_t RequestArm();
        request_future_t RequestDisarm();
        request_future_t RequestArmStatus();
        request_future_t RequestPing();
        request_future_t RequestSetVelocity(short velocity);
        request_future_t RequestSetSteering(short steering);

        // Register to receive motor controller alert handlers
        void RegisterAlertHandler(request_handler_t handler);
    };
}

#endif
