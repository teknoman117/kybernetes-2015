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

    private:
        std::mutex callbacksMutex;
        std::mutex requestsMutex;

        // Alert callbacks
        std::vector<request_handler_t> callbacks;
        std::map<std::pair<int, std::string>, std::promise<std::string> > requests;

        //State currentState;

    public:
        MotionController(std::string path, const SerialPort::BaudRate baudRate = SerialPort::BAUD_57600);
        ~MotionController();
        void processMessage(std::string& message);
        void Close();

        std::future<std::string> RequestARM();

        void RegisterAlertHandler(request_handler_t handler);
    };
}

#endif
