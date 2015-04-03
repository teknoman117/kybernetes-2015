#ifndef  __SERIAL_DEVICE_HPP__
#define  __SERIAL_DEVICE_HPP__

#include <SerialPort.h>

#include <string>
#include <thread>
#include <mutex>

namespace kybernetes
{
    class SerialDevice
    {
    public:
        void Join();

    protected:
        SerialPort    serialPort;

        // Kill worker signal
        volatile bool killWorker;
        std::mutex    killWorkerMutex;
        std::thread   worker;

        SerialDevice(std::string path, const SerialPort::BaudRate baudRate);
        virtual void processMessage(std::string& message) = 0;
        void Close();
    };
}

#endif
