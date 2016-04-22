#ifndef __SERIAL_DISPATCH_DEVICE__
#define __SERIAL_DISPATCH_DEVICE__

#include <kybernetes/io/serialport.hpp>
#include <dispatch/dispatch.h>

#include <functional>
#include <string>
#include <vector>
#include <memory>

namespace kybernetes
{
    namespace io
    {
        // Wrapper around a serial port which gets data via libdispatch
        class SerialDispatchDevice
        {
        public:
            typedef std::function<void (const std::string& message)> handler_t;

        private:
            std::unique_ptr<SerialPort> serialPort;
            std::string                 port;
            dispatch_queue_t            queue;
            dispatch_semaphore_t        serialPortLock;
            handler_t                   handler;

            std::vector<char>           buffer;

        public:
            SerialDispatchDevice(const std::string& path, dispatch_queue_t queue, uint32_t baudrate, SerialPort::DataBits dataBits, SerialPort::Parity parity, SerialPort::StopBits stopBits, std::function<void (bool)> callback);
            SerialDispatchDevice(const std::string& path, dispatch_queue_t queue, uint32_t baudrate, std::function<void (bool)> callback);
            ~SerialDispatchDevice();

            void SetHandler(const handler_t& handler);

            bool Write(const std::string& line);
        };
    }
}

#endif
