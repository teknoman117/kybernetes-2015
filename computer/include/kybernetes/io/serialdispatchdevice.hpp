#ifndef __SERIAL_DISPATCH_DEVICE__
#define __SERIAL_DISPATCH_DEVICE__

#include <kybernetes/io/serialport.hpp>
#include <dispatch/dispatch.h>

#include <functional>
#include <string>
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
            dispatch_io_t               channel;
            dispatch_queue_t            queue;
            handler_t                   handler;
            
            std::string                 buffer;

        public:
            SerialDispatchDevice(const std::string& path, dispatch_queue_t queue, uint32_t baudrate, SerialPort::DataBits dataBits, SerialPort::Parity parity, SerialPort::StopBits stopBits, std::function<void (int)> callback);
            SerialDispatchDevice(const std::string& path, dispatch_queue_t queue, uint32_t baudrate, std::function<void (int)> callback);
            ~SerialDispatchDevice();

            void SetHandler(handler_t&& handler);
            void SetDataSizeHints(size_t lowWater, size_t highWater);
        };
    }
}

#endif
