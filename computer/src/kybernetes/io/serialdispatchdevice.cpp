#include <kybernetes/io/serialdispatchdevice.hpp>
#include <errno.h>

using namespace std;

namespace kybernetes
{
    namespace io
    {
        SerialDispatchDevice::SerialDispatchDevice(const string& path, dispatch_queue_t queue, uint32_t baudrate, SerialPort::DataBits dataBits, SerialPort::Parity parity, SerialPort::StopBits stopBits, function<void (int)> callback)
            : queue(queue)
        {
            // Open the serial port
            serialPort = new SerialPort(path, baudrate, dataBits, parity, stopBits, [&queue, &callback] (SerialPort::Error error)
            {
                // If the port failed to open
                if(error != SerialPort::Success)
                {
                    dispatch_async(queue, ^
                    {
                        callback(-EPERM);
                    });
                }
            });
            serialPort->Flush();

            // Create the io channel for the serial port
            channel = dispatch_io_create(DISPATCH_IO_STREAM, serialPort->GetHandle(), queue, ^(int error)
            {
                // If the channel failed to open
                if(error)
                {
                    callback(error);
                }
            });

            // Don't bug user unless we've received at least a js_Event object
            dispatch_io_set_low_water(channel, 1);
            dispatch_io_set_high_water(channel, 57);

            // Establish a read handler
            dispatch_io_read(channel, 0, SIZE_MAX, queue, ^(bool done, dispatch_data_t data, int error)
            {
                // Did we successfully get data?
                if(data)
                {
                    // Add any received data onto the buffer
                    dispatch_data_apply(data, ^(dispatch_data_t, size_t, const void *location, size_t size)
                    {
                        buffer.append((const char*) location, size);
                        return true;
                    });

                    // Attempt to find the end of the data
                    size_t p = buffer.find("\r\n");
                    if(p != string::npos)
                    {
                        // Get the string up to this point
                        string message = buffer.substr(0, p+2);
                        buffer.erase(0, p+2);

                        if(handler)
                        {
                            handler(message);
                        }
                    }
                }
            });
        }

        SerialDispatchDevice::SerialDispatchDevice(const string& path, dispatch_queue_t queue, uint32_t baudrate, function<void (int)> callback)
            : SerialDispatchDevice(path, queue, baudrate, SerialPort::DataBits8, SerialPort::ParityNone, SerialPort::StopBits1, callback)
        {
        }

        SerialDispatchDevice::~SerialDispatchDevice()
        {
            // stop reading
            if(serialPort->IsOpen())
            {
                dispatch_io_close(channel, DISPATCH_IO_STOP);
            }
            delete serialPort;
        }

        void SerialDispatchDevice::SetHandler(SerialDispatchDevice::handler_t handler)
        {
            this->handler = handler;
        }
    }
}
