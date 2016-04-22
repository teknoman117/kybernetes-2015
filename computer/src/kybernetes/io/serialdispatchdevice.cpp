#include <kybernetes/io/serialdispatchdevice.hpp>
#include <kybernetes/utility/pollhandler.hpp>

#include <algorithm>

using namespace std;

namespace kybernetes
{
    namespace io
    {
        SerialDispatchDevice::SerialDispatchDevice(const string& path, dispatch_queue_t queue_, uint32_t baudrate, SerialPort::DataBits dataBits, SerialPort::Parity parity, SerialPort::StopBits stopBits, function<void (bool)> callback)
            : port(path), queue(queue_)
        {

            serialPortLock = dispatch_semaphore_create(1);
            serialPort     = make_unique<SerialPort>(path, baudrate, dataBits, parity, stopBits, [this, callback] (SerialPort *device, SerialPort::Error error)
            {
                if(error != SerialPort::Success || !device->SetBlocking(false))
                {
                    dispatch_async(queue, ^{ callback(false); });
                    return;
                }

                device->Flush();

                // Register signal handler
                utility::PollHandler::Instance()->AttachHandler(device->GetHandle(), [this, device] ()
                {
                    // Wait for the serial port to become available
                    dispatch_semaphore_wait(serialPortLock, DISPATCH_TIME_FOREVER);

                    // Read all the available data from the serial port
                    vector<char>::size_type quantity = device->Available();
                    vector<char>::size_type position = buffer.size();
                    buffer.resize(position + quantity);
                    device->Read(buffer.data() + position, quantity);

                    // Search for ASCII lines in the data
                    const char *terminator = "\r\n";
                    vector<char>::iterator it;

                    while((it = std::search(buffer.begin(), buffer.end(), terminator, terminator+2)) != buffer.end())
                    {
                        // Construct a string from the section of the buffer
                        string message(buffer.begin(), it);
                        buffer.erase(buffer.begin(), it+2);
                        if(handler)
                        {
                            handler(message);
                        }
                    }

                    // Unlock the serial port
                    dispatch_semaphore_signal(serialPortLock);
                });

                // Success!
                dispatch_async(queue, ^{ callback(true); });
            });
        }

        SerialDispatchDevice::SerialDispatchDevice(const string& path, dispatch_queue_t queue, uint32_t baudrate, function<void (bool)> callback)
            : SerialDispatchDevice(path, queue, baudrate, SerialPort::DataBits8, SerialPort::ParityNone, SerialPort::StopBits1, callback)
        {
        }

        SerialDispatchDevice::~SerialDispatchDevice()
        {
            if(!serialPort->IsOpen())
                return;

            utility::PollHandler::Instance()->DetachHandler(serialPort->GetHandle());
        }

        void SerialDispatchDevice::SetHandler(const SerialDispatchDevice::handler_t& handler)
        {
            this->handler = move(handler);
        }

        // Write data to the device
        bool SerialDispatchDevice::Write(const string& line)
        {
            return serialPort->Write(line.data(), line.size()) == line.size();
        }
    }
}
