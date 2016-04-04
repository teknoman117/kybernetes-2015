#include <kybernetes/io/serialdispatchdevice.hpp>
#include <kybernetes/utility/posixsignalhandler.hpp>

#include <iostream>
#include <algorithm>
//#include <chrono>

#include <fcntl.h>
#include <unistd.h>

using namespace std;
//using namespace std::chrono;

namespace kybernetes
{
    namespace io
    {
        SerialDispatchDevice::SerialDispatchDevice(const string& path, dispatch_queue_t queue_, uint32_t baudrate, SerialPort::DataBits dataBits, SerialPort::Parity parity, SerialPort::StopBits stopBits, function<void (bool)> callback)
            : port(path), queue(queue_), signalHandler(-1)
        {
            serialPort = make_unique<SerialPort>(path, baudrate, dataBits, parity, stopBits, [this, callback] (SerialPort *device, SerialPort::Error error)
            {
                if(error != SerialPort::Success)
                {
                    dispatch_async(queue, ^{ callback(false); });
                    return;
                }

                if(fcntl(device->GetHandle(), F_SETOWN, getpid() ) < 0 )
                {
                    dispatch_async(queue, ^{ callback(false); });
                    return;
                }

                if(fcntl(device->GetHandle(), F_SETFL, FASYNC ) < 0)
                {
                    dispatch_async(queue, ^{ callback(false); });
                    return;
                }
                device->Flush();

                // Register signal handler
                signalHandler = utility::PosixSignalHandler::Instance()->AttachHandler(SIGIO, [this, device] ()
                {
                    // Read all the available data from the serial port
                    vector<char>::size_type quantity = device->Available();
                    vector<char>::size_type position = buffer.size();
                    buffer.resize(position + quantity);
                    device->Read(buffer.data() + position, quantity);

                    // Search for ASCII lines in the data
                    const char *terminator = "\r\n";
                    vector<char>::iterator it;

                    //int n = 0;
                    while((it = std::search(buffer.begin(), buffer.end(), terminator, terminator+2)) != buffer.end())
                    {
                        // Construct a string from the section of the buffer
                        string message(buffer.begin(), it);
                        buffer.erase(buffer.begin(), it+2);
                        handler(message);
                        //n++;
                    }

                    //high_resolution_clock::time_point now = high_resolution_clock::now();
                    //duration<double> frameDeltaRaw = duration_cast<duration<double>>(now - previousTime);
                    //previousTime = now;

                    //cout << "[DEBUG " << port << "] ingested " << quantity << " bytes, dispatched " << n << " messages, " << buffer.size() << " bytes left.  elapsed since previous " << frameDeltaRaw.count() << endl;
                });

                // Success!
                dispatch_async(queue, ^{ callback(true); });
            });
            //previousTime = high_resolution_clock::now();
        }

        SerialDispatchDevice::SerialDispatchDevice(const string& path, dispatch_queue_t queue, uint32_t baudrate, function<void (bool)> callback)
            : SerialDispatchDevice(path, queue, baudrate, SerialPort::DataBits8, SerialPort::ParityNone, SerialPort::StopBits1, callback)
        {
        }

        SerialDispatchDevice::~SerialDispatchDevice()
        {
            if(!serialPort->IsOpen())
                return;

            utility::PosixSignalHandler::Instance()->DetachHandler(signalHandler);
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
