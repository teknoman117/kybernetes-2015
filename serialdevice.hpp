#ifndef  __SERIAL_DEVICE_HPP__
#define  __SERIAL_DEVICE_HPP__

#include <SerialPort.h>
#include <dispatch/dispatch.h>

#include <iostream>
#include <string>
#include <thread>
#include <functional>

namespace kybernetes
{
    class SerialDevice
    {
    protected:
        dispatch_queue_t queue;
        SerialPort       serialPort;

        // Create a serial device, storing the path and a dispatch queue
        SerialDevice(std::string path, dispatch_queue_t queue);

        // Open the serial device
        virtual bool Open(const SerialPort::BaudRate baudRate,
                          const SerialPort::CharacterSize characterSize,
                          const SerialPort::Parity parityType,
                          const SerialPort::StopBits stopBits,
                          const SerialPort::FlowControl flowControl);

        // Close the serial device
        virtual void Close();
    };
}

#endif
