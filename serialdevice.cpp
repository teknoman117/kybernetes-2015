#include "serialdevice.hpp"

using namespace kybernetes;

// Create a serial device, storing the path and a dispatch queue
SerialDevice::SerialDevice(std::string path, dispatch_queue_t queue)
    : serialPort(path), queue(queue)
{

}

// Open the serial device
bool SerialDevice::Open(const SerialPort::BaudRate baudRate,
                        const SerialPort::CharacterSize characterSize,
                        const SerialPort::Parity parityType,
                        const SerialPort::StopBits stopBits,
                        const SerialPort::FlowControl flowControl)
{
    // If the port is already open, close it first
    if(serialPort.IsOpen())
    {
        serialPort.Close();
    }

    // Open the serial port
    try
    {
        serialPort.Open(baudRate, characterSize, parityType, stopBits, flowControl);
    }
    catch (std::runtime_error e)
    {
        return false;
    }

    // Success
    return true;
}

// Close the serial device
void SerialDevice::Close()
{
    if(serialPort.IsOpen())
    {
        serialPort.Close();
    }
}
