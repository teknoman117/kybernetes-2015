#include "serialdevice.hpp"

using namespace kybernetes;
using namespace std;

// Create a serial device
SerialDevice::SerialDevice(string path, const SerialPort::BaudRate baudRate)
    : serialPort(path), killWorker(false)
{
    // Open the serial port
    serialPort.Open(baudRate, SerialPort::CHAR_SIZE_8, SerialPort::PARITY_NONE, SerialPort::STOP_BITS_1, SerialPort::FLOW_CONTROL_NONE);

    // Fire up a thread to process sentences from the device
    worker = thread([this] ()
    {
        bool localKillWorker = false;
        while(!localKillWorker)
        {
            try
            {
                string message = serialPort.ReadLine(2500);
                processMessage(message);
            }
            catch (SerialPort::ReadTimeout)
            {
                // This is probably a bad time (tm)
            }

            // Check if we should kill off the worker thread
            {
                lock_guard<mutex> lock(killWorkerMutex);
                localKillWorker = killWorker;
            }
        }
        serialPort.Close();
    });
}

// Blocks until the worker is killed off
void SerialDevice::Close()
{
    // Kill the worker
    {
        lock_guard<mutex> lock(killWorkerMutex);

        // If the worker is running
        if(killWorker)
            return;

        killWorker = true;
    }
    worker.join();
}

void SerialDevice::Join()
{
    {
        lock_guard<mutex> lock(killWorkerMutex);

        // If the worker is running
        if(killWorker)
            return;
    }
    worker.join();
}
