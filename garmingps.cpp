#include "garmingps.hpp"
#include <Block.h>

using namespace kybernetes;
using namespace std;

GarminGPS::GarminGPS(std::string path, dispatch_queue_t queue)
    : SerialDevice(path, queue), killWorker(false)
{

}

void GarminGPS::Open(void (^completionHandler)(bool, std::string error))
{
    // Open the serial port
    if(!SerialDevice::Open(SerialPort::BAUD_9600, SerialPort::CHAR_SIZE_8, SerialPort::PARITY_NONE, SerialPort::STOP_BITS_1, SerialPort::FLOW_CONTROL_NONE))
    {
        completionHandler(false, "Failed to open serial device");
        return;
    }

    cout << "Got to this point" << endl;

    // Dispatch a thread to handle GPS operations
    //void (^completionHandlerInternal)(bool, std::string) = Block_copy(completionHandler);
    worker = std::thread([this/*, completionHandlerInternal*/] ()
    {
        // Initialization
        std::string sentence;
        bool        validSentence = false;
        while(!validSentence)
        {
            try
            {
                sentence = this->serialPort.ReadLine();
            }
            catch (SerialPort::ReadTimeout timeout)
            {
                // The read has timed out, fail out.
                /*dispatch_async(this->queue, ^
                {
                    completionHandlerInternal(false, "GPS Device Failed to Respond");
                });*/
                SerialDevice::Close();
                return;
            }
            validSentence = GarminGPS::IsValidGPSSentence(sentence);
        }

        // The read has timed out, fail out.
        /*dispatch_async(this->queue, ^
        {
            completionHandlerInternal(true, "Success");
        });*/

        // We are synchronized
        while(!this->killWorker)
        {
            // Get a sentence from the GPS
            try
            {
                sentence = this->serialPort.ReadLine();
            }
            catch (SerialPort::ReadTimeout timeout)
            {
                continue;
            }

            // Process it
            if(GarminGPS::IsValidGPSSentence(sentence))
            {
                //dispatch_async(this->queue, ^
                //{
                    std::cout << sentence;
                //});
            }
        }

        // Close the serial port
        SerialDevice::Close();
    });
}

void GarminGPS::Close(void (^completionHandler)())
{
    killWorker = true;
    worker.join();
}

// Verify that a packet from the GPS is valid
bool GarminGPS::IsValidGPSSentence(const std::string& sentence)
{
    if(sentence.size() != 57)
        return false;
    else if(sentence[0] != '@' || sentence[55] != '\r' || sentence[56] != '\n')
        return false;

    // Its probably valid
    return true;
}
