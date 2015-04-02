#include <iostream>
#include <string>

#include <unistd.h>

//#include <garmingpsd/gpscoordinate.hpp>
#include <SerialPort.h>

using namespace std;

// Verify that a packet from the GPS is valid
bool IsGPSSentenceValid(const string& sentence)
{
    if(sentence.size() != 57)
        return false;
    else if(sentence[0] != '@' || sentence[55] != '\r' || sentence[56] != '\n')
        return false;

    // Its probably valid
    return true;
}

int main (int argc, char** argv)
{
    // Port to use for the GPS
    auto   serialPortBaud = SerialPort::BAUD_9600;
    string serialPortPath = "/dev/ttyS0";

    // Parse command line options
    char option;
    while ((option = getopt (argc, argv, "p:k")) != -1)
    switch (option)
    {
        case 'k':
            break;
        case 'p':
            serialPortPath = optarg;
            break;
        case '?':
            if (optopt == 'p')
                cerr << "Option -" << optopt << " requires an argument." << endl;
            else if (isprint (optopt))
                cerr << "Unknown option \'-" << optopt << "\'." << endl;
            else
                cerr << "Unknown option character \'\\x" << hex << optopt << "\'." << endl;
            return 1;
        default:
            abort ();
    };
    //for (int index = optind; index < argc; index++)
    //    cout << "Non-option argument " << argv[index] << endl;

    // Open the serial port
    SerialPort serialPort(serialPortPath);
    try
    {
        serialPort.Open(serialPortBaud, SerialPort::CHAR_SIZE_8, SerialPort::PARITY_NONE, SerialPort::STOP_BITS_1, SerialPort::FLOW_CONTROL_NONE);
    } catch (std::exception &)
    {
        cerr << "Couldn't open " << serialPortPath << endl;
        return 1;
    }

    // Continually read data from the GPS
    for(;;)
    {
        string sentence = serialPort.ReadLine();
        if(IsGPSSentenceValid(sentence))
            cout << sentence;
    }

    return 0;
}
