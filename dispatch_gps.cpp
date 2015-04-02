#include <iostream>
#include <string>
#include <cstring>
#include <cerrno>
#include <vector>

#include <unistd.h>
#include <termios.h>
#include <linux/input.h>
#include <sys/ioctl.h>

#include <dispatch/dispatch.h>

using namespace std;

bool IsValidGPSSentence(const std::string& sentence)
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
    // Open the joystick
    int fd = open("/dev/kybernetes/gps", O_RDONLY);
    if(fd < 0)
    {
        cerr << "Unable to open GPS" << endl;
        return 1;
    }

    // Configure the serial port for GPS settings (9600 8N1)
    struct termios settings;
    tcgetattr(fd, &settings);

    // Set the baudrate
    cfsetispeed(&settings, B9600);
    cfsetospeed(&settings, B9600);

    // Set the port flow options
    settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
    settings.c_cflag |= (CLOCAL | CREAD | CS8);
    settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    settings.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
    settings.c_oflag &= ~(OPOST | ONLCR | OCRNL);

    // Set the port settings
    tcsetattr(fd, TCSANOW, &settings);

    // Flush the serial port
    tcflush(fd, TCIOFLUSH);

    // Locate the high priority queue
    //dispatch_queue_t queue = dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH,0);
    dispatch_queue_t queue = dispatch_get_main_queue();

    // Create a libdispatch io channel for the joystick
    dispatch_io_t serialport = dispatch_io_create(DISPATCH_IO_STREAM, fd, queue, ^(int error)
    {
        if(error)
        {
            cerr << "Got an error from GPS: " << error << " (" << strerror(error) << ")" << endl;
        }
    });

    // The GPS sentence is always 57 bytes long
    dispatch_io_set_low_water(serialport, 1);
    dispatch_io_set_high_water(serialport, 57);


    // Establish a read handler
    __block string buffer;
    dispatch_io_read(serialport, 0, -1, queue, ^(bool done, dispatch_data_t data, int error)
    {
        if(data)
        {
            dispatch_data_apply(data, ^(dispatch_data_t, size_t, const void *rawdata, size_t size)
            {
                buffer.append(static_cast<const char *>(rawdata), size);
                return true;
            });

            // See if we can find the delimeter
            size_t delimiter = buffer.find("\r\n");
            if(delimiter != string::npos)
            {
                string packet(buffer.begin(), buffer.begin() + delimiter + 2);
                cout << (IsValidGPSSentence(packet) ? "Valid: " : " Invalid: ") << packet;
                buffer.erase(buffer.begin(), buffer.begin() + delimiter + 2);
            }
        }
    });

    dispatch_main();

    return 0;
}
