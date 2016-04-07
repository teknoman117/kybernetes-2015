#include <kybernetes/io/serialport.hpp>

#include <unistd.h>
#include <fcntl.h>
#include <asm/termios.h>
#include <sys/ioctl.h>

using namespace std;

// Redefine the things we need
extern int tcflush (int __fd, int __queue_selector) __THROW;

namespace kybernetes
{
    namespace io
    {
        SerialPort::SerialPort(const string& path, uint32_t baudrate, DataBits dataBits, Parity parity, StopBits stopBits, const function<void (SerialPort *, Error)>& callback)
            : baudrate(baudrate), dataBits(dataBits), parity(parity), stopBits(stopBits), descriptor(-1)
        {
            // Attempt to open the serial port
            descriptor = open(path.c_str(), O_RDWR | O_NONBLOCK);
            if(descriptor < 0)
            {
                // An error has occurred
                callback(nullptr, SerialPort::ErrorOpenFailed);
                return;
            }

            // Get the active serial port configuration
            struct termios2 configuration;
            ioctl(descriptor, TCGETS2, &configuration);

            // Set the baudrate
            configuration.c_cflag &= ~CBAUD;
            configuration.c_cflag |= BOTHER;
            configuration.c_cflag |= (CREAD | CLOCAL);
            configuration.c_ispeed = baudrate;
            configuration.c_ospeed = baudrate;

            // Set the databits
            configuration.c_cflag &= ~CSIZE;
            switch(dataBits)
            {
                case SerialPort::DataBits5:
                    configuration.c_cflag |= CS5;
                    break;
                case SerialPort::DataBits6:
                    configuration.c_cflag |= CS6;
                    break;
                case SerialPort::DataBits7:
                    configuration.c_cflag |= CS7;
                    break;
                case SerialPort::DataBits8:
                    configuration.c_cflag |= CS8;
                    break;
            };

            // Set the parity
            configuration.c_cflag &= ~PARENB;
            switch(parity)
            {
                case SerialPort::ParityEven:
                    configuration.c_cflag |= PARENB;
                    configuration.c_cflag &= ~PARODD;
                    break;
                case SerialPort::ParityOdd:
                    configuration.c_cflag |= PARENB;
                    configuration.c_cflag |= PARODD;
                    break;
                case SerialPort::ParityNone:
                    break;
            };

            // Set the stop bits
            configuration.c_cflag = ((stopBits == SerialPort::StopBits1) ? configuration.c_cflag & ~CSTOPB : configuration.c_cflag | CSTOPB );

            // Ensure some bugs with Arduinos are solved
            configuration.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            configuration.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);
            configuration.c_oflag &= ~(OPOST | ONLCR | OCRNL);

            // Write out configuration
            ioctl(descriptor, TCSETS2, &configuration);

            // Success!
            callback(this, SerialPort::Success);
        }

        SerialPort::SerialPort(const string& path, uint32_t baudrate, const function<void (SerialPort *, Error)>& callback)
            : SerialPort(path, baudrate, SerialPort::DataBits8, SerialPort::ParityNone, SerialPort::StopBits1, callback)
        {
        }

        SerialPort::~SerialPort()
        {
            if(IsOpen())
                Close();
        }

        // Get the serial port descriptor
        int SerialPort::GetHandle() const
        {
            return descriptor;
        }

        bool SerialPort::IsOpen() const
        {
            return (descriptor >= 0);
        }

        // Flow control
        bool SerialPort::SetBlocking(bool blocking)
        {
            int flags;
            int ret;

            // Fetch the current device flags
            ret = fcntl (descriptor, F_GETFL, 0);
            if (ret == -1)
            {
                return false;
            }

            // Enable or disable blocking
            flags = blocking ? (ret & ~O_NONBLOCK) : (ret | O_NONBLOCK);

            // Set the flags
            ret = fcntl (descriptor, F_SETFL, flags);
            if (ret == -1)
            {
                return false;
            }

            return true;
        }

        // Get the number of bytes in the input buffer
        int SerialPort::Available() const
        {
            if(!IsOpen())
                return 0;

            int available = 0;
            ioctl(descriptor, FIONREAD, &available);
            return available;
        }

        // Flush all data in the input buffer
        void SerialPort::Flush()
        {
            if(IsOpen())
            {
                // Query the number of bytes
                size_t n = Available();
                uint8_t *data = new uint8_t[n];
                read(descriptor, data, n);
                delete[] data;
            }
        }

        // Close the port
        void SerialPort::Close()
        {
            // Close the serialport
            if(IsOpen())
            {
                close(descriptor);
                descriptor = -1;
            }
        }

        ssize_t SerialPort::Write(const void *data, size_t n)
        {
            return write(descriptor, data, n);
        }

        ssize_t SerialPort::Read(void *data, size_t n)
        {
            return read(descriptor, data, n);
        }
    }
}
