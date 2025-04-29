#ifndef __SERIALPORT_HPP__
#define __SERIALPORT_HPP__

#include <cstdint>
#include <functional>
#include <string>

namespace kybernetes
{
    namespace io
    {
        class SerialPort
        {
        public:
            // Possible stop bits configurations
            typedef enum : uint32_t
            {
                StopBits1,
                StopBits2,
            } StopBits;

            // Possible parity configurations
            typedef enum : uint32_t
            {
                ParityNone,
                ParityOdd,
                ParityEven,
            } Parity;

            // Possible data bits configurations
            typedef enum : uint32_t
            {
                DataBits5,
                DataBits6,
                DataBits7,
                DataBits8,
            } DataBits;

            // Possible errors encountered
            typedef enum : uint32_t
            {
                Success,
                ErrorOpenFailed,
            } Error;

        private:
            // Serialport configuration
            uint32_t    baudrate;
            DataBits    dataBits;
            Parity      parity;
            StopBits    stopBits;

            // Serialport state
            int         descriptor;

        public:
            SerialPort(const std::string& path, uint32_t baudrate, const std::function<void (SerialPort *, Error)>& callback);
            SerialPort(const std::string& path, uint32_t baudrate, DataBits dataBits, Parity parity, StopBits stopBits, const std::function<void (SerialPort *, Error)>& callback);
            ~SerialPort();

            // Get the handle of the serial port
            int GetHandle() const;

            // Is the port open
            bool IsOpen() const;

            // The amount of bytes in the read buffer
            int Available() const;

            // Flush all pending data on the serial port
            void Flush();

            // Close the serialport
            void Close();

            // Set if the port blocks
            bool SetBlocking(bool blocking = true);

            // Data read/write operations
            ssize_t Write(const void *data, size_t n);
            ssize_t Read(void *data, size_t n);
        };
    }
};

#endif
