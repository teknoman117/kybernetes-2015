#ifndef __GARMIN_GPS_HPP__
#define __GARMIN_GPS_HPP__

#include "serialdevice.hpp"
#include <cstdint>

namespace kybernetes
{
    class GarminGPS : public SerialDevice
    {
    public:
        struct State
        {
            // Type of fix of a garmin gps
            typedef enum _fix_status_t : char
            {
                Invalid = '_',
                Simulated = 'S',
                TwoDimentional = 'g',
                ThreeDimentional = 'G',
                Differential2D = 'd',
                Differential3D = 'D'
            } FixStatus ;

            // Basic coordinate
            int32_t   timestamp;
            double    latitude;
            double    longitude;
            double    altitude;

            // Extended options
            FixStatus status;
            float     precision;
            float     velocity[3];

            // Some utility functions
            double DistanceTo(struct State& state);
            double HeadingTo(struct State& state);
            State();
        };
    private:
        static bool IsValidGPSSentence(const std::string& sentence);
        std::mutex callbacksMutex;
        std::vector<std::function<void (GarminGPS::State&)> > callbacks;

    public:
        GarminGPS(std::string path, const SerialPort::BaudRate baudRate = SerialPort::BAUD_9600);
        ~GarminGPS();
        void processMessage(std::string& message);
        void Close();

        void RegisterHandler(std::function<void (GarminGPS::State&)> handler);
    };
}

#endif
