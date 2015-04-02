#ifndef __GARMIN_GPS_HPP__
#define __GARMIN_GPS_HPP__

#include "serialdevice.hpp"
#include <cstdint>

namespace kybernetes
{
    class GarminGPS : SerialDevice
    {
    public:
        struct State
        {
            // Type of fix of a garmin gps
            typedef enum _fix_status_t : uint32_t
            {
                Invalid = 0,
                Simulated = 1,
                TwoDimentional = 2,
                ThreeDimentional = 3,
                Differential2D = 4,
                Differential3D = 5
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
        };
    private:
        volatile bool killWorker;
        std::thread worker;
        static bool IsValidGPSSentence(const std::string& sentence);

    public:
        GarminGPS(std::string path, dispatch_queue_t queue);

        void Open(void (^completionHandler)(bool, std::string error));
        void Close(void (^completionHandler)());
    };
}

#endif
