#ifndef __GARMIN_GPS_HPP__
#define __GARMIN_GPS_HPP__

#include <kybernetes/io/serialdispatchdevice.hpp>
#include <cstdint>
#include <memory>

namespace kybernetes
{
    namespace sensor
    {
        class GarminGPS
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

            std::function<void (GarminGPS::State&)> handler;
            std::unique_ptr<io::SerialDispatchDevice> device;

            void ReceiveMessageHandler(const std::string& message);

        public:
            GarminGPS(std::string path, dispatch_queue_t queue, const uint32_t baudrate, std::function<void (GarminGPS *, bool)> callback);

            void SetHandler(std::function<void (GarminGPS::State&)>&& handler);

        };
    }
}

#endif
