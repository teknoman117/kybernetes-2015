#ifndef __SENSOR_CONTROLLER_HPP__
#define __SENSOR_CONTROLLER_HPP__

#include <kybernetes/io/serialdispatchdevice.hpp>
#include <memory>

namespace kybernetes
{
    namespace controller
    {
        class SensorController
        {
        public:
            typedef std::function<void (bool)> SuccessCallback;

            // Sonar state packet
            struct SonarState
            {
                typedef enum : unsigned char
                {
                    CenterLeft = 0,
                    Center = 1,
                    CenterRight = 2,
                } SonarName;

                float distance[3];

                float operator[] (unsigned char i) const
                {
                    return distance[i];
                }

                SonarState()
                {
                    distance[0] = distance[1] = distance[2] = 0;
                }
            };

            // IMU state object
            struct IMUState
            {
                typedef enum : unsigned char
                {
                    Roll = 0,
                    Pitch = 1,
                    Yaw = 2,
                } AngleName;

                // Euler angles of the orientation
                float angles[3];

                float operator[] (unsigned char i) const
                {
                    return angles[i];
                }

                IMUState()
                {
                    angles[0] = angles[1] = angles[2] = 0;
                }
            };

            // Bumper state object
            struct BumperState
            {
                typedef enum : unsigned char
                {
                    Left = 0,
                    Right = 1,
                } BumperName;

                bool value[2];

                float operator[] (unsigned char i) const
                {
                    return value[i];
                }

                // get whether the bumper is pressed at all
                bool IsPressed() const
                {
                    return value[0] || value[1];
                }

                BumperState()
                {
                    value[0] = value[1] = false;
                }
            };

        private:
            std::unique_ptr<io::SerialDispatchDevice> device;
            dispatch_queue_t                          queue;
            SuccessCallback                           readyCallback;

            std::function<void (SensorController::SonarState&)>  sonarCallback;
            std::function<void (SensorController::IMUState&)>    imuCallback;
            std::function<void (SensorController::BumperState&)> bumperCallback;

            void ReceiveMessageHandler(const std::string& message);

        public:
            SensorController(std::string path, dispatch_queue_t queue, const uint32_t baudrate, SuccessCallback&& handler);

            void SetSonarHandler(std::function<void (SensorController::SonarState&)>&& handler);
            void SetIMUHandler(std::function<void (SensorController::IMUState&)>&& handler);
            void SetBumperHandler(std::function<void (SensorController::BumperState&)>&& handler);
        };
    }
}

#endif
