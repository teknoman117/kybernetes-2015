#ifndef __MOTION_CONTROLLER_HPP__
#define __MOTION_CONTROLLER_HPP__

#include <kybernetes/io/serialdispatchdevice.hpp>
#include <memory>
#include <map>
#include <mutex>
#include <atomic>

namespace kybernetes
{
    namespace controller
    {
        class MotionController
        {
        public:
            typedef enum : uint8_t
            {
                ArmingStatusArmed     = 0,
                ArmingStatusDisarming = 1,
                ArmingStatusIdle      = 2,
                ArmingStatusKilled    = 3,
            } ArmingStatus;

            typedef enum : uint8_t
            {
                AlertStatusArmed     = 0,
                AlertStatusDisarming = 1,
                AlertStatusIdle      = 2,
                AlertStatusKilled    = 3,
                AlertHeartbeat       = 4,
                AlertTimeout         = 5,
                AlertReset           = 6,
                AlertReady           = 7,
                AlertUnknown         = 8,
            } Alert;

            typedef std::function<void (const std::string&)> StringCallback;
            typedef std::function<void (bool)>               SuccessCallback;
            typedef std::function<void (Alert)>              AlertCallback;
            typedef std::function<void (ArmingStatus)>       ArmingStatusCallback;

        private:
            std::unique_ptr<io::SerialDispatchDevice> device;
            dispatch_queue_t                          queue;
            std::atomic<uint8_t>                      index;

            AlertCallback                             alertHandler;
            std::map<uint8_t, SuccessCallback>        requestArmCallbacks;
            std::map<uint8_t, SuccessCallback>        requestDisarmCallbacks;
            std::map<uint8_t, SuccessCallback>        requestPingCallbacks;
            std::map<uint8_t, ArmingStatusCallback>   requestArmStatusCallbacks;
            std::map<uint8_t, SuccessCallback>        setVelocityCallbacks;
            std::map<uint8_t, SuccessCallback>        setSteeringCallbacks;

            std::mutex                                requestArmCallbacksMutex;
            std::mutex                                requestDisarmCallbacksMutex;
            std::mutex                                requestPingCallbacksMutex;
            std::mutex                                requestArmStatusCallbacksMutex;
            std::mutex                                setVelocityCallbacksMutex;
            std::mutex                                setSteeringCallbacksMutex;

        public:
            MotionController(const std::string& path, dispatch_queue_t queue, uint32_t baudrate = 57600);
            ~MotionController();

            void RequestArm(const SuccessCallback& handler);
            void RequestDisarm(const SuccessCallback& handler);
            void RequestPing(const SuccessCallback& handler);
            void RequestArmStatus(const ArmingStatusCallback& handler);

            void SetVelocity(short velocity, const SuccessCallback& handler);
            void SetSteering(short steering, const SuccessCallback& handler);

            void SetAlertHandler(const AlertCallback& handler);
            static std::string GetStringRepresentation(Alert alert);
        };
    }
}

#endif
