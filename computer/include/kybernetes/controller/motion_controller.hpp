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
                AlertUnknown         = 6,
            } Alert;

            typedef std::function<void (const std::string&)>              StringCallback;
            typedef std::function<void (bool)>                            SuccessCallback;
            typedef std::function<void (Alert)>                           AlertCallback;
            typedef std::function<void (ArmingStatus)>                    ArmingStatusCallback;
            typedef std::function<void (const std::vector<std::string>&)> DebugCallback;

        private:
            std::unique_ptr<io::SerialDispatchDevice> device;
            dispatch_queue_t                          queue;
            std::atomic<uint8_t>                      index;

            AlertCallback                             alertHandler;
            DebugCallback                             debugHandler;
            SuccessCallback                           readyHandler;

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

            void ReceiveMessageHandler(const std::string& message);

        public:
            MotionController(const std::string& path, dispatch_queue_t queue, uint32_t baudrate, SuccessCallback&& handler);
            ~MotionController();

            void RequestArm(SuccessCallback&& handler);
            void RequestDisarm(SuccessCallback&& handler);
            void RequestPing(SuccessCallback&& handler);
            void RequestArmStatus(ArmingStatusCallback&& handler);

            void SetVelocity(short velocity, SuccessCallback&& handler);
            void SetSteering(short steering, SuccessCallback&& handler);
            void SetPID(float kp, float ki, float kd);
            void SetDebug(bool debug = true);

            void SetAlertHandler(AlertCallback&& handler);
            void SetDebugHandler(DebugCallback&& handler);

            static std::string GetStringRepresentation(Alert alert);
        };
    }
}

#endif
