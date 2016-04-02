#ifndef __MOTION_CONTROLLER_HPP__
#define __MOTION_CONTROLLER_HPP__

#include <kybernetes/io/serialdispatchdevice.hpp>
#include <memory>

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

        private:
            std::unique_ptr<io::SerialDispatchDevice> device;
            dispatch_queue_t                          queue;

            std::function<void (const std::string&)>  alertHandler;

        public:
            MotionController(const std::string& path, dispatch_queue_t queue, uint32_t baudrate = 57600);
            ~MotionController();

            void RequestArm(std::function<void (bool)> handler);
            void RequestDisarm(std::function<void (bool)> handler);
            void RequestArmStatus(std::function<void (ArmingStatus)> handler);
            void RequestPing(std::function<void (bool)> handler);

        	void SetVelocity(short velocity);
            void SetSteering(short steering);

            void SetAlertHandler(std::function<void (const std::string&)>&& handler);
        };
    }
}

#endif
