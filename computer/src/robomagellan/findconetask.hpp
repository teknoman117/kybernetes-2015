#include <fstream>
#include "task.hpp"

struct FindConeTask : public Task
{
	uint8_t                                  upperY;
	uint8_t                                  lowerY;
	uint8_t                                  upperU;
	uint8_t                                  lowerU;
	uint8_t                                  upperV;
	uint8_t                                  lowerV;
	
	short                                    highspeed;
	short                                    lowspeed;
	bool                                     armed;

	FindConeTask(std::ifstream& manifest);

	void HandleSonarMessage(const kybernetes::controller::SensorController::SonarState&);
	void HandleBumperMessage(const kybernetes::controller::SensorController::BumperState&);
	void HandleIMUMessage(const kybernetes::controller::SensorController::IMUState&);
	void HandleGPSMessage(const kybernetes::sensor::GarminGPS::State&);
	void HandleMotionControllerAlert(const kybernetes::controller::MotionController::Alert&);
	void HandleCameraFrame(const kybernetes::sensor::V4L2Camera::FrameBuffer&);
};
