#include <fstream>
#include "task.hpp"

struct MoveToTask : public Task
{
	kybernetes::sensor::GarminGPS::State     waypoint;
	short                                    highspeed;
	short                                    lowspeed;
	bool                                     armed;
	bool                                     avoiding;
	int                                      debug;
	bool                                     flags[3];

    double                                   headingToObjective;
    double                                   distanceToObjective;

	MoveToTask(std::ifstream& manifest);

	void HandleSonarMessage(const kybernetes::controller::SensorController::SonarState&);
	void HandleBumperMessage(const kybernetes::controller::SensorController::BumperState&);
	void HandleIMUMessage(const kybernetes::controller::SensorController::IMUState&);
	void HandleGPSMessage(const kybernetes::sensor::GarminGPS::State&);
	void HandleMotionControllerAlert(const kybernetes::controller::MotionController::Alert&);
	void HandleCameraFrame(const kybernetes::sensor::V4L2Camera::FrameBuffer&);
};
