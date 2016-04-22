#ifndef __TASK_HPP__
#define __TASK_HPP__

#include <kybernetes/controller/motion_controller.hpp>
#include <kybernetes/controller/sensor_controller.hpp>
#include <kybernetes/sensor/garmingps.hpp>
#include <kybernetes/sensor/v4l2camera.hpp>
#include <functional>

struct Task
{
	// Robot state data
	std::function<void (Task&)>               taskCompletedCallback;
	bool                                      taskEnabled;

	kybernetes::controller::MotionController *motionController;
	kybernetes::controller::SensorController *sensorController;
	kybernetes::sensor::GarminGPS            *gps;
	kybernetes::sensor::V4L2Camera           *camera;

	// Required functions
	virtual void HandleSonarMessage(const kybernetes::controller::SensorController::SonarState&) = 0;
	virtual void HandleBumperMessage(const kybernetes::controller::SensorController::BumperState&) = 0;
	virtual void HandleIMUMessage(const kybernetes::controller::SensorController::IMUState&) = 0;
	virtual void HandleGPSMessage(const kybernetes::sensor::GarminGPS::State&) = 0;
	virtual void HandleMotionControllerAlert(const kybernetes::controller::MotionController::Alert&) = 0;
	virtual void HandleCameraFrame(const kybernetes::sensor::V4L2Camera::FrameBuffer&) = 0;

	Task()
	{
		taskEnabled = false;
	}

	virtual ~Task()
	{
	}

	void SetTaskCompletedCallback(std::function<void (Task&)>&& callback)
	{
		taskCompletedCallback = std::move(callback);
	}
};

#endif
