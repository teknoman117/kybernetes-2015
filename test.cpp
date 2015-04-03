#include <iostream>
#include "sensor_controller.hpp"

using namespace std;
using namespace kybernetes;

int main (int argc, char** argv)
{
    // Get the main dispatch queue
    SensorController controller("/dev/ttyACM0");
    controller.RegisterHandler([] (SensorController::State& state)
    {
        //std::cout << "Sonars = " << state.sonar[0] << ", " << state.sonar[1] << ", " << state.sonar[2] << std::endl;
        std::cout << "Bumpers = " << state.bumper[0] << ", " << state.bumper[1] << std::endl;
        //std::cout << "IMU = " << state.rotation[0] << ", " << state.rotation[1] << ", " << state.rotation[2] << std::endl;
    });
    controller.Join();

    return 0;
}
