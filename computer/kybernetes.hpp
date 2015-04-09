#ifndef  __KYBERNETES_HPP__
#define  __KYBERNETES_HPP__

#include <string>

#ifndef NS_ENUM
#define NS_ENUM(_type, _name) enum _name : _type _name; enum _name : _type
#endif

namespace kybernetes
{
    // Constants in Kybernetes applications
    namespace constants
    {
        const static std::string GPSPath = "/dev/kybernetes/gps";
        const static std::string CameraPath = "/dev/kybernetes/camera";
        const static std::string SensorControllerPath = "/dev/kybernetes/sensor_controller";
        const static std::string MotionControllerPath = "/dev/kybernetes/motion_controller";

        const static double DegToRad = 3.14159265359 / 180.0;
        const static double RadToDeg = 180.0 / 3.14159265359;
    }
}

#endif
