#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <memory>

#include <kybernetes/constants/constants.hpp>
#include <kybernetes/utility/application.hpp>
#include <kybernetes/sensor/uvccamera.hpp>
#include <kybernetes/cv/cv.hpp>

using namespace std;

using namespace kybernetes::utility;
using namespace kybernetes::sensor;
using namespace kybernetes::constants;

class TestApplication : public Application::Delegate
{
	//uint8_t testPixels[32];
	uint8_t testPixelResults[640*480];

    std::unique_ptr<UVCCamera> camera;

public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {
    	camera = make_unique<UVCCamera>(CameraPath, 640, 480, V4L2_PIX_FMT_YUYV);
        camera->ptz_reset();

        thread([this] () 
        {
            struct v4l2_buffer  buffer;
            void               *data;
            size_t              length;

            while(true)
            {
                camera->capture_buffer(&buffer, &data, &length);
                kybernetes::cv::yuv422_bithreshold(data, (void *) &testPixelResults[0], 640, 480, 64, 64, 64, 128+64, 128+64, 128+64);
                camera->release_buffer(&buffer);

                cout << "Frame processed" << endl;
            }

        }).detach();
    }

    void ApplicationWillTerminate()
    {
    }
};

int main (int argc, char** argv)
{
    Application::Run<TestApplication>(argc, argv);
    return 0;
}

