#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <memory>
#include <algorithm>
#include <cinttypes>
#include <kybernetes/constants/constants.hpp>
#include <kybernetes/utility/application.hpp>
#include <kybernetes/utility/pollhandler.hpp>
#include <kybernetes/sensor/uvccamera.hpp>
#include <kybernetes/cv/cv.hpp>

#include <kybernetes/network/serversocket.hpp>
#include <kybernetes/network/socket.hpp>

#include <dispatch/dispatch.h>

using namespace std;

using namespace kybernetes::utility;
using namespace kybernetes::sensor;
using namespace kybernetes::constants;
using namespace kybernetes::network;

class TestApplication : public Application::Delegate
{
	//uint8_t testPixels[32];
	uint8_t testPixelResults[640*480];

    std::unique_ptr<UVCCamera> camera;
    std::vector<Socket>        clients;
    ServerSocket               server;

    uint32_t                   imageWidth;
    uint32_t                   imageHeight;

public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {
        imageWidth = 320;
        imageHeight = 240;
    	camera = make_unique<UVCCamera>("/dev/video0", imageWidth, imageHeight, V4L2_PIX_FMT_RGB24, dispatch_get_main_queue(), [this] (bool success)
        {
            if(!success)
            {
                cout << "An error occurred opening the camera" << endl;
                Application::Instance()->Exit();
                return;
            }

            camera->SetFrameCaptureCallback([this] (void *imageData, size_t imageDataSize)
            {
                //kybernetes::cv::yuv422_bithreshold(imageData, (void *) &testPixelResults[0], 640, 480, 64, 64, 64, 128+64, 128+64, 128+64);
                cout << "Frame processed" << endl;
                for_each(clients.begin(), clients.end(), [&] (Socket& client)
                {
                    client.Write((void *) &imageDataSize, sizeof(size_t));
                    client.Write((void *) &imageWidth, sizeof(uint32_t));
                    client.Write((void *) &imageHeight, sizeof(uint32_t));
                    client.Write(imageData, imageDataSize);
                });
            });
        });

        // Open a server to allow people to fetch images
        server.StartListening(11311);
        server.SetBlocking(false);
        PollHandler::Instance()->AttachHandler(server.GetHandle(), [this] ()
        {
            // We have a pending connection
            Socket client;
            server.AcceptConnection(client);
            clients.push_back(move(client));
        });
    }

    void ApplicationWillTerminate()
    {
        PollHandler::Instance()->DetachHandler(server.GetHandle());
        for_each(clients.begin(), clients.end(), [] (Socket& socket)
        {
            socket.Disconnect();
        });
        server.StopListening();
    }
};

int main (int argc, char** argv)
{
    Application::Run<TestApplication>(argc, argv);
    return 0;
}


/*#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
int main(int argc, char** argv)
{
    VideoCapture cap;
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
        return 0;
    for(;;)
    {
          Mat frame;
          cap >> frame;
          if( frame.empty() ) break; // end of video stream
          std::cout << "dims " << frame.cols << " " << frame.rows << std::endl;

    }
    // the camera will be closed automatically upon exit
    // cap.close();
    return 0;
}*/

