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
#include <kybernetes/sensor/v4l2camera.hpp>
#include <kybernetes/cv/cv.hpp>

#include <kybernetes/network/serversocket.hpp>
#include <kybernetes/network/socket.hpp>

#include <dispatch/dispatch.h>
#include <chrono>
#include <jpeglib.h>
#include <cstdio>
#include <sstream>

using namespace std;
using namespace std::chrono;

using namespace kybernetes::utility;
using namespace kybernetes::sensor;
using namespace kybernetes::constants;
using namespace kybernetes::network;

namespace
{
    static void jpegWrite(unsigned char* img, int width, int height, char* jpegFilename)
    {
        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;

        JSAMPROW row_pointer[1];
        FILE *outfile = fopen( jpegFilename, "wb" );

        // try to open file for saving
        if (!outfile) {
            //errno_exit("jpeg");
            return;
        }

        // create jpeg data
        cinfo.err = jpeg_std_error( &jerr );
        jpeg_create_compress(&cinfo);
        jpeg_stdio_dest(&cinfo, outfile);

        // set image parameters
        cinfo.image_width = width;
        cinfo.image_height = height;
        cinfo.input_components = 3;
        cinfo.in_color_space = JCS_YCbCr;

        // set jpeg compression parameters to default
        jpeg_set_defaults(&cinfo);
        // and then adjust quality setting
        jpeg_set_quality(&cinfo, 70, TRUE);

        // start compress
        jpeg_start_compress(&cinfo, TRUE);

        // feed data
        while (cinfo.next_scanline < cinfo.image_height) {
            row_pointer[0] = &img[cinfo.next_scanline * cinfo.image_width *  cinfo.input_components];
            jpeg_write_scanlines(&cinfo, row_pointer, 1);
        }

        // finish compression
        jpeg_finish_compress(&cinfo);

        // destroy jpeg data
        jpeg_destroy_compress(&cinfo);

        // close output file
        fclose(outfile);
    }
}

class TestApplication : public Application::Delegate
{
	//uint8_t testPixels[32];
	uint8_t testPixelResults[640*480];

    std::unique_ptr<V4L2Camera> camera;
    std::vector<Socket>        clients;
    ServerSocket               server;
    high_resolution_clock::time_point last;

    uint32_t                   imageWidth;
    uint32_t                   imageHeight;
    uint32_t                   count;
    uint32_t                   n;

public:
    void ApplicationDidLaunch(Application *application, int argc, char **argv)
    {
        imageWidth = 320;
        imageHeight = 240;
        n=0;
        last = high_resolution_clock::now();

    	camera = make_unique<V4L2Camera>("/dev/video0", imageWidth, imageHeight, V4L2_FIELD_ANY, V4L2_PIX_FMT_YUYV, 30, dispatch_get_main_queue(), [this] (bool success)
        {
            if(!success)
            {
                cout << "An error occurred opening the camera" << endl;
                Application::Instance()->Exit();
                return;
            }

            cout << "Camera opened" << endl;
            camera->SetFrameCaptureCallback([this] (const V4L2Camera::FrameBuffer &image)
            {
                high_resolution_clock::time_point now = high_resolution_clock::now();
                duration<double> frameDeltaRaw = duration_cast<duration<double>>(now-last);
                last = now;
                cout << "Frame Delta - " << frameDeltaRaw.count() << ", size = " << image.imageSize << endl;

                //kybernetes::cv::yuv422_bithreshold(image.imageData, (void *) &testPixelResults[0], imageWidth, imageHeight, 64, 64, 64, 128+64, 128+64, 128+64);
                if(++count == 10)
                {
                    // conver to yuv444
                    size_t imageSize  = 2 * imageWidth * imageHeight;
                    size_t imageSize2 = 3 * imageWidth * imageHeight;
                    std::vector<uint8_t> yuv444;
                    uint8_t *yuv422 = (uint8_t*) image.imageData;
                    yuv444.resize(imageSize2);
                    for(int i = 0, j = 0; i < imageSize; i+=4, j+=6)
                    {
                        yuv444[j+0] = yuv422[i+0];
                        yuv444[j+1] = yuv422[i+1];
                        yuv444[j+2] = yuv422[i+3];
                        yuv444[j+3] = yuv422[i+2];
                        yuv444[j+4] = yuv422[i+1];
                        yuv444[j+5] = yuv422[i+3];
                    }

                    ostringstream name;
                    name << "output_" << n++ << ".jpg" << ends;
                    jpegWrite(yuv444.data(), (int) imageWidth, (int) imageHeight, (char*) name.str().data());

                    count = 0;
                    for_each(clients.begin(), clients.end(), [&] (Socket& client)
                    {
                        client.Write((void *) &image.imageSize, sizeof(size_t));
                        client.Write((void *) &imageWidth, sizeof(uint32_t));
                        client.Write((void *) &imageHeight, sizeof(uint32_t));
                        client.Write(image.imageData, image.imageSize);
                    });
                }
            });
            camera->SetStreaming(true);
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

