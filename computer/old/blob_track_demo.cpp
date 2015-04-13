/*
 *  blob_track_demo.cpp
 *
 *  Copyright (c) 2013 Nathaniel Lewis, Robotics Society at UC Merced
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Kybernetes dependencies
#include "uvccamera.hpp"
#include "cv.hpp"

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// cvbloblib
#include <opencvblobslib/BlobResult.h>

// Language deps
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <cassert>
#include <csignal>
#include <unistd.h>

// Boost
#include <thread>
#include <mutex>

// Kybernetes objects
kybernetes::sensor::UVCCamera     *camera;
std::string                        camera_device;
int                                width;
int                                height;
int                                port;
int                                pt_response;
uint8_t                            should_track = 1;

// Shared resources for imaging
struct v4l2_buffer                 image_buffer;
void*                              image_data = NULL;
size_t                             image_size;
std::mutex                         image_data_mutex;

// Binary resultant
void*                              image_resultant = NULL;
std::mutex                         image_resultant_mutex;

// Shared flags
bool                               __kill = false;
uint8_t                            y_max = 0;
uint8_t                            y_min = 0;
uint8_t                            u_max = 0;
uint8_t                            u_min = 0;
uint8_t                            v_max = 0;
uint8_t                            v_min = 0;
std::mutex                         range_mutex;

// Catch the kill signal
void handle_sigint (int sig)
{
    __kill = true;
}

// Thread to handle image capture and processing
void image_process_thread(void)
{
    // Local copy of image data
    struct v4l2_buffer buffer;
    void*              data;
    size_t             size;
    void*              resultant;
    void*              erosion;
    bool               initialized = false;

    // Timing structures
    struct timeval  tv1, tv2;

    // Start the camera
    camera = new kybernetes::sensor::UVCCamera(camera_device, width, height, V4L2_PIX_FMT_YUYV);
    std::cout << " >> Using camera on: " << camera_device << std::endl;

    // Wait for it to initialize
    camera->capture_buffer(&buffer, &data, &size);
    camera->release_buffer(&buffer);
    camera->ptz_reset();
    sleep(3);

    // Create the structuring element for erosion
    int erosion_size = 2;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*erosion_size + 1, 2*erosion_size + 1), cv::Point(erosion_size, erosion_size));

    // Loop while not killed
    while(!__kill)
    {
        // Capture an image to the local variables
        camera->capture_buffer(&buffer, &data, &size);

        // If we have are on the first picture
        if(!initialized)
        {
            // Compute the location of the centeroid pixel
            size_t loc = (height*width) + width;
            uint8_t *image = (uint8_t *) data;
            int16_t y = (((uint16_t)image[loc]) + ((uint16_t)image[loc+2]) / 2);
            int16_t u = image[loc+1];
            int16_t v = image[loc+3];

            int16_t t = y + 64;
            if(t > 255) t = 255;
            y_max = t;
            t = y - 64;
            if(t < 0) t = 0;
            y_min = t;

            t = u + 16;
            if(t > 255) t = 255;
            u_max = t;
            t = u - 16;
            if(t < 0) t = 0;
            u_min = t;

            t = v + 16;
            if(t > 255) t = 255;
            v_max = t;
            t = v - 16;
            if(t < 0) t = 0;
            v_min = t;

            initialized = true;
        }

        // Push to shared instance (free old buffer)
        {
            std::lock_guard<std::mutex> lock(image_data_mutex);
            if(image_data) camera->release_buffer(&image_buffer);
            image_buffer = buffer;
            image_data = data;
            image_size = size;
        }

        // Get start time
        gettimeofday(&tv1, NULL);

        // Perform the resultant operation
        resultant = malloc (width * height);
        erosion   = malloc (width * height);
        {
            std::lock_guard<std::mutex> lock2(range_mutex);
            kybernetes::cv::yuv422_bithreshold(data, resultant, width, height, y_min, u_min, v_min, y_max, u_max, v_max);
        }

        // Create the eroded image
        cv::Mat a(cv::Size(width, height), CV_8UC1, resultant);
        cv::Mat b(cv::Size(width, height), CV_8UC1, erosion);
        cv::erode(a, b, element);

        // Free the resultant
        free(resultant);

        // Perform blobing
        IplImage img = b;
        CBlobResult blobs = CBlobResult(&img, NULL, 0);
        blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 50);
        CBlob biggestBlob;
        blobs.GetNthBlob(CBlobGetArea(), 0, biggestBlob);

        // Get finish time
        gettimeofday(&tv2, NULL);
        double totalTime = (double) (tv2.tv_usec - tv1.tv_usec)/1000000 + (double) (tv2.tv_sec - tv1.tv_sec);
        int     mX = biggestBlob.GetBoundingBox().x + (biggestBlob.GetBoundingBox().width / 2);
        int     mY = biggestBlob.GetBoundingBox().y + (biggestBlob.GetBoundingBox().height / 2);
        std::cout << " << largest blob at = {" << mX << ", " << mY << "}";
        std::cout << "; Time taken = " << totalTime << " >> " << std::endl;

        // Instruct pan/tilt to move
        if(mX != 0 && mY != 0 && should_track)
            camera->ptz_move_relative(((width / 2) - mX) * pt_response, (mY - (height / 2)) * pt_response, 0);

        // Store the resultant (free old one)
        {
            std::lock_guard<std::mutex> lock3(image_resultant_mutex);
            if(image_resultant) free(image_resultant);
            image_resultant = erosion;
        }
    }

    // Close the camera
    delete camera;
    std::cout << " << Camera closed" << std::endl;
}

// Main thread
int main(int argc, char **argv)
{
    // Check that we have the proper number of parameters
    if(argc < 5)
    {
        std::cerr << "Error: Too Few Arguments" << std::endl;
        std::cerr << "Usage: " << argv[0] << " <video device file> <width> <height> <pt response per pixel error>" << std::endl;
        std::cerr << "   (e.g. " << argv[0] << " /dev/video0 640 480 4)" << std::endl;
        return 1;
    }

    // Add a handler for Control-C
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = handle_sigint;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Ignore sigpipe
    signal(SIGPIPE, SIG_IGN);

    // Get the parameters (set for now)
    camera_device    = argv[1];
    width            = atoi(argv[2]);
    height           = atoi(argv[3]);
    pt_response      = atoi(argv[4]);
    should_track     = 0;

    // Start the server
    /*kybernetes::network::ServerSocket server;
    if(!server.startListening(port))
    {
        // Could not start server
        std::cerr << " << Could not start server on port " << port << std::endl;
        return 1;
    }*/

    // Start the processing thread
    std::thread processing_thread(image_process_thread);

    // Loop forever
    /*while(!__kill)
    {
        // Wait for a client to connect (and check if we get killed)
        kybernetes::network::Socket client;
        while(!server.acceptConnection(client) && !__kill) continue;
        if(__kill) break;

        // Client connected
        std::cout << " >> Client Connected" << std::endl;

        // Start streaming data while we don't encounter an error
        while(!__kill)
        {
            // Wait for a character to be received
            char command[4];
            if(!client.read((void *) &command, 1))
            {
                break;
            }

            // Figure out what to do
            if(command[0] == 'x') break;
            else if(command[0] == 'i')
            {
                // Lock the shared image data
                std::lock_guard<std::mutex> lock(image_data_mutex);

                // Write the data to the client
                assert(client.write((void *) &width, 4));
                assert(client.write((void *) &height, 4));
                assert(client.write((void *) &image_size, 4));
                assert(client.write(image_data, image_size));
            }
            else if(command[0] == 'r')
            {
                // Lock the shared image data
                std::lock_guard<std::mutex> lock(image_resultant_mutex);
                size_t nu_size = width * height;

                // Write the data to the client
                assert(client.write((void *) &width, 4));
                assert(client.write((void *) &height, 4));
                assert(client.write((void *) &nu_size, 4));
                assert(client.write(image_resultant, nu_size));
            }
            else if(command[0] == 'u')
            {
                // Update the color parameters
                std::lock_guard<std::mutex> lock(range_mutex);
                assert(client.read((void *) &y_max, 1));
                assert(client.read((void *) &y_min, 1));
                assert(client.read((void *) &u_max, 1));
                assert(client.read((void *) &u_min, 1));
                assert(client.read((void *) &v_max, 1));
                assert(client.read((void *) &v_min, 1));

                // Alert to new data
                std::cout << " >> Color params update = {";
                std::cout << (unsigned int) y_min << "," << (unsigned int) y_max << ",";
                std::cout << (unsigned int) u_min << "," << (unsigned int) u_max << ",";
                std::cout << (unsigned int) v_min << "," << (unsigned int) v_max;
                std::cout << "}" << std::endl;
            }
            else if(command[0] == 't')
            {
                // Update the color parameters
                assert(client.read((void *) &should_track, 1));

                // derp
                std::cout << " >> Tracking ";
                if(should_track) std::cout << "Enabled";
                else std::cout << "Disabled";
                std::cout << std::endl;
            }
            else if(command[0] == 'e')
            {
                camera->ptz_reset();
            }
        }

        // Stuff
        std::cout << " << Client disconnected" << std::endl;
        client.disconnect();
    }*/

    // Stop the server
    /*server.stopListening();
    std::cout << " << Server Down" << std::endl;*/
    processing_thread.join();

    // Return success
    return 0;
}
