/*
 *  uvccamera.hpp
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

#ifndef _kybernetes_sensor_uvccamera_h_
#define _kybernetes_sensor_uvccamera_h_

#define NB_BUFFER 16

#define PTZ_PAN_MAX 60
#define PTZ_PAN_MIN -60
#define PTZ_TILT_MAX 40
#define PTZ_TILT_MIN -40

// Language dependencies
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <functional>

#include <dispatch/dispatch.h>

// System dependencies
#include <linux/videodev2.h>

namespace kybernetes 
{
    namespace sensor 
    {
        class UVCCamera 
        {
            // The string which holds the device file this camera is on
            std::string videodevice;
            
            // File descriptor for the camera
            int cam;

            // V4L2 Structures
            struct v4l2_requestbuffers rb;
            struct v4l2_capability     cap;
            struct v4l2_format         fmt;
            struct v4l2_buffer         buf;

            // Buffers for frame data
            void         *mem[NB_BUFFER];

            // Image size
            unsigned int  width;
            unsigned int  height;
            
            // Image format
            unsigned int  formatIn;
            
            // Flag holding streaming state
            bool          isstreaming;
            
            // Position of the pan/tilt turret
            int           m_pan;
            int           m_tilt;

            // Callback variables
            std::function<void (void *data, size_t length)> frameCaptureCallback;
            std::thread                                     frameCaptureThread;
            std::atomic<bool>                               frameCaptureThreadKill;
            dispatch_queue_t                                queue;

            // Initialize the v4l2 camera
            int initV4L2();
            
            // Enable or disable streaming
            int setStreaming(bool streaming);
            
            // Access camera controls
            int isControl(int control, struct v4l2_queryctrl *queryctrl);
            int getControl(int control);
            int setControl(int control, int value);
            
            // A bit more complex controls, grab only the buffers.  You must call
            // release buffer at some point, or the camera runs out of buffers.
            int capture_buffer(struct v4l2_buffer* buffer, void** data, size_t* len);
            int release_buffer(struct v4l2_buffer* buffer);

        public:
            // Creation and destruction of the framegrabber V4L2_PIX_FMT_YUYV
            UVCCamera( std::string device, int width, int height, int format, dispatch_queue_t queue, std::function<void (bool)> callback );
            ~UVCCamera();

            // Handler
            void SetFrameCaptureCallback(std::function<void (void *data, size_t length)>&& callback);

            // Standard color controls
            int brightness( int nbrightness );
            int saturation( int nsaturation );
            int contrast( int ncontrast );
            int gain( int ngain );
            
            // Pan, tilt, zoom controls
            int ptz_reset(void);
            int ptz_move_relative(int pan, int tilt, int zoom);
            int ptz_move_absolute(int pan, int tilt, int zoom);
        };
    }
}

#endif
 
