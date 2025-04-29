/*
 *  uvccamera.cpp
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

#include <kybernetes/sensor/uvccamera.hpp>

// System dependencies
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

// Language deps
#include <iostream>
#include <cstdlib>
#include <cstring>

using namespace kybernetes::sensor;
using namespace std;

// Supports V4L2_PIX_FMT_YUYV and V4L2_PIX_FMT_MJPEG
UVCCamera::UVCCamera( std::string device, int _width, int _height, int format, dispatch_queue_t queue_, std::function<void (bool)> callback )
    : videodevice(device), width(_width), height(_height), formatIn(format), isstreaming(false), m_pan(0), m_tilt(0), queue(queue_)
{
    // Check that we have correct parameters
    if( (device.length() == 0) || (width == 0) || (height == 0) ) 
    {
        dispatch_async(queue, ^{callback(false);});
        return;
    }

    // Try to initialize V4L2
    if(initV4L2()) 
    {
        close(cam);
        dispatch_async(queue, ^{callback(false);});
        return;
    }

    // Try to start the video feed
    if(setStreaming(true))
    {
        close(cam);
        dispatch_async(queue, ^{callback(false);});
        return;
    }

    // 
    frameCaptureThreadKill = false;
    frameCaptureThread = thread([this] ()
    {
        while(!frameCaptureThreadKill)
        {
            struct v4l2_buffer  buffer;
            void               *imageData;
            size_t              imageDataSize;

            if(!capture_buffer(&buffer, &imageData, &imageDataSize))
            {
                dispatch_async(queue, ^
                {
                    // clone the buffer so we can delete it
                    struct v4l2_buffer dbuffer = buffer;
                    if(frameCaptureCallback)
                    {
                        frameCaptureCallback(imageData, imageDataSize);
                    }
                    release_buffer(&dbuffer);
                });
            }
        }
    });
    frameCaptureThread.detach();

    dispatch_async(queue, ^{callback(true);});
}

UVCCamera::~UVCCamera()
{
    frameCaptureThreadKill = true;
    frameCaptureThread.join();

    // Stop streaming
    setStreaming(false);
    
    // Unmap the buffers
    for (int i = 0; i < NB_BUFFER; i++) 
        munmap (mem[i], buf.length);
    
    // Close the camera
    close(cam);
}

void UVCCamera::SetFrameCaptureCallback(std::function<void (void *data, size_t length)>&& callback)
{
    frameCaptureCallback = move(callback);
}

int UVCCamera::initV4L2()
{
    // Open the camera device file
    if( (cam = open(videodevice.c_str(), O_RDWR)) < 0 ) 
    {
        std::cerr << "Error: UVCCamera::initV4L2() - failed to open device" << std::endl;
        return 1;
    }

    // Query the device capabilities
    memset (&cap, 0, sizeof (struct v4l2_capability));
    if( ioctl (cam, VIDIOC_QUERYCAP, &cap) < 0 ) 
    {
        std::cerr << "Error: UVCCamera::initV4L2() - unable to query device " << videodevice << std::endl;
        return 1;
    }

    // If the capabilities don't contain image capture, error
    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) 
    {
        std::cerr << "Error: UVCCamera::initV4L2() - " << videodevice << " no video capture support" << std::endl;
        return 1;
    }

    // If the capabilities don't contain video streaming, error
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) 
    {
        std::cerr << "Error: UVCCamera::initV4L2() - " << videodevice << " no i/o streaming support" << std::endl;
        return 1;
    }

    // Create the format structure, which defines camera settings
    memset (&fmt, 0, sizeof (struct v4l2_format));
    fmt.fmt.pix.pixelformat = formatIn;
    fmt.fmt.pix.width       = width;
    fmt.fmt.pix.height      = height;
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Set the format of the camera
    if( ioctl(cam, VIDIOC_S_FMT, &fmt) < 0 ) {
        std::cerr << "Error: UVCCamera::initV4L2() - Unable to set format" << std::endl;
        return 1;
    }

    // Check that the dimension settings were accepted
    if ((fmt.fmt.pix.width != width) || (fmt.fmt.pix.height != height)) {
        std::cerr << "Warning: UVCCamera::initV4L2() - size (" << width << "," << height << ") unavailable, using (" << fmt.fmt.pix.width << "," << fmt.fmt.pix.height << ") instead" << std::endl;
        width = fmt.fmt.pix.width;
        height = fmt.fmt.pix.height;
    }

    // Check that the pixel format was accepted
    if( fmt.fmt.pix.pixelformat != formatIn ) {
        std::cerr << "Warning: UVCCamera::initV4L2() - format [" << formatIn << "] unavailable, using [" << fmt.fmt.pix.pixelformat << "] instead" << std::endl;
        formatIn = fmt.fmt.pix.pixelformat;
    }

    // Request the buffers in which image data is stored
    memset (&this->rb, 0, sizeof (struct v4l2_requestbuffers));
    rb.count = NB_BUFFER;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    if( ioctl (cam, VIDIOC_REQBUFS, &rb) < 0 ) {
        std::cerr << "Error: UVCCamera::initV4L2() - Unable to allocate buffers" << std::endl;
        return 1;
    }

    // Query and map buffers
    for (int i = 0; i < NB_BUFFER; i++) {
        // Create the buffer 
        memset (&buf, 0, sizeof (struct v4l2_buffer));
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if( ioctl (this->cam, VIDIOC_QUERYBUF, &this->buf) < 0 ) {
            std::cerr << "Error: UVCCamera::initV4L2() - Unable to query buffer [" << i << "]" << std::endl;
            return 1;
        }

        // Memory map the buffer
        mem[i] = mmap (0, buf.length, PROT_READ, MAP_SHARED, cam, buf.m.offset);
        if (this->mem[i] == MAP_FAILED) {
            std::cerr << "Error: UVCCamera::initV4L2() - Unable to map buffer [" << i << "]" << std::endl;
            return 1;
        }
    }

    // Queue the buffers for usage
    for (int i = 0; i < NB_BUFFER; ++i) {
        // Select the buffer
        memset (&buf, 0, sizeof (struct v4l2_buffer));
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        // Queue the buffer
        if( ioctl(cam, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "Error: UVCCamera::initV4L2() - Unable to queue buffer [" << i << "]" << std::endl;
            return 1;
        }
    }
    return 0;
}

// Set if streaming is enabled or not
int UVCCamera::setStreaming(bool streaming)
{
    // If we are already in a particular mode, ignore the call
    if(isstreaming == streaming) return 0;

    // Otherwise, turn streaming on or off
    int command = (isstreaming = streaming) ? VIDIOC_STREAMON : VIDIOC_STREAMOFF;
    int type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Call ioctl
    if( ioctl (cam, command, &type) < 0) {
        std::cerr << "Error: UVCCamera::setStreaming() - Unable to set stream mode to [" << isstreaming << "]" << std::endl;
        return 1;
    } 

    // Return success
    return 0;   
}

int UVCCamera::capture_buffer(struct v4l2_buffer* buffer, void** data, size_t* len)
{
    // Ensure the video is indeed streaming
    if(setStreaming(true)) {
        std::cerr << "Error: UVCCamera::capture_buffer() - could not start video stream" << std::endl;
        return 1;
    }

    // Set the buffer type
    memset( buffer, 0, sizeof (struct v4l2_buffer) );
    buffer->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer->memory = V4L2_MEMORY_MMAP;

    // Dequeue a buffer frame
    if( ioctl (cam, VIDIOC_DQBUF, buffer) < 0 ) {
        std::cerr << "Error: UVCCamera::capture_buffer() - Unable to dequeue buffer" << std::endl;
        return 1;
    }
    
    // Assign the provided pointer to a pointer to the video buffer
    *data = mem[buffer->index];
    
    // Return how many bytes were used
    *len    = buffer->bytesused;
    return 0;
}

int UVCCamera::release_buffer(struct v4l2_buffer* buffer)
{
    // Requeue the buffer
    if( ioctl (this->cam, VIDIOC_QBUF, buffer) < 0 ) {
        std::cerr << "Error: UVCCamera::release_buffer() - Unable to requeue buffer" << std::endl;
        return 1;
    }
    return 0;
}

int UVCCamera::isControl(int control, struct v4l2_queryctrl *queryctrl) {
    queryctrl->id = control;
    if ( ioctl (this->cam, VIDIOC_QUERYCTRL, queryctrl) < 0)
    {
        std::cerr << "Error: UVCCamera::isControl() - ioctl querycontrol error" << std::endl;
    } else if (queryctrl->flags & V4L2_CTRL_FLAG_DISABLED)
    {
        //ROS_ERROR( "UVCCamera::isControl() - control %s disabled\n", (char *) queryctrl->name);
    } else if (queryctrl->flags & V4L2_CTRL_TYPE_BOOLEAN)
    {
        return 0;
    } else if (queryctrl->type & V4L2_CTRL_TYPE_INTEGER)
    {
        return 0;
    } else
    {
        //ROS_ERROR( "UVCCamera::isControl() - control %s unsupported\n", (char *) queryctrl->name);
    }
    return 1;
}

int UVCCamera::getControl(int control) {
    // Query controls structures
    struct v4l2_queryctrl queryctrl;
    struct v4l2_control   control_s;
    
    // Test if the the requested control is an actual control
    if (this->isControl (control, &queryctrl))
        return 1;
    
    // Otherwise read the control value
    control_s.id = control;
    if ( ioctl (this->cam, VIDIOC_G_CTRL, &control_s) < 0) {
        std::cerr << "Error: UVCCamera::getControl() - ioctl get control error" << std::endl;
        return 1;
    }
    return control_s.value;
}

int UVCCamera::setControl(int control, int value) {
    // Query controls structures
    struct v4l2_queryctrl queryctrl;
    struct v4l2_control   control_s;
    
    // Test if the the requested control is an actual control
    if (this->isControl (control, &queryctrl) < 0) return -1;
    
    // If the requested value is within the bounds of the control, attempt to set the control
    if ((value >= queryctrl.minimum) && (value <= queryctrl.maximum)) {
        control_s.id = control;
        control_s.value = value;
        if (ioctl (this->cam, VIDIOC_S_CTRL, &control_s) < 0) {
            std::cerr << "Error: UVCCamera::setControl() - ioctl set control error" << std::endl;
            return 1;
        }
    }
    return 0;
}

int UVCCamera::saturation( int nsaturation ) {
    if( nsaturation ) {         //if nsaturation > 0 it will overwrite current one
        setControl(V4L2_CID_SATURATION, nsaturation);
    }
    return getControl(V4L2_CID_SATURATION);
}

int UVCCamera::brightness( int nbrightness ) {
    if( nbrightness ) {         //if nbrightmess > 0 it will overwrite current one
        this->setControl(V4L2_CID_BRIGHTNESS, nbrightness);
    }
    return this->getControl(V4L2_CID_BRIGHTNESS);
}

int UVCCamera::contrast( int ncontrast ) {
    if( ncontrast ) {         //if ncontrast > 0 it will overwrite current one
        this->setControl(V4L2_CID_CONTRAST, ncontrast);
    }
    return this->getControl(V4L2_CID_CONTRAST);
}

int UVCCamera::gain( int ngain ) {
    if( ngain ) {         //if ngain > 0 it will overwrite current one
        this->setControl(V4L2_CID_GAIN, ngain);
    }
    return this->getControl(V4L2_CID_GAIN);
}

int UVCCamera::ptz_reset( void ) {
    // V4L2 External Control Structures
    struct v4l2_ext_control xctrls[2];
    struct v4l2_ext_controls ctrls;

    // Set up controls for a total PTZ reset
    xctrls[0].id = V4L2_CID_PAN_RESET;
    xctrls[0].value = 1;
    xctrls[1].id = V4L2_CID_TILT_RESET;
    xctrls[1].value = 1;
    ctrls.count = 2;
    ctrls.controls = xctrls;

    // Execute
    if (ioctl(cam, VIDIOC_S_EXT_CTRLS, &ctrls) < 0)
    {
        std::cerr << "VIDIOC_S_EXT_CTRLS - Pan/Tilt error. Are the extended controls available?" << std::endl;
        return 1;
    }
    
    // upon success, reset internal position tracking
    m_pan = 0;
    m_tilt = 0;
    
    return 0;
}
 
int UVCCamera::ptz_move_relative(int pan, int tilt, int zoom)
{
    // Check boundaries
    if(m_pan + pan > PTZ_PAN_MAX || m_pan + pan < PTZ_PAN_MIN || m_tilt + tilt > PTZ_TILT_MAX || m_tilt + tilt < PTZ_TILT_MIN)
    {
        std::cerr << "Error: UVCCamera::ptz_move_relative() - requested motion would put camera beyond bounds" << std::endl;
        return 1;
    }
    
    // V4L2 External Control Structures
    struct v4l2_ext_control xctrls[3];
    struct v4l2_ext_controls ctrls;

    // Set up controls for a PTZ move
    xctrls[0].id = V4L2_CID_PAN_RELATIVE;
    xctrls[0].value = pan * 64;
    xctrls[1].id = V4L2_CID_TILT_RELATIVE;
    xctrls[1].value = tilt * 64;
    ctrls.count = 2;
    ctrls.controls = xctrls;

    // Execute
    if (ioctl(cam, VIDIOC_S_EXT_CTRLS, &ctrls) < 0)
    {
        std::cerr << "VIDIOC_S_EXT_CTRLS - Pan/Tilt error. Are the extended controls available?" << std::endl;
        return 1;
    }
    
    // Now that we have succeeded, store the motion
    m_pan += pan;
    m_tilt += tilt;
 
    // return success   
    return 0;
}

int UVCCamera::ptz_move_absolute(int pan, int tilt, int zoom)
{
    // Check boundaries
    if(pan > PTZ_PAN_MAX || pan < PTZ_PAN_MIN || tilt > PTZ_TILT_MAX || tilt < PTZ_TILT_MIN)
    {
        std::cerr << "Error: UVCCamera::ptz_move_absolute() - requested motion is beyond camera bounds" << std::endl;
        return 1;
    }
    
    // V4L2 External Control Structures
    struct v4l2_ext_control xctrls[3];
    struct v4l2_ext_controls ctrls;

    // Set up controls for a PTZ move
    xctrls[0].id = V4L2_CID_PAN_RELATIVE;
    xctrls[0].value = (m_pan - pan) * 64;  //multiply difference by 64, because camera takes motions in 1/64 degree increments
    xctrls[1].id = V4L2_CID_TILT_RELATIVE;
    xctrls[1].value = (m_tilt - tilt) * 64;
    ctrls.count = 2;
    ctrls.controls = xctrls;

    // Execute
    if (ioctl(cam, VIDIOC_S_EXT_CTRLS, &ctrls) < 0)
    {
        std::cerr << "VIDIOC_S_EXT_CTRLS - Pan/Tilt error. Are the extended controls available?" << std::endl;
        return 1;
    }
    
    // Now that we have succeeded, store the motion
    m_pan = pan;
    m_tilt = tilt;
 
    // return success   
    return 0;
}
