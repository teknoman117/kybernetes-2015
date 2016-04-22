#include <kybernetes/sensor/v4l2camera.hpp>
#include <kybernetes/utility/pollhandler.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <asm/types.h>
#include <libv4l2.h>

#include <cinttypes>
#include <cerrno>
#include <cstring>
#include <cassert>

#include <algorithm>

#define CLEAR(x) memset ( (void *) &(x), 0, sizeof (x))
#define VIDIOC_REQBUFS_COUNT 16

//#define __DEBUG_V4L2__

#ifdef __DEBUG_V4L2__
#include <iostream>
#include <chrono>
#endif

using namespace std;
using namespace std::chrono;

// Unexported data
namespace
{
	/**
		Do ioctl and retry if error was EINTR ("A signal was caught during the ioctl() operation."). Parameters are the same as on ioctl.

		\param fd file descriptor
		\param request request
		\param argp argument
		\returns result from ioctl
	*/
	int xioctl(int fd, int request, void* argp)
	{
		int r;

		do r = v4l2_ioctl(fd, request, argp);
		while (-1 == r && EINTR == errno);

		return r;
	}
}

namespace kybernetes
{
	namespace sensor
	{
		V4L2Camera::V4L2Camera(const std::string& path, size_t width, size_t height, int field, int format, int fps, dispatch_queue_t queue, SuccessCallback callback)
			: mWidth(width), mHeight(height), mFormat(format), mField(field), mFPS(fps), mQueue(queue), mIsStreaming(false)
		{
			if(!InternalDeviceOpen(path))
			{
				dispatch_async(mQueue, ^{callback(false);});
				return;
			}

			if(!InternalDeviceInit())
			{
				dispatch_async(mQueue, ^{callback(false);});
				return;
			}

			dispatch_async(mQueue, ^{callback(true);});
		}

		V4L2Camera::~V4L2Camera()
		{
			SetStreaming(false);
			InternalDeviceUninit();
			InternalDeviceClose();

			#ifdef __DEBUG_V4L2__
			cerr << "[V4L2Camera] Camera closed" << endl;
			#endif
		}

		bool V4L2Camera::InternalAllocateBuffers()
		{
			struct v4l2_requestbuffers req;

			CLEAR(req);

			req.count  = VIDIOC_REQBUFS_COUNT;
			req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			req.memory = V4L2_MEMORY_MMAP;

			if (-1 == xioctl(mDescriptor, VIDIOC_REQBUFS, &req))
			{
				#ifdef __DEBUG_V4L2__
				if (EINVAL == errno) 
				{
					cerr << "[V4L2Camera] FATAL: VIDIOC_REQBUFS: Does not support mmap'd buffers" << endl;
				} 
				else 
				{
					cerr << "[V4L2Camera] FATAL: VIDIOC_REQBUFS" << endl;
				}
				#endif

				return false;
			}

			if (req.count < VIDIOC_REQBUFS_COUNT/2) 
			{
				#ifdef __DEBUG_V4L2__
				cerr << "[V4L2Camera] FATAL: Insufficient buffer memory - got " << req.count << endl;
				#endif

				return false;
			}

			mBuffers.resize(req.count);

			int  idx   = 0;
			bool error = false;
			for_each (mBuffers.begin(), mBuffers.end(), [&] (V4L2Camera::FrameBuffer& buffer)
			{
				struct v4l2_buffer buf;

				CLEAR(buf);

				buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				buf.memory = V4L2_MEMORY_MMAP;
				buf.index  = idx++;

				if (-1 == xioctl(mDescriptor, VIDIOC_QUERYBUF, &buf))
				{
					#ifdef __DEBUG_V4L2__
					cerr << "[V4L2Camera] FATAL: VIDIOC_QUERYBUF" << endl;
					#endif

					error = true;
					return;
				}

				buffer.imageData = v4l2_mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, mDescriptor, buf.m.offset);
				buffer.imageSize = buf.length;

				if (MAP_FAILED == buffer.imageData)
				{
					#ifdef __DEBUG_V4L2__
					cerr << "[V4L2Camera] FATAL: MMAP" << endl;
					#endif

					error = true;
					return;
				}
			});

			return !error;
		}

		bool V4L2Camera::InternalDeallocateBuffers()
		{
			bool error = false;
			for_each(mBuffers.begin(), mBuffers.end(), [&] (const V4L2Camera::FrameBuffer& buffer)
		    {
		        if (-1 == v4l2_munmap(buffer.imageData, buffer.imageSize))
		        {
					#ifdef __DEBUG_V4L2__
					cerr << "[V4L2Camera] FATAL: MUNMAP" << endl;
					#endif

					error = true;
		        }
		    });

		    return !error;
		}

		bool V4L2Camera::InternalDeviceOpen(const string& path)
		{
			struct stat st;

			// stat file
			if (-1 == stat(path.c_str(), &st))
			{
				#ifdef __DEBUG_V4L2__
				cerr << "[V4L2Camera] FATAL: Cannot identify \'" << path << "\'" << endl;
				#endif

				return false;
			}

			// check if its device
			if (!S_ISCHR(st.st_mode))
			{
				#ifdef __DEBUG_V4L2__
				cerr << "[V4L2Camera] FATAL: Not a device: \'" << path << "\'" << endl;
				#endif

				return false;
			}

			// open device
			mDescriptor = v4l2_open(path.c_str(), O_RDWR | O_NONBLOCK, 0);

			// check if opening was successfull
			if (-1 == mDescriptor) 
			{
				#ifdef __DEBUG_V4L2__
				cerr << "[V4L2Camera] FATAL: Can not open: \'" << path << "\'" << endl;
				#endif

				return false;
			}

			return true;
		}

		bool V4L2Camera::InternalDeviceClose()
		{
			if (-1 == v4l2_close(mDescriptor))
			{
				#ifdef __DEBUG_V4L2__
				cerr << "[V4L2Camera] FATAL: Can not close device" << endl;
				#endif

				return false;
			}

			mDescriptor = -1;
			return true;
		}

		bool V4L2Camera::InternalDeviceInit()
		{
			struct v4l2_capability cap;
			struct v4l2_cropcap    cropcap;
			struct v4l2_crop       crop;
			struct v4l2_format     fmt;
			struct v4l2_streamparm frameint;

			if (-1 == xioctl(mDescriptor, VIDIOC_QUERYCAP, &cap))
			{
				#ifdef __DEBUG_V4L2__
				if (EINVAL == errno) 
				{
					cerr << "[V4L2Camera] FATAL: VIDIOC_QUERYCAP: Device is no V4L2 device" << endl; 
				} else 
				{
					cerr << "[V4L2Camera] FATAL: VIDIOC_QUERYCAP" << endl; 
				}
				#endif

				return false;
			}

			if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
			{
				#ifdef __DEBUG_V4L2__
				cerr << "[V4L2Camera] FATAL: VIDIOC_QUERYCAP: Device is no video capture device" << endl; 
				#endif

				return false;
			}
		    
		    if (!(cap.capabilities & V4L2_CAP_STREAMING))
		    {
				#ifdef __DEBUG_V4L2__
				cerr << "[V4L2Camera] FATAL: VIDIOC_QUERYCAP: Device does not support streaming I/O" << endl; 
				#endif

				return false;
		    }

			/* Select video input, video standard and tune here. */
			CLEAR(cropcap);
			cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			if (0 == xioctl(mDescriptor, VIDIOC_CROPCAP, &cropcap))
			{
				crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				crop.c = cropcap.defrect; /* reset to default */

				if (-1 == xioctl(mDescriptor, VIDIOC_S_CROP, &crop))
				{
					switch (errno)
					{
						case EINVAL:
							/* Cropping not supported. */
							break;
						default:
							/* Errors ignored. */
							break;
					}
				}
			} 
			else 
			{
				/* Errors ignored. */
			}

			CLEAR(fmt);

			// v4l2_format
			//V4L2_FIELD_ANY
			fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			fmt.fmt.pix.width       = mWidth;
			fmt.fmt.pix.height      = mHeight;
			fmt.fmt.pix.field       = mField;
			fmt.fmt.pix.pixelformat = mFormat;

			if (-1 == xioctl(mDescriptor, VIDIOC_S_FMT, &fmt))
			{
				#ifdef __DEBUG_V4L2__
				cerr << "[V4L2Camera] FATAL: VIDIOC_S_FMT" << endl; 
				#endif

				return false;
			}

			if (fmt.fmt.pix.pixelformat != mFormat)
			{
				#ifdef __DEBUG_V4L2__
				cerr << "[V4L2Camera] FATAL: VIDIOC_S_FMT: Device did not accept format" << endl; 
				#endif

				return false;
			}

			/* Note VIDIOC_S_FMT may change width and height. */
			if (mWidth != fmt.fmt.pix.width)
			{
				mWidth = fmt.fmt.pix.width;
				
				#ifdef __DEBUG_V4L2__
				cerr << "[V4L2Camera] WARN: VIDIOC_S_FMT: Device width changed to " << mWidth << endl; 
				#endif
			}

			if (mHeight != fmt.fmt.pix.height)
			{
				mHeight = fmt.fmt.pix.height;
				
				#ifdef __DEBUG_V4L2__
				cerr << "[V4L2Camera] WARN: VIDIOC_S_FMT: Device height changed to " << mHeight << endl; 
				#endif
			}
			
			/* If the user has set the fps to -1, don't try to set the frame interval */
			if (mFPS != -1)
			{
				CLEAR(frameint);

				/* Attempt to set the frame interval. */
				frameint.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				frameint.parm.capture.timeperframe.numerator = 1;
				frameint.parm.capture.timeperframe.denominator = mFPS;
				if (-1 == xioctl(mDescriptor, VIDIOC_S_PARM, &frameint))
				{
					#ifdef __DEBUG_V4L2__
					cerr << "[V4L2Camera] WARN: VIDIOC_S_PARM: Unable to change frame rate" << endl; 
					#endif
				}
			}

			// Allocate buffers
			return InternalAllocateBuffers();
		}

		bool V4L2Camera::InternalDeviceUninit()
		{
			return InternalDeallocateBuffers();
		}

		bool V4L2Camera::SetStreaming(bool streaming)
		{
			if(streaming != mIsStreaming)
			{
				if(streaming)
				{
					unsigned int i = 0;
					bool error = false;

					for_each(mBuffers.begin(), mBuffers.end(), [&] (V4L2Camera::FrameBuffer&)
					{
						struct v4l2_buffer buf;

						CLEAR(buf);

						buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
						buf.memory = V4L2_MEMORY_MMAP;
						buf.index  = i++;

						if (-1 == xioctl(mDescriptor, VIDIOC_QBUF, &buf))
						{
							#ifdef __DEBUG_V4L2__
							cerr << "[V4L2Camera] FATAL: VIDIOC_QBUF: Unable to queue buffer" << endl; 
							#endif

							error = true;
						}
					});

					if(error)
						return false;

					enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
					if (-1 == xioctl(mDescriptor, VIDIOC_STREAMON, &type))
					{
						#ifdef __DEBUG_V4L2__
						cerr << "[V4L2Camera] FATAL: VIDIOC_STEAMON: Unable to begin streaming" << endl; 
						#endif

						return false;
					}

					// Register the handler
					utility::PollHandler::Instance()->AttachHandler(mDescriptor, bind(&V4L2Camera::InternalFrameEventHandler, this));
				}
				else
				{
					utility::PollHandler::Instance()->DetachHandler(mDescriptor);

					enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
				    if (-1 == xioctl(mDescriptor, VIDIOC_STREAMOFF, &type))
					{
						#ifdef __DEBUG_V4L2__
						cerr << "[V4L2Camera] FATAL: VIDIOC_STEAMOFF: Unable to stop streaming" << endl; 
						#endif

						return false;
					}
				}
			}
			
			mIsStreaming = streaming;
			return true;
		}

		void V4L2Camera::InternalFrameEventHandler()
		{
			struct v4l2_buffer buf;

			CLEAR(buf);

			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;

			#ifdef __DEBUG_V4L2__
			high_resolution_clock::time_point start = high_resolution_clock::now();
			#endif

			if (-1 == xioctl(mDescriptor, VIDIOC_DQBUF, &buf)) 
			{
				switch (errno) 
				{
					case EAGAIN:
						return;

					case EIO:
						// Could ignore EIO, see spec
						// fall through

					default:
						#ifdef __DEBUG_V4L2__
						cerr << "[V4L2Camera] FATAL: VIDIOC_DQBUF: Unable to get buffer!!" << endl; 
						#endif
						break;
				}
			}

			assert(buf.index < n_buffers);

			#ifdef __DEBUG_V4L2__
			high_resolution_clock::time_point end = high_resolution_clock::now();
			duration<double> frameDeltaRaw = duration_cast<duration<double>>(end-start);
			cerr << "read wait - " << frameDeltaRaw.count() << endl;
			#endif

			// Dispatch the frame capture event
			dispatch_async(mQueue, ^
			{
				if(mCallback)
				{
					mCallback(mBuffers[buf.index]);
				}

				if (-1 == xioctl(mDescriptor, VIDIOC_QBUF, (void *) &buf))
				{
					#ifdef __DEBUG_V4L2__
					cerr << "[V4L2Camera] FATAL: VIDIOC_QBUF: Unable to return buffer!!" << endl; 
					#endif
				}
			});
		}

		void V4L2Camera::SetFrameCaptureCallback(FrameCaptureCallback&& callback)
		{
			mCallback = move(callback);
		}
	}
}
