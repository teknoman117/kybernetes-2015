#ifndef __V4L2CAMERA_HPP__
#define __V4L2CAMERA_HPP__

#include <vector>
#include <string>
#include <functional>

#include <dispatch/dispatch.h>
#include <linux/videodev2.h>

/**
 * Definition of a V4L2 camera which pushes frames to a dispatch queue
 */
namespace kybernetes
{
	namespace sensor
	{
		class V4L2Camera
		{
		public:
			struct FrameBuffer
			{
				void*  imageData;
				size_t imageSize;
			};

			typedef std::function<void (bool)>               SuccessCallback;
			typedef std::function<void (const FrameBuffer&)> FrameCaptureCallback;

		private:

			std::vector<FrameBuffer>  mBuffers;
			int                       mDescriptor;

			size_t                    mWidth;
			size_t                    mHeight;
			int                       mFormat;
			int                       mField;
			int                       mFPS;
			bool                      mIsStreaming;

			FrameCaptureCallback      mCallback;
			dispatch_queue_t          mQueue;

			bool InternalAllocateBuffers();
			bool InternalDeallocateBuffers();
			bool InternalDeviceOpen(const std::string& path);
			bool InternalDeviceClose();
			bool InternalDeviceInit();
			bool InternalDeviceUninit();
			
			void InternalFrameEventHandler();

		public:
			V4L2Camera(const std::string& path, size_t width, size_t height, int field, int format, int fps, dispatch_queue_t queue, SuccessCallback callback);
			~V4L2Camera();

			bool SetStreaming(bool streaming);
			void SetFrameCaptureCallback(FrameCaptureCallback&& calback);
		};
	}
}

#endif
