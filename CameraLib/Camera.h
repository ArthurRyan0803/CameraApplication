#pragma once

#include "Framework.h"

namespace CameraLib
{
	#define CHECK_IS_OPENED() if(!isOpened()) throw std::logic_error("Camera is not opened!")
	#define CHECK_NOT_CAPTURING() if(isCapturing()) throw std::logic_error("Camera is capturing! Please stop it first!")

	class CAMERALIB_DLL Camera: boost::noncopyable
	{
	public:

		virtual void open() = 0;
		virtual void close() = 0;
		virtual bool isOpened() = 0;

		virtual void startCapture() = 0;
		virtual void stopCapture() = 0;
		virtual bool isCapturing() = 0;

		virtual void showParameterDialog() = 0;

		virtual size_t getViews() = 0;
	
		virtual void oneShot(cv::OutputArray data) = 0;
		virtual ~Camera() = 0;

		virtual void setFrameReadyCallback(std::function<void(cv::InputArray)> callback) = 0;
	};
	
}
