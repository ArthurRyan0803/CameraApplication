#pragma once

#include "Framework.h"
#include "FrameListener.h"

namespace CameraLib
{
#define CHECK_IS_OPENED() if(!isOpened()) throw std::logic_error("Camera is not opened!")
#define CHECK_NOT_CAPTURING() if(isCapturing()) throw std::logic_error("Camera is capturing! Please stop it first!")

	class CAMERALIB_SDK Camera: boost::noncopyable
	{
	public:

		virtual void open() = 0;
		virtual void close() = 0;
		virtual bool isOpened() = 0;

		virtual void startContinuesCapture() = 0;
		virtual void stopContinuesCapture() = 0;
		virtual bool isCapturing() = 0;

		virtual void setExposure(double us) = 0;
		virtual void getExposure(double& us) const = 0;
		virtual void getGainRange(int& min, int& max, int& step) const = 0;

		virtual void setGain(int value) = 0;
		virtual void getGain(int& value) const = 0;
		virtual void getExposureRange(double& min, double& max, double& step) const = 0;
		
		virtual void onceCapture(cv::OutputArray data) = 0;
		virtual ~Camera() = default;

		virtual void registerFrameListener(std::weak_ptr<FrameListener> listener) = 0;
	};

	template<int IWidth, int IHeight>
	void toMat(BYTE* gray, BYTE* rgb, cv::Mat& left, cv::Mat& right, cv::Mat& color)
	{
		cv::Mat gray_mat(IHeight, IWidth * 2, CV_8UC1, gray);
		cv::Mat rgb_mat(IHeight, IWidth, CV_8UC3, rgb);

		left = gray_mat.colRange(IWidth, IWidth * 2).clone(); 
		right = gray_mat.colRange(0, IHeight).clone();
		color = rgb_mat.clone();
	}
}
