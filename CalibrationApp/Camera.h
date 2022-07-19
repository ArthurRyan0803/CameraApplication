#pragma once
#include <functional>
#include <boost/core/noncopyable.hpp>
#include <opencv2/core/mat.hpp>

#include "ImageData.h"


class Camera: boost::noncopyable
{
public:
	virtual void open() = 0;
	virtual void close() = 0;
	virtual bool isOpened() = 0;

	virtual void startCapture() = 0;
	virtual void stopCapture() = 0;
	virtual bool isCapturing() = 0;

	/*
	 * @return: {width, height}
	 */
	virtual std::array<int, 2> getResolution() = 0;

	virtual bool oneShot(cv::OutputArray data) = 0;
	virtual ~Camera() = 0;

	virtual void setCapturingStartCallback(std::function<void()>&& callback) = 0;
	virtual void setCapturingStopCallback(std::function<void()>&& callback) = 0;
	virtual void setFrameReadyCallback(std::function<void(const cv::Mat&)>&& callback) = 0;
};
