#pragma once

#include <opencv2/opencv.hpp>

namespace CameraLib
{
	class FrameListener
	{
	public:
		virtual ~FrameListener() = default;
		virtual void frameReadyCallback(cv::InputArray data) = 0;
	};
}
