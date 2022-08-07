#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include <Windows.h>

namespace CameraLib
{
	class PDRBuffer
	{
	public:
		std::shared_ptr<BYTE> left, right;					// the buffers to store raw image data
		std::shared_ptr<BYTE> left_isp, right_isp, rgb_isp;		// the buffers to store image data after processed

		PDRBuffer(const cv::Size& image_size);
	};
};

