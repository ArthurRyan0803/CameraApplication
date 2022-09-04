#pragma once

#include <opencv2/opencv.hpp>


class ImageUtils
{
public:
	ImageUtils() = delete;
	~ImageUtils() = delete;

	static cv::Mat createBinaryMask(size_t width, size_t height, const std::vector<cv::Point2f>& pos_locations, uint8_t pos_value = 255)
	{
		cv::Mat image(height, width, CV_8UC1, cv::Scalar(0));

		for(const auto& pt: pos_locations)
		{
			if((0 <= pt.y) && (pt.y < height) && (0 <= pt.x) && (pt.x < width))
				image.at<uint8_t>((int)pt.y, (int)pt.x) = pos_value;
		}

		return image;
	}

	//static double meanSampleHistgram(const cv::Mat& image, uint8_t hist_regions, const cv::Mat& mask)
	//{
	//	assert(255 % hist_regions == 0);

	//	std::vector<float> hist;
	//	cv::calcHist({image}, {0}, mask, hist, {256}, {0, 256});
	//}
};
