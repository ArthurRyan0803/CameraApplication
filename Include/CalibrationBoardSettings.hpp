#pragma once

#include <opencv2/core/types.hpp>

#define DEFAULT_HORIZONTAL_COUNT 11
#define DEFAULT_VERTICAL_COUNT 9
#define DEFAULT_INTERVAL 15
#define DEFAULT_IMAGES 8

class CalibrationBoardSettings
{

public:
	size_t horizontal_count;
	size_t vertical_count;
	double interval;		// Unit: mm
	size_t images_count;

	cv::Size count() const
	{
		return cv::Size(horizontal_count, vertical_count);
	}

	CalibrationBoardSettings():
		horizontal_count(DEFAULT_HORIZONTAL_COUNT), vertical_count(DEFAULT_VERTICAL_COUNT), interval(DEFAULT_INTERVAL), images_count(DEFAULT_IMAGES)
	{ }

	CalibrationBoardSettings(size_t horizontal_count, size_t vertical_count, double interval, int images_count):
		horizontal_count(horizontal_count), vertical_count(vertical_count), interval(interval), images_count(images_count)
	{ }

};
