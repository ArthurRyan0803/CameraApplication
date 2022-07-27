#pragma once


#define DEFAULT_HORIZONTAL_COUNT 11
#define DEFAULT_VERTICAL_COUNT 9
#define DEFAULT_INTERVAL 15


class CalibrationBoardSettings
{

public:
	size_t horizontal_count;
	size_t vertical_count;
	double interval;		// Unit: mm

	CalibrationBoardSettings():
		horizontal_count(DEFAULT_HORIZONTAL_COUNT), vertical_count(DEFAULT_VERTICAL_COUNT), interval(DEFAULT_INTERVAL) { }

	CalibrationBoardSettings(size_t horizontal_count, size_t vertical_count, double interval):
		horizontal_count(horizontal_count), vertical_count(vertical_count), interval(interval) { }

};
