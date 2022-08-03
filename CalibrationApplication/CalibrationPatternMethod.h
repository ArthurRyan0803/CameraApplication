#pragma once

#include <opencv2/core/types.hpp>

#include "CalibrationBoardSettings.hpp"

enum Pattern { Invalid = 0, Chessboard = 1, CirclesArray = 2 };


bool findKeyPoints(
	const cv::Mat& image, std::vector<cv::Point2f>& points, Pattern pattern, const cv::Size& points_count, bool roughly=false
);

void calKeyPointsPhyCoordinates(
	const CalibrationBoardSettings& settings, Pattern pattern, std::vector<cv::Point3f>& points_coordinates
);

void calImagesKeyPointsPhyCoordinates(
	const CalibrationBoardSettings& settings, Pattern pattern, size_t images, std::vector<std::vector<cv::Point3f>>& phyKeyPoints
);
