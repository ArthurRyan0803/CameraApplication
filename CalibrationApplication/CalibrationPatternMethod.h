#pragma once

#include <opencv2/core/types.hpp>

enum Pattern { Invalid = 0, Chessboard = 1, CirclesArray = 2 };


bool findKeyPoints(
	const cv::Mat& image, std::vector<cv::Point2f>& points, Pattern pattern, const cv::Size& points_count, bool roughly=false
);

void calKeyPointsPhyCoordinates(
	const cv::Size& key_points_count, float key_points_phy_interval, 
	std::vector<cv::Point3f>& points_coordinates, Pattern pattern
);

