#pragma once

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <thread>
#include "Logger.hpp"


enum Pattern { Invalid = 0, Chessboard = 1, CirclesGrid = 2 };


bool planarCalibration(
	const std::vector<std::shared_ptr<cv::Mat>>& images, 
	cv::Mat& intrinsic_matrix, cv::Mat& distort_coefficients, cv::Mat& rotation_matrix, cv::Mat& translation_vector,
	std::vector<std::vector<cv::Point2f>>& key_points, std::vector<bool>& key_points_found_flags,
	float key_points_phy_interval, const cv::Size& key_points_count, const cv::Size& sub_pixel_search_window_size, 
	Pattern pattern, double& RMS
);
