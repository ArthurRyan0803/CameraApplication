#pragma once

#include <vector>
#include <memory>
#include <opencv2/calib3d.hpp>

#include "CalibrationPatternMethod.h"
#include "Logger.hpp"


bool planarCalibration(
	const std::vector<std::shared_ptr<cv::Mat>>& images, 
	cv::Mat& intrinsic_matrix, cv::Mat& distort_coefficients, cv::Mat& rotation_matrix, cv::Mat& translation_vector,
	std::vector<std::vector<cv::Point2f>>& key_points, std::vector<bool>& key_points_found_flags,
	float key_points_phy_interval, const cv::Size& key_points_count,  
	Pattern pattern, double& RMS
);
