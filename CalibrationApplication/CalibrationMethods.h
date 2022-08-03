#pragma once

#include <vector>
#include <memory>
#include <opencv2/calib3d.hpp>

#include "CalibrationPatternMethod.h"
#include "CalibrationParams.hpp"
#include "CalibrationBoardSettings.hpp"


bool planarCalibration(
	const std::vector<std::shared_ptr<cv::Mat>>& images,
	const CalibrationBoardSettings& board_settings, Pattern pattern,
	PlanarCalibrationParams& params, std::vector<std::vector<cv::Point2f>>& key_points, std::vector<bool>& key_points_found_flags
);

bool stereoCalibration(
	const std::vector<std::shared_ptr<cv::Mat>>& left_images, 
	const std::vector<std::shared_ptr<cv::Mat>>& right_images,
	size_t basis_image_index,
	const CalibrationBoardSettings& board_settings, Pattern pattern,
	PlanarCalibrationParams& left_cam_params, PlanarCalibrationParams& right_cam_params,
	StereoCalibrationParams& stereo_params,
	std::vector<std::vector<cv::Point2f>>& left_key_points, std::vector<bool>& left_flags,
	std::vector<std::vector<cv::Point2f>>& right_key_points, std::vector<bool>& right_flags
);
