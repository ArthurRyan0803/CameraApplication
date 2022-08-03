#include "CalibrationMethods.h"


constexpr int CHESSBOARD_FLAGS = cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ADAPTIVE_THRESH;
const cv::TermCriteria SUB_PIXEL_SEARCH_CRITERIA(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001);

static Logger& logger_ = GET_LOGGER();


void find_key_points_mt(
	const std::vector<std::shared_ptr<cv::Mat>>& images,
	std::vector<std::vector<cv::Point2f>>& key_points,
	std::vector<bool>& flags, Pattern pattern, CalibrationBoardSettings settings
)
{
	key_points.resize(images.size());
	flags.resize(images.size());

	std::vector<std::shared_ptr<std::thread>> threads;
	for(size_t i=0; i<images.size(); i++)
	{
		std::shared_ptr<std::thread> thd = std::make_shared<std::thread>(
			[&flags, i, &images, &key_points, &settings, pattern]
			{
				try
				{
					flags[i] = findKeyPoints(
						*images[i], key_points[i], pattern, settings.count(), false
					);
				}
				catch (const std::exception& e)
				{
					logger_.error(e.what());
					throw e;
				}
			}
		);
		threads.push_back(thd);
	}

	for(auto& thd: threads)
		thd->join();
}


bool planarCalibration(
	const std::vector<std::shared_ptr<cv::Mat>>& images,
	const CalibrationBoardSettings& board_settings, Pattern pattern,
	PlanarCalibrationParams& params, std::vector<std::vector<cv::Point2f>>& key_points, std::vector<bool>& key_points_found_flags
)
{
	try
	{
		auto imageSize = cv::Size(images[0]->cols, images[0]->rows);
	
		key_points_found_flags.resize(images.size());
		key_points.resize(images.size());
		
		find_key_points_mt(images, key_points, key_points_found_flags, pattern, board_settings);

		bool found = std::all_of(
			key_points_found_flags.cbegin(), key_points_found_flags.cend(), [](const bool x) {return x;}
		);

		if(!found)
			return false;

		std::vector<std::vector<cv::Point3f>> phyKeyPoints;
		calImagesKeyPointsPhyCoordinates(board_settings, pattern, images.size(), phyKeyPoints);
		
		params.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
		
		// Use standard calibration method here.
		params.RMS = calibrateCameraRO(
			phyKeyPoints, key_points, imageSize, 
			-1, params.camera_matrix, params.distortions,
			params.rvec, params.tvec, cv::noArray()
		);
		
		return cv::checkRange(params.camera_matrix) && cv::checkRange(params.distortions);
	}
	catch (const std::exception& e)
	{
		logger_.error(e.what());
		throw e;
	}
}


bool stereoCalibration(
	const std::vector<std::shared_ptr<cv::Mat>>& left_images, 
	const std::vector<std::shared_ptr<cv::Mat>>& right_images,
	size_t basis_image_index,
	const CalibrationBoardSettings& board_settings, Pattern pattern,
	PlanarCalibrationParams& left_cam_params, PlanarCalibrationParams& right_cam_params,
	StereoCalibrationParams& stereo_params,
	std::vector<std::vector<cv::Point2f>>& left_key_points, std::vector<bool>& left_flags,
	std::vector<std::vector<cv::Point2f>>& right_key_points, std::vector<bool>& right_flags
)
{
	assert(left_images.size() == right_images.size());
	auto images_count = left_images.size();
	cv::Size image_size = {left_images[0]->cols, left_images[0]->rows};

	// 1. Find pattern key points.
	find_key_points_mt(left_images, left_key_points, left_flags, pattern, board_settings);
	find_key_points_mt(right_images, right_key_points, right_flags, pattern, board_settings);

	bool found = std::all_of(
		left_flags.cbegin(), left_flags.cend(), [](const bool x) {return x;}
	)
	&&
	std::all_of(
		right_flags.cbegin(), right_flags.cend(), [](const bool x) {return x;}
	);

	if(!found)
		return false;

	// 2. Pattern key points physical locations.
	std::vector<std::vector<cv::Point3f>> phyKeyPoints;
	calImagesKeyPointsPhyCoordinates(board_settings, pattern, images_count, phyKeyPoints);
	
	// 3. Calibration
	std::vector<cv::Mat> left_rvecs, right_rvecs, left_tvecs, right_tvecs;
	left_cam_params.RMS = cv::calibrateCamera(
		phyKeyPoints, left_key_points, image_size, 
		left_cam_params.camera_matrix, left_cam_params.distortions, left_rvecs, left_tvecs
	);
	left_cam_params.rvec = left_rvecs[basis_image_index];
	left_cam_params.tvec = left_tvecs[basis_image_index];
	cv::Rodrigues(left_cam_params.rvec, left_cam_params.rmat);

	right_cam_params.RMS = cv::calibrateCamera(
		phyKeyPoints, right_key_points, image_size, 
		right_cam_params.camera_matrix, right_cam_params.distortions, right_rvecs, right_tvecs
	);
	right_cam_params.rvec = right_rvecs[basis_image_index];
	right_cam_params.tvec = right_tvecs[basis_image_index];
	cv::Rodrigues(right_cam_params.rvec, right_cam_params.rmat);
	
	stereo_params.RMS = cv::stereoCalibrate(phyKeyPoints, left_key_points, right_key_points,
		left_cam_params.camera_matrix, left_cam_params.distortions,
		right_cam_params.camera_matrix, right_cam_params.distortions,
		image_size, stereo_params.R, stereo_params.T, stereo_params.E, stereo_params.F
	);

	// 4. Epipolar rectify
	cv::stereoRectify(
		left_cam_params.camera_matrix, left_cam_params.distortions, 
		right_cam_params.camera_matrix, right_cam_params.distortions,
		image_size, stereo_params.R, stereo_params.T, 
		stereo_params.R1, stereo_params.R2, stereo_params.P1, stereo_params.P2, stereo_params.Q
	);

	cv::initUndistortRectifyMap(
		left_cam_params.camera_matrix, left_cam_params.distortions, 
		stereo_params.R1, stereo_params.P1, image_size, CV_16SC2,
		stereo_params.left_map1, stereo_params.left_map2
	);

	cv::initUndistortRectifyMap(
		right_cam_params.camera_matrix, right_cam_params.distortions, 
		stereo_params.R2, stereo_params.P2, image_size, CV_16SC2,
		stereo_params.right_map1, stereo_params.right_map2
	);

	return true;
}
