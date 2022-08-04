#include "CalibrationMethods.h"


constexpr int CHESSBOARD_FLAGS = cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ADAPTIVE_THRESH;
const cv::TermCriteria SUB_PIXEL_SEARCH_CRITERIA(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001);

static Logger& logger_ = GET_LOGGER();


void APIENTRY find_key_points_mt(
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


bool APIENTRY planarCalibration(
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


bool APIENTRY stereoCalibration(
	const std::vector<std::shared_ptr<cv::Mat>>& left_images, 
	const std::vector<std::shared_ptr<cv::Mat>>& right_images,
	size_t basis_image_index,
	const CalibrationBoardSettings& board_settings, Pattern pattern,
	DualViewCalibrationParams& params,
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
	params.left.RMS = cv::calibrateCamera(
		phyKeyPoints, left_key_points, image_size, 
		params.left.camera_matrix, params.left.distortions, left_rvecs, left_tvecs
	);
	params.left.rvec = left_rvecs[basis_image_index];
	params.left.tvec = left_tvecs[basis_image_index];
	cv::Rodrigues(params.left.rvec, params.left.rmat);

	params.right.RMS = cv::calibrateCamera(
		phyKeyPoints, right_key_points, image_size, 
		params.right.camera_matrix, params.right.distortions, right_rvecs, right_tvecs
	);
	params.right.rvec = right_rvecs[basis_image_index];
	params.right.tvec = right_tvecs[basis_image_index];
	cv::Rodrigues(params.right.rvec, params.right.rmat);
	
	params.stereo.RMS = cv::stereoCalibrate(phyKeyPoints, left_key_points, right_key_points,
		params.left.camera_matrix, params.left.distortions,
		params.right.camera_matrix, params.right.distortions,
		image_size, params.stereo.R, params.stereo.T, params.stereo.E, params.stereo.F
	);

	// 4. Epipolar rectify
	cv::stereoRectify(
		params.left.camera_matrix, params.left.distortions, 
		params.right.camera_matrix, params.right.distortions,
		image_size, params.stereo.R, params.stereo.T, 
		params.stereo.R1, params.stereo.R2, params.stereo.P1, params.stereo.P2, params.stereo.Q
	);

	cv::initUndistortRectifyMap(
		params.left.camera_matrix, params.left.distortions, 
		params.stereo.R1, params.stereo.P1, image_size, CV_16SC2,
		params.stereo.left_map1, params.stereo.left_map2
	);

	cv::initUndistortRectifyMap(
		params.right.camera_matrix, params.right.distortions, 
		params.stereo.R2, params.stereo.P2, image_size, CV_16SC2,
		params.stereo.right_map1, params.stereo.right_map2
	);

	return true;
}
