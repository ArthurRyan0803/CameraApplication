#include "Calibration.h"


constexpr int CHESSBOARD_FLAGS = cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ADAPTIVE_THRESH;
const cv::TermCriteria SUB_PIXEL_SEARCH_CRITERIA(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001);

static Logger& logger_ = GET_LOGGER();

bool findKeyPoints(
	const cv::Mat& calibBoardImage, std::vector<cv::Point2f>& keyPoints, Pattern pattern, const cv::Size& keyPointsCount,
	const cv::Size& subPixelSearchWindowSize
)
{
	cv::Mat grayImage;
	cvtColor(calibBoardImage, grayImage, cv::COLOR_RGB2GRAY);

	assert(grayImage.elemSize() == 1 && "Gray calibBoardImage element size is not!");

	int flag;
	if(pattern == Chessboard)
		flag = CHESSBOARD_FLAGS;
	else
		throw std::invalid_argument("The flag is not supported!");

	bool found = findChessboardCorners(grayImage, keyPointsCount, keyPoints, flag);
	if(!found)
		return false;

	cornerSubPix(grayImage, keyPoints, subPixelSearchWindowSize, cv::Size(-1, -1), SUB_PIXEL_SEARCH_CRITERIA);
	return true;
}


void calKeyPointsPhyCoordinates(
	const cv::Size& keyPointsCount, float keyPointsPhyInterval, std::vector<cv::Point3f>& points, Pattern pattern
)
{
	points.clear();
	
    switch(pattern)
    {
    case Chessboard:
        for( int i = 0; i < keyPointsCount.height; ++i )
            for( int j = 0; j < keyPointsCount.width; ++j )
                points.emplace_back(cv::Point3f(keyPointsPhyInterval * j, keyPointsPhyInterval * i, 0));
        break;
    default:
        break;
    }
}


bool planarCalibration(
	const std::vector<std::shared_ptr<cv::Mat>>& images, 
	cv::Mat& intrinsic_matrix, cv::Mat& distort_coefficients, cv::Mat& rotation_matrix, cv::Mat& translation_vector,
	std::vector<std::vector<cv::Point2f>>& key_points, std::vector<bool>& key_points_found_flags,
	float key_points_phy_interval, const cv::Size& key_points_count, const cv::Size& sub_pixel_search_window_size, 
	Pattern pattern, double& RMS
)
{
	try
	{
		auto imageSize = cv::Size(images[0]->cols, images[0]->rows);
	
		key_points_found_flags.resize(images.size());
		key_points.resize(images.size());
		std::vector<std::shared_ptr<std::thread>> threads;

		// 1. Dispatch threads to find key points in calibration board.
		for(size_t i=0; i<images.size(); i++)
		{
			std::shared_ptr<std::thread> thread(
				new std::thread(
					[i, &images, &key_points, &key_points_found_flags, pattern, &key_points_count, &sub_pixel_search_window_size]()
					{
						key_points[i].clear();
						try
						{
							key_points_found_flags[i] = findKeyPoints(
								*images[i], key_points[i], pattern, key_points_count, sub_pixel_search_window_size
							);
						}
						catch (const std::exception& e)
						{
							logger_.error(e.what());
							throw e;
						}
					}
				)
			);
			threads.push_back(thread);
		}

		for(auto& thread: threads)
			thread->join();

		bool found = std::all_of(
			key_points_found_flags.cbegin(), key_points_found_flags.cend(), [](const bool x) {return x;}
		);

		if(!found)
			return false;

		std::vector<std::vector<cv::Point3f>> phyKeyPoints(1);
		calKeyPointsPhyCoordinates(key_points_count, key_points_phy_interval, phyKeyPoints[0], pattern);
		phyKeyPoints.resize(images.size(), phyKeyPoints[0]);
		
		intrinsic_matrix = cv::Mat::eye(3, 3, CV_64F);
		
		// Use standard calibration method here.
		RMS = calibrateCameraRO(
			phyKeyPoints, key_points, imageSize, 
			-1, intrinsic_matrix, distort_coefficients,
			rotation_matrix, translation_vector, cv::noArray()
		);
		
		return checkRange(intrinsic_matrix) && checkRange(distort_coefficients);
	}
	catch (const std::exception& e)
	{
		logger_.error(e.what());
		throw e;
	}
}
