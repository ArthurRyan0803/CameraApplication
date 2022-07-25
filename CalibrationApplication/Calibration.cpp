#include "Calibration.h"


constexpr int CHESSBOARD_FLAGS = cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ADAPTIVE_THRESH;
// findChessboardCorners(gray_image, chessboard_size, points, flag);
const cv::TermCriteria SUB_PIXEL_SEARCH_CRITERIA(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001);


bool findKeyPoints(
	const cv::Mat& calibBoardImage, std::vector<cv::Point2f>& keyPoints, Pattern pattern, const cv::Size& keyPointsCount,
	const cv::Size& subPixelSearchWindowSize
)
{
	cv::Mat grayImage;
	cv::cvtColor(calibBoardImage, grayImage, cv::COLOR_RGB2GRAY);

	assert(grayImage.elemSize() == 1 && "Gray calibBoardImage element size is not !");

	int flag;
	if(pattern == Chessboard)
		flag = CHESSBOARD_FLAGS;
	else
		throw std::invalid_argument("The flag is not supported!");

	bool found = cv::findChessboardCorners(grayImage, keyPointsCount, keyPoints, flag);
	if(!found)
		return false;

	cv::cornerSubPix(grayImage, keyPoints, subPixelSearchWindowSize, cv::Size(-1, -1), SUB_PIXEL_SEARCH_CRITERIA);
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


bool runCalibration(
	const std::vector<std::shared_ptr<cv::Mat>>& images, 
	cv::Mat& cameraIntrinsicMatrix, cv::Mat& distortCoefficients, cv::Mat& rotationMatrix, cv::Mat& translationVector,
	float keyPointsPhyInterval, const cv::Size& keyPointsCount, const cv::Size& subPixelSearchWindowSize, Pattern pattern,
	float& RMS
)
{
	auto imageSize = cv::Size(images[0]->cols, images[0]->rows);

	std::vector<std::vector<cv::Point2f>> keyPointsTable(images.size());
	std::vector<bool> foundFlags(images.size());
	std::vector<std::shared_ptr<std::thread>> threads;

	for(size_t i=0; i<images.size(); i++)
	{
		std::shared_ptr<std::thread> thread(
			new std::thread(
				[i, &images, &keyPointsTable, &foundFlags, pattern, &keyPointsCount, &subPixelSearchWindowSize]()
				{
					foundFlags[i] = findKeyPoints(
						*images[i], keyPointsTable[i], pattern, 
						keyPointsCount, subPixelSearchWindowSize
					);
				}
			)
		);
		threads.push_back(thread);
	}

	for(auto& thread: threads)
		thread->join();

	bool found = std::all_of(
		foundFlags.cbegin(), foundFlags.cend(), [](const bool x) {return x;}
	);

	if(!found)
		return false;

	std::vector<std::vector<cv::Point3f>> phyKeyPoints(1);
	calKeyPointsPhyCoordinates(keyPointsCount, keyPointsPhyInterval, phyKeyPoints[0], pattern);
	phyKeyPoints.resize(images.size(), phyKeyPoints[0]);
	
	cameraIntrinsicMatrix = cv::Mat::eye(3, 3, CV_64F);
	
	// Use standard calibration method here.
	RMS = cv::calibrateCameraRO(
		phyKeyPoints, keyPointsTable, imageSize, -1, cameraIntrinsicMatrix, distortCoefficients,
		rotationMatrix, translationVector, cv::noArray()
	);
	
	return cv::checkRange(cameraIntrinsicMatrix) && cv::checkRange(distortCoefficients);
}
