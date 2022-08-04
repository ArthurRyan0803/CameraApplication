#include <execution>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <minwindef.h>

#include "CalibrationPatternMethod.h"


// Corner sub pixel search params
const cv::TermCriteria SUB_PIXEL_SEARCH_CRITERIA(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001);
constexpr int SUB_PIXEL_SEARCH_WINDOW = 11;


bool APIENTRY findKeyPoints(
	const cv::Mat& image, std::vector<cv::Point2f>& points, Pattern pattern, const cv::Size& points_count, bool roughly
)
{
	cv::Mat image_detect;
	if(image.channels() == 3)
		cvtColor(image, image_detect, cv::COLOR_RGB2GRAY);
	else
		image.copyTo(image_detect);

	switch (pattern)
	{
		case Chessboard:
		{
			bool found = findChessboardCorners(
				image_detect, points_count, points, 
				cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ADAPTIVE_THRESH
			);

			if(!found) return false;
			
			if(!roughly)
				cornerSubPix(
					image_detect, points, cv::Size(SUB_PIXEL_SEARCH_WINDOW, SUB_PIXEL_SEARCH_WINDOW), 
					cv::Size(-1, -1), SUB_PIXEL_SEARCH_CRITERIA
				);

			return found;
		}
		
		case CirclesArray:
		{
			return cv::findCirclesGrid(255 - image_detect, points_count, points);
		}

		default:
			throw std::invalid_argument("Unrecognized calibration pattern!");
	}
}


void APIENTRY calKeyPointsPhyCoordinates(
	const CalibrationBoardSettings& settings, Pattern pattern, std::vector<cv::Point3f>& points_coordinates
)
{
	points_coordinates.clear();
	auto count = settings.count();
	
    switch(pattern)
    {
    case CirclesArray:
    case Chessboard:
        for( int r = count.height - 1; r >= 0; --r )
            for( int c = 0; c < count.width; ++c )
                points_coordinates.emplace_back(cv::Point3f(settings.interval * c, settings.interval * r, 0));
        break;
    case Invalid:
		throw std::invalid_argument("Invalid pattern type!");
    default:
        break;
    }
}


void APIENTRY calImagesKeyPointsPhyCoordinates(
	const CalibrationBoardSettings& settings, Pattern pattern, size_t images, std::vector<std::vector<cv::Point3f>>& phyKeyPoints
)
{
	phyKeyPoints.clear();
	phyKeyPoints.resize(1);
	calKeyPointsPhyCoordinates(settings, pattern, phyKeyPoints[0]);
	phyKeyPoints.resize(images, phyKeyPoints[0]);
}
