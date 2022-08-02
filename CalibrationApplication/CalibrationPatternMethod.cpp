#include "CalibrationPatternMethod.h"

#include <execution>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>


// Corner sub pixel search params
const cv::TermCriteria SUB_PIXEL_SEARCH_CRITERIA(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001);
constexpr int SUB_PIXEL_SEARCH_WINDOW = 11;


bool findKeyPoints(
	const cv::Mat& image, std::vector<cv::Point2f>& points, Pattern pattern, const cv::Size& points_count, bool roughly
)
{
	switch (pattern)
	{
		case Chessboard:
		{
			bool found = findChessboardCorners(
				image, points_count, points, 
				cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ADAPTIVE_THRESH
			);
			if(!found) return false;
			if(!roughly)
				cornerSubPix(
					image, points, cv::Size(SUB_PIXEL_SEARCH_WINDOW, SUB_PIXEL_SEARCH_WINDOW), 
					cv::Size(-1, -1), SUB_PIXEL_SEARCH_CRITERIA
				);
			return found;
		}
		
		case CirclesArray:
		{
			if(image.channels() == 3)
				cvtColor(image, image, cv::COLOR_RGB2GRAY);
			
			return cv::findCirclesGrid(255 - image, points_count, points);
		}
		default:
			throw std::invalid_argument("Unrecognized calibration pattern!");
	}
}


void calKeyPointsPhyCoordinates(
	const cv::Size& key_points_count, float key_points_phy_interval, std::vector<cv::Point3f>& points_coordinates, Pattern pattern
)
{
	points_coordinates.clear();
	
    switch(pattern)
    {
    case CirclesArray:
    case Chessboard:
        for( int i = key_points_count.height - 1; i >= 0; --i )
            for( int j = 0; j < key_points_count.width; ++j )
                points_coordinates.emplace_back(cv::Point3f(key_points_phy_interval * j, key_points_phy_interval * i, 0));
        break;
    case Invalid:
		throw std::invalid_argument("Invalid pattern type!");
    default:
        break;
    }
}
