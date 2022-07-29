#include "CalibrationPatternMethod.h"

#include <execution>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>


// Corner sub pixel search params
const cv::TermCriteria SUB_PIXEL_SEARCH_CRITERIA(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001);
constexpr int SUB_PIXEL_SEARCH_WINDOW = 11;

// Circle search params (adaptive threshold)
constexpr int ADA_THD_MAX_VALUE = 255;
constexpr int ADA_THD_BLOCK_SIZE = 51;
constexpr int ADA_THD_CONST = -5;

constexpr int EROSION_SIZE = 3;
const cv::Mat element = cv::getStructuringElement(
	cv::MORPH_ELLIPSE,
    cv::Size( 2*EROSION_SIZE + 1, 2*EROSION_SIZE+1 ),
    cv::Point( 1, 1 ) 
);


bool findKeyPoints(
	const cv::Mat& image, std::vector<cv::Point2f>& points, Pattern pattern, const cv::Size& points_count, bool roughly
)
{
	cv::Mat image_detect;
	if(image.channels() == 3)
		cvtColor(image, image_detect, cv::COLOR_RGB2GRAY);
	else
		image.copyTo(image_detect);

	assert(image_detect.elemSize() == 1 && "Gray image element size is not!");

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
			image_detect = 255 - image_detect;
			return cv::findCirclesGrid(image_detect, points_count, points);
			cv::findCirclesGrid(
				image_detect, points_count, points
			);
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
        for( int i = 0; i < key_points_count.height; ++i )
            for( int j = 0; j < key_points_count.width; ++j )
                points_coordinates.emplace_back(cv::Point3f(key_points_phy_interval * j, key_points_phy_interval * i, 0));
        break;
    case Invalid:
		throw std::invalid_argument("Invalid pattern type!");
    default:
        break;
    }
}
