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


bool findCirclesPatterns(
	const cv::Mat& image_gray, const cv::Size& points_count, std::vector<cv::Point2f>& points
)
{
	cv::Mat image_binary;
	cv::adaptiveThreshold(image_gray, image_binary, ADA_THD_MAX_VALUE, 
		cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, ADA_THD_BLOCK_SIZE, ADA_THD_CONST
	);
	
	cv::imwrite("C:\\Users\\ryan\\Desktop\\shot.png", image_gray);
	cv::imwrite("C:\\Users\\ryan\\Desktop\\shot_thd_erode_before.png", image_binary);
	cv::erode(image_binary, image_binary, element);
	cv::imwrite("C:\\Users\\ryan\\Desktop\\shot_thd_erode_after.png", image_binary);
	
	cv::Mat labels, stats, centroids;
	auto count = cv::connectedComponentsWithStats(image_binary, labels, stats, centroids);

	// Roughly filter areas
	//double area_sum = 0;
	std::vector<int> indices;
	for(int i=0; i < count; i++)
	{
		auto width = stats.at<int>(i, 2), height = stats.at<int>(i, 3);
		if(width <= 5 || height <= 5 || abs(width - height) > 5)
			continue;
		
		indices.push_back(i);
		//area_sum += stats.at<int>(i, 4);
	}

	points.clear();
	for(auto i: indices)
	{
		auto x = static_cast<float>(centroids.at<double>(i, 0));
		auto y = static_cast<float>(centroids.at<double>(i, 1));
		points.emplace_back(x, y);
	}

	//double area_avg = area_sum / count;
	
	//cv::Mat image_color;
	//cv::cvtColor(image_gray, image_color, cv::COLOR_GRAY2RGB);
	//for(auto i: indices)
	//{
	//	int x = static_cast<int>(centroids.at<double>(i, 0));
	//	int y = static_cast<int>(centroids.at<double>(i, 1));
	//	cv::Point center(x, y);
	//	auto width = stats.at<int>(i, 2), height = stats.at<int>(i, 3);
	//	auto radius = static_cast<int>(sqrt(pow(width, 2) + pow(height, 2)) / 2);

	//	cv::circle(image_color, center, radius, cv::Scalar(255, 0, 0));		// color: RGB
	//}

	//cv::imwrite("C:\\Users\\ryan\\Desktop\\shot_circles.png", image_color);

	return true;
}


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
			return findCirclesPatterns(
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
