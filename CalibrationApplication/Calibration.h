#pragma once

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <thread>


enum Pattern { Invalid = 0, Chessboard = 1, CirclesGrid = 2 };


bool runCalibration(
	const std::vector<std::shared_ptr<cv::Mat>>& images, 
	cv::Mat& cameraIntrinsicMatrix, cv::Mat& distortCoefficients, cv::Mat& rotationMatrix, cv::Mat& translationVector,
	float keyPointsPhyInterval, const cv::Size& keyPointsCount, const cv::Size& subPixelSearchWindowSize, Pattern pattern,
	float& RMS
);