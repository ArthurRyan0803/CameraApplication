#include "TripleViewsCalibrationWindow.h"

TripleViewsCalibrationWindow::TripleViewsCalibrationWindow(const std::shared_ptr<CameraLib::Camera>& camera, QWidget* parent)
	: AbstractCalibrationWindow(camera, parent)
{

}

void TripleViewsCalibrationWindow::cameraFrameReadyCallback(cv::InputArray image_data)
{

}

void TripleViewsCalibrationWindow::startCalibBoardDetectThread()
{

}

void TripleViewsCalibrationWindow::stopCalibBoardDetectThread()
{

}

void TripleViewsCalibrationWindow::shotImage()
{
	std::vector<cv::Mat> images(3);
	camera_->oneShot(images);
}


void TripleViewsCalibrationWindow::saveImage(const std::string& filename)
{

}


void TripleViewsCalibrationWindow::abortCalibration()
{

}


void TripleViewsCalibrationWindow::shotCalibImage()
{

}


void TripleViewsCalibrationWindow::calibrate(const std::string& folder)
{

}
