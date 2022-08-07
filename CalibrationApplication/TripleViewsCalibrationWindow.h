#pragma once
#include "AbstractCalibrationWindow.h"
#include "PDRCamera.h"


class TripleViewsCalibrationWindow :
    public AbstractCalibrationWindow
{
private:
	void cameraFrameReadyCallback(cv::InputArray image_data) override;
	void startCalibBoardDetectThread() override;
	void stopCalibBoardDetectThread() override;

protected:
	void shotImage() override;
	void saveImage(const std::string& filename) override;
	void abortCalibration() override;
	void shotCalibImage() override;
	void calibrate(const std::string& folder) override;


public:
	TripleViewsCalibrationWindow(const std::shared_ptr<CameraLib::Camera>& camera, QWidget* parent = nullptr);
};

