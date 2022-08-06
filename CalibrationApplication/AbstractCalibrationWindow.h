#pragma once

#include <memory>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtk-9.0/QVTKOpenGLStereoWidget.h>

#include "Camera.h"
#include "ui_AbstractCalibrationWindow.h"
#include "CalibrationBoardSettings.hpp"
#include "CalibrationMethods.h"


class AbstractCalibrationWindow : public QMainWindow
{
	Q_OBJECT

public:
	AbstractCalibrationWindow(
		const std::shared_ptr<CameraLib::Camera>& camera,
		QWidget *parent = nullptr
	);
	~AbstractCalibrationWindow() override = default;

protected:
	Ui::AbstractCalibrationWindowClass ui_{};
	volatile Pattern calib_pattern_;
	CalibrationBoardSettings calib_board_settings_;
	std::string calib_files_folder_;
	std::array<int, 2> cam_resolution_{};
	std::shared_ptr<CameraLib::Camera> camera_;

	/* ------ Events ------ */
	//void showEvent(QShowEvent* e) override;
	void closeEvent(QCloseEvent* e) override;

	virtual void shotImage() = 0;
	virtual void saveImage(const std::string& filename) = 0;
	virtual void abortCalibration() = 0;
	virtual void shotCalibImage() = 0;
	virtual void calibrate(const std::string& folder) = 0;

private:

	volatile bool is_calibrating_;
	
	virtual void cameraFrameReadyCallback(cv::InputArray image_data) = 0;

	virtual void startCalibBoardDetectThread() = 0;
	virtual void stopCalibBoardDetectThread() = 0;

	//void findingCalibBoardPattern(int index);
	
	/* ------ Events ------ */

	/* ------ Slot methods ------ */
	void oneShotButtonClicked();
	void captureButtonClicked();
	void detectBoardCheckboxStateChanged();
	void calibPatternChanged();
	void saveImageButtonClicked();

	void calibrationButtonClicked();
	void grabCalibImageButtonClicked();
	void parameterButtonClicked() const;
	void calibBoardSettingsButtonClicked();
	void finishCalibrationImageGrabbing();

	/* ------ Slot methods ------ */

	//void copyFrame(const cv::Mat& frame, int buffer_index);
	//bool paintImage(int buffer_index);

	//void visualizeCalibPlanar(
	//	const CalibrationBoardSettings& settings, double r, double g, double b, const std::string& id_prefix
	//);

	//void visualizeCamera(
	//	const PlanarCalibrationParams& camera_params, const cv::Mat& R, const std::string& id,
	//	double r, double g, double b
	//);
};
