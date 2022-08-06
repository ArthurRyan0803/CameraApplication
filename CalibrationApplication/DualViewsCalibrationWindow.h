#pragma once

#include <pcl/visualization/pcl_visualizer.h>

#include <qimage.h>
#include <memory>

#include <vtk-9.0/QVTKOpenGLStereoWidget.h>

#include "Camera.h"
#include "ui_DualViewsCalibrationWindow.h"
#include "CalibrationBoardSettings.hpp"
#include "CalibrationMethods.h"


class DualViewsCalibrationWindow : public QMainWindow
{
	Q_OBJECT

public:
	DualViewsCalibrationWindow(
		std::shared_ptr<CameraLib::Camera> camera,
		QWidget *parent = nullptr
	);
	~DualViewsCalibrationWindow() override = default;

private:
	std::shared_ptr<CameraLib::Camera> camera_;
	Ui::DualViewsCalibrationWindow ui_{};
	volatile bool check_calib_board_;
	CalibrationBoardSettings calib_board_settings_;
	volatile Pattern calib_pattern_;
	volatile bool is_calibrating_;
	std::vector<std::shared_ptr<cv::Mat>> left_calib_images_, right_calib_images_;
	std::string calib_files_folder_;
	std::array<int, 2> cam_resolution_{};

	std::array<std::unique_ptr<QImage>, 2> q_images_;
	std::array<std::unique_ptr<cv::Mat>, 2> frame_buffers_;
	std::array<std::vector<cv::Point2f>, 2> frames_key_points_;

	std::array<std::mutex, 2> q_image_mutexes_;
	std::array<std::mutex, 2> frame_buffer_mutexes_;
	std::array<std::mutex, 2> key_points_mutexes_;

	std::array<std::unique_ptr<std::thread>, 2> board_detect_threads_;

	std::unique_ptr<QVTKOpenGLStereoWidget> stereo_widget_;
	std::unique_ptr<pcl::visualization::PCLVisualizer> pcl_visualizer_;

	void cameraFrameReadyCallback(cv::InputArray image_data);

	void startCalibBoardDetectThread();
	void stopCalibBoardDetectThread();
	void findingCalibBoardPattern(int index);

	/* ------ Events ------ */
	//void showEvent(QShowEvent* e) override;
	void closeEvent(QCloseEvent* e) override;
	bool eventFilter(QObject* obj, QEvent* e) override;
	/* ------ Events ------ */

	/* ------ Slot methods ------ */
	void oneShotButtonClicked();
	void captureButtonClicked();
	void detectBoardCheckboxStateChanged();
	void calibPatternChanged();
	void saveImageButtonClicked();

	void calibrationButtonClicked();
	void grabCalibImageButtonClicked();
	void parameterButtonClicked();
	void calibBoardSettingsButtonClicked();
	/* ------ Slot methods ------ */

	void copyFrame(const cv::Mat& frame, int buffer_index);
	bool paintImage(int buffer_index);

	void DualViewsCalibrationWindow::visualizeCalibPlanar(
		const CalibrationBoardSettings& settings, double r, double g, double b, const std::string& id_prefix
	);

	void DualViewsCalibrationWindow::visualizeCamera(
		const PlanarCalibrationParams& camera_params, const cv::Mat& R, const std::string& id,
		double r, double g, double b
	);
};
