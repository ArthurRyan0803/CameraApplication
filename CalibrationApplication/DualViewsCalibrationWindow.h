#pragma once

#include <qimage.h>
#include <memory>
#include <QFileDialog>

#include "Camera.h"
#include "ui_DualViewsCalibrationWindow.h"
#include "CalibrationBoardSettings.hpp"
#include "CalibrationMethods.h"
#include "Logger.hpp"


class DualViewsCalibrationWindow : public QMainWindow
{
	Q_OBJECT

public:
	DualViewsCalibrationWindow(std::shared_ptr<Camera> camera, QWidget *parent = nullptr);
	~DualViewsCalibrationWindow() = default;

private:
	Ui::DualViewsCalibrationWindow ui_;
	Logger& logger_;
	volatile bool check_calib_board_;
	CalibrationBoardSettings calib_board_settings_;
	volatile Pattern calib_pattern_;

	std::shared_ptr<Camera> camera_;
	std::array<int, 2> cam_resolution_;

	std::array<std::unique_ptr<QImage>, 2> q_images_;
	std::array<std::unique_ptr<cv::Mat>, 2> frame_buffers_;
	std::array<std::vector<cv::Point2f>, 2> frames_key_points_;

	std::array<std::mutex, 2> q_image_mutexes_;
	std::array<std::mutex, 2> frame_buffer_mutexes_;
	std::array<std::mutex, 2> key_points_mutexes_;

	std::array<std::unique_ptr<std::thread>, 2> board_detect_threads_;

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
};
