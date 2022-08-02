#pragma once

#include <mutex>
#include <QDialog>
#include <QFileDialog>
#include <QPainter>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include "AbstractCamerasFactory.h"
#include "CalibrationMethods.h"
#include "Camera.h"
#include "Logger.hpp"
#include "MainNavigationWindow.h"
#include "ui_SingleViewCalibrationWindow.h"
#include "CalibrationSettingDialog.h"
#include "Utils.hpp"
#include "UICommon.h"


class SingleViewCalibrationWindow : public QMainWindow
{
	Q_OBJECT

private:

	std::unique_ptr<QImage> q_image_buffer_;					// QImage for painting.
	std::unique_ptr<cv::Mat> frame_buffer_;						// Image for processing.
	std::tuple<bool, std::vector<cv::Point2f>> pattern_points_;	// Key points in the planarCalibration board.
	std::unique_ptr<std::thread> board_detect_thread_;			// The thread to detect points in the planarCalibration board.
	std::array<int, 2> cam_resolution_;
	std::vector<std::shared_ptr<cv::Mat>> calib_images_;		// Store images for planarCalibration.

	Pattern calib_pattern_;										// Pattern of planarCalibration board.
	volatile bool check_calib_board_;							// Indicating whether to detect planarCalibration board.
	bool is_calibrating;										// Indicating whether this dialog is under planarCalibration procedure or not.
	std::string calib_files_folder_;							// The path of folder to store the planarCalibration files.
	
	std::mutex q_image_mutex_;									// The r/w access mutex of QImage.
	std::mutex frame_buffer_mutex_;								// The r/w access mutex of frame buffer.
	std::mutex pattern_points_mutex_;							// The r/w access mutex of planarCalibration board key points.

	CalibrationBoardSettings calib_board_settings_;				

	Logger& logger_;
	
public:
	SingleViewCalibrationWindow(std::shared_ptr<Camera> camera, QWidget *parent = nullptr);
	~SingleViewCalibrationWindow() override = default;

private:
	Ui::SingleViewCalibrationWindowClass ui_{};
	std::shared_ptr<Camera> camera_;
	
	void cameraFrameReadyCallback(cv::InputArray image_data);

	void startCalibBoardDetectThread();
	void stopCalibBoardDetectThread();
	void findingCalibBoardPattern();

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
};
