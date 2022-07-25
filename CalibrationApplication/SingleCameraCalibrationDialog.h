#pragma once

#include <QDialog>
#include "ui_SingleCameraCalibrationDialog.h"
#include <opencv2/opencv.hpp>

#include "Calibration.h"
#include "Utils.hpp"
#include "Camera.h"
#include "Logger.hpp"


class SingleCameraCalibrationDialog : public QDialog, std::enable_shared_from_this<SingleCameraCalibrationDialog>
{
	Q_OBJECT

private:
	
	static std::map<QString, Pattern> patters_map_;

	std::unique_ptr<QImage> q_image_buffer_;					// QImage for painting.
	std::unique_ptr<cv::Mat> frame_buffer_;						// Image for processing.
	std::tuple<bool, std::vector<cv::Point2f>> pattern_points_;	// Key points in the runCalibration board.
	std::unique_ptr<std::thread> board_detect_thread_;			// The thread to detect points in the runCalibration board.
	std::array<int, 2> cam_resolution_;
	std::vector<std::shared_ptr<cv::Mat>> calib_images_;		// Store images for runCalibration.

	Pattern calib_pattern_;										// Pattern of runCalibration board.
	volatile bool check_calib_board_;							// Indicating whether to detect runCalibration board.
	bool is_calibrating;										// Indicating whether this dialog is under runCalibration procedure or not.
	std::string calib_files_folder_;							// The path of folder to store the runCalibration files.
	
	std::mutex q_image_mutex_;									// The r/w access mutex of QImage.
	std::mutex frame_buffer_mutex_;								// The r/w access mutex of frame buffer.
	std::mutex pattern_points_mutex_;							// The r/w access mutex of runCalibration board key points.

	Logger& logger_;
	
public:
	SingleCameraCalibrationDialog(std::shared_ptr<Camera> camera, QWidget *parent = nullptr);
	~SingleCameraCalibrationDialog() override;

private:
	Ui::SingleCameraCalibrationDialogClass ui_;
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
	/* ------ Slot methods ------ */
};
