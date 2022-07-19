#pragma once

#include <QDialog>
#include "ui_SingleCameraCalibrationDialog.h"
#include "Camera.h"
#include <opencv2/opencv.hpp>
#include "Utils.hpp"

class SingleCameraCalibrationDialog : public QDialog
{
	Q_OBJECT

private:
	enum Pattern { Invalid = 0, Chessboard = 1, CirclesGrid = 2 };
	static std::map<QString, Pattern> patters_map_;

	std::unique_ptr<QImage> q_image_buffer_;					// QImage for painting.
	std::unique_ptr<cv::Mat> frame_buffer_;						// Image for processing.
	std::tuple<bool, std::vector<cv::Point2f>> board_points;	// Key points in the calibration board.
	std::unique_ptr<std::thread> board_detect_thread_;			// The thread to detect points in the calibration board.
	std::array<int, 2> cam_resolution_;
	Pattern calib_pattern_;										// Pattern of calibration board.
	volatile bool check_calib_board_;							// Indicating whether to detect calibration board.
	
	std::mutex q_image_mutex_;									// The r/w access mutex of QImage.
	std::mutex frame_buffer_mutex_;								// The r/w access mutex of frame buffer.
	std::mutex board_corners_mutex_;							// The r/w access mutex of calibration board key points.
	
public:
	SingleCameraCalibrationDialog(std::shared_ptr<Camera> camera, QWidget *parent = nullptr);
	~SingleCameraCalibrationDialog() override = default;

private:
	Ui::SingleCameraCalibrationDialogClass ui_;
	std::shared_ptr<Camera> camera_;

	void showEvent(QShowEvent* e) override;
	void closeEvent(QCloseEvent* e) override;
	bool eventFilter(QObject* obj, QEvent* e) override;

	void grabButtonClicked();
	void captureButtonClicked();
	void detectBoardCheckboxStateChanged();
	void calibPatternChanged();
	
	void cameraFrameReadyCallback(const cv::Mat& frame);

	void startCalibBoardDetectThread();
	void stopCalibBoardDetectThread();
	void findingCalibBoardPattern();
};
