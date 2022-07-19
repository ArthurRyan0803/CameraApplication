#include "SingleCameraCalibrationDialog.h"
#include "AbstractCamerasFactory.h"
#include "MainNavigationWindow.h"
#include <QPainter>
#include <mutex>

#include "Logger.hpp"
#include "Utils.hpp"
#include <opencv2/calib3d.hpp>


std::map<QString, SingleCameraCalibrationDialog::Pattern> SingleCameraCalibrationDialog::patters_map_ {
	{ "Chessboard", Chessboard}
};

SingleCameraCalibrationDialog::SingleCameraCalibrationDialog(
	std::shared_ptr<Camera> camera, QWidget* parent
): QDialog(parent), q_image_buffer_(nullptr), check_calib_board_(false), cam_resolution_({}), camera_(std::move(camera))
{
	ui_.setupUi(this);

	connect(ui_.btnGrab, &QPushButton::clicked, this, &SingleCameraCalibrationDialog::grabButtonClicked);
	connect(ui_.btnCapture, &QPushButton::clicked, this, &SingleCameraCalibrationDialog::captureButtonClicked);
	connect(ui_.btnExit, &QPushButton::clicked, this, &SingleCameraCalibrationDialog::close);
	connect(ui_.cbCalibPattern, &QComboBox::currentTextChanged, this, &SingleCameraCalibrationDialog::calibPatternChanged);
	connect(ui_.cbDetectCalibBoard, &QCheckBox::stateChanged, this, &SingleCameraCalibrationDialog::detectBoardCheckboxStateChanged);
	
	camera_->setCapturingStartCallback([]{ Logger::log("Start capture!" );});
	camera_->setCapturingStopCallback([]{ Logger::log("Stop capture!" );});
	camera_->setFrameReadyCallback(std::bind(&SingleCameraCalibrationDialog::cameraFrameReadyCallback, this, std::placeholders::_1));
	
	ui_.canvas->installEventFilter(this);

	for(auto& pair: patters_map_)
	{
		ui_.cbCalibPattern->addItem(pair.first);
	}
}

void SingleCameraCalibrationDialog::cameraFrameReadyCallback(const cv::Mat& frame)
{
	{
		std::lock_guard frame_lock(frame_buffer_mutex_);
		if(!frame_buffer_)
			frame_buffer_ = std::make_unique<cv::Mat>();
		frame.copyTo(*frame_buffer_);
	}
	{
		std::lock_guard q_image_lock(q_image_mutex_);
		if(!q_image_buffer_)
			Utils::mat2qimage(frame, q_image_buffer_);
		Utils::updateImageData(frame, *q_image_buffer_);
	}
	ui_.canvas->update();
}

void SingleCameraCalibrationDialog::findingCalibBoardPattern()
{
	Logger::log("Start finding chessboard");

	cv::Mat gray_image;
	auto flag = cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ADAPTIVE_THRESH;
	cv::Size2i chessboard_size(5, 7);

	while(check_calib_board_)
	{
		{
			std::lock_guard lock(frame_buffer_mutex_);
			if(!frame_buffer_)
			{
				throw std::logic_error("Why frame_buffer_ is empty?");
			}
			frame_buffer_->copyTo(gray_image);
			cvtColor(*frame_buffer_, gray_image, cv::COLOR_RGB2GRAY);
		}
		
		{
			std::vector<cv::Point2f> points;

			bool success;
			switch(calib_pattern_)
			{
			case Chessboard:
				success = findChessboardCorners(gray_image, chessboard_size, points, flag);
				break;
			default:
				throw std::runtime_error((boost::format("Unrecognized calibration board type: %1%") % calib_pattern_).str());
			}
			
			board_corners_mutex_.lock();
			std::get<0>(board_points) = success;
			std::get<1>(board_points) = points;
			board_corners_mutex_.unlock();

			std::string str = success ? "success" : "fails";
			Logger::log("Find chessboard " + str);
		}
	}

	Logger::log("Stop finding chessboard");
}


void SingleCameraCalibrationDialog::startCalibBoardDetectThread()
{
	if(board_detect_thread_)
		throw std::logic_error("Why the calibration board detection thread is still alive?");

	check_calib_board_ = true;
	board_detect_thread_ = 
		std::make_unique<std::thread>(std::bind(&SingleCameraCalibrationDialog::findingCalibBoardPattern, this));
}

void SingleCameraCalibrationDialog::stopCalibBoardDetectThread()
{
	if(!board_detect_thread_)
		return;

	check_calib_board_ = false;
	board_detect_thread_->join();
	board_detect_thread_.reset();
}

/* -------------------- Events --------------------*/

void SingleCameraCalibrationDialog::showEvent(QShowEvent* e)
{
	QDialog::showEvent(e);
	camera_->open();
	cam_resolution_ = camera_->getResolution();
}

void SingleCameraCalibrationDialog::closeEvent(QCloseEvent* e)
{
	QDialog::closeEvent(e);
	camera_->close();
	if(board_detect_thread_)
	{
		check_calib_board_ = false;
		board_detect_thread_->join();
		board_detect_thread_.reset();
	}
}

bool SingleCameraCalibrationDialog::eventFilter(QObject* obj, QEvent* e)
{
	if(e->type() == QEvent::Paint && obj == ui_.canvas)
	{
	    QPainter painter(ui_.canvas);
		std::array<int, 4> paint_region{};
	    painter.setRenderHint(QPainter::Antialiasing, true);
		auto pen = QPen(QBrush(QColor::fromRgb(255, 0, 0)), 10.0f);
		painter.setPen(pen);

		{
			// Paint image.
			std::lock_guard lock(this->q_image_mutex_);
			
			if(q_image_buffer_)
			{
				Utils::getImagePaintRegion(
					cam_resolution_, 
					{ui_.canvas->width(), ui_.canvas->height()}, 
					paint_region
				);
				QRectF region_rect_f(paint_region[0], paint_region[1], paint_region[2], paint_region[3]);
				painter.drawImage(region_rect_f, *q_image_buffer_);
			}
			else
			{
				return false;
			}
		}

		// Paint calibration board points.
		if(check_calib_board_)
		{
			board_corners_mutex_.lock();
			auto board_corners_copy = board_points;
			board_corners_mutex_.unlock();

			if(std::get<0>(board_corners_copy))	// These points are valid!
			{
				for(auto& point: std::get<1>(board_corners_copy))
				{
					auto point_in_widget =  
						Utils::convertPaintPositions(cam_resolution_, paint_region, {point.x, point.y});

					painter.drawPoint(point_in_widget[0], point_in_widget[1]);
				}
			}
		}

		return true;
	}

	return QDialog::eventFilter(obj, e);
}

/* -------------------- Events --------------------*/

/* -------------------- Slot methods --------------------*/

void SingleCameraCalibrationDialog::calibPatternChanged()
{
	auto box = dynamic_cast<QComboBox*>(sender());
	auto text  = box->currentText();
	calib_pattern_ = patters_map_[text];
}

void SingleCameraCalibrationDialog::detectBoardCheckboxStateChanged()
{
	auto checkbox = dynamic_cast<QCheckBox*>(sender());
	if(checkbox->isChecked())
	{
		startCalibBoardDetectThread();
	}
	else
	{
		stopCalibBoardDetectThread();
	}
}

void SingleCameraCalibrationDialog::grabButtonClicked()
{
	std::lock_guard frame_lock(frame_buffer_mutex_);
	if(!frame_buffer_)
	{
		frame_buffer_ = std::make_unique<cv::Mat>();
	}
	if(camera_->oneShot(*frame_buffer_))
	{
		{
			std::lock_guard q_image_lock(q_image_mutex_);
			if(!q_image_buffer_)
				Utils::mat2qimage(*frame_buffer_, q_image_buffer_);
			Utils::updateImageData(*frame_buffer_, *q_image_buffer_);
		}
		ui_.canvas->update();
	}
	else
	{
		throw std::runtime_error("Failed to grab image!");
	}
}

void SingleCameraCalibrationDialog::captureButtonClicked()
{
	if(camera_->isCapturing())
	{
		camera_->stopCapture();
		ui_.btnCapture->setText("Start capturing");
		ui_.btnGrab->setEnabled(true);
		ui_.cbDetectCalibBoard->setChecked(false);	// This method will trigger slot [ detectBoardCheckboxStateChanged() ].
		ui_.cbDetectCalibBoard->setEnabled(false);
	}
	else
	{
		camera_->startCapture();
		ui_.btnCapture->setText("Stop capturing");
		ui_.btnGrab->setEnabled(false);
		ui_.cbDetectCalibBoard->setEnabled(true);
	}
}

/* -------------------- Slot methods --------------------*/
