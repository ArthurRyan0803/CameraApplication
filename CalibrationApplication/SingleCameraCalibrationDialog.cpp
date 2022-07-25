#include <opencv2/calib3d.hpp>
#include <QPainter>
#include <mutex>
#include <QFileDialog>
#include <QDialog>
#include <boost/filesystem.hpp>

#include "SingleCameraCalibrationDialog.h"
#include "AbstractCamerasFactory.h"
#include "MainNavigationWindow.h"
#include "Logger.hpp"
#include "Utils.hpp"


//namespace fs=boost::filesystem;

constexpr int KEY_POINTS_PHY_INTERVAL = 50;		// The physical interval of key points. (Unit: mm)
constexpr int KEY_POINTS_HORIZONTAL_COUNT = 7;
constexpr int KEY_POINTS_VERTICAL_COUNT = 5;
constexpr int MAX_CALIB_IMAGES_COUNT = 8;
constexpr float PATTERN_PHY_WIDTH = KEY_POINTS_PHY_INTERVAL * KEY_POINTS_HORIZONTAL_COUNT;

constexpr int SUB_PIXEL_SEARCH_WINDOW = 11;
const cv::TermCriteria SUB_PIXEL_SEARCH_CRITERIA(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001);


std::map<QString, Pattern> SingleCameraCalibrationDialog::patters_map_ {
	{ "Chessboard", Chessboard }
};

SingleCameraCalibrationDialog::SingleCameraCalibrationDialog(
	std::shared_ptr<Camera> camera, QWidget* parent
):
QDialog(parent), q_image_buffer_(nullptr), cam_resolution_({}), check_calib_board_(false), is_calibrating(false),
camera_(std::move(camera)), logger_(Logger::instance(__FILE__))
{
	ui_.setupUi(this);

	connect(ui_.btnOneShot, &QPushButton::clicked, this, &SingleCameraCalibrationDialog::oneShotButtonClicked);
	connect(ui_.btnCapture, &QPushButton::clicked, this, &SingleCameraCalibrationDialog::captureButtonClicked);
	connect(ui_.btnExit, &QPushButton::clicked, this, &SingleCameraCalibrationDialog::close);
	connect(ui_.cmbCalibPattern, &QComboBox::currentTextChanged, this, &SingleCameraCalibrationDialog::calibPatternChanged);
	connect(ui_.ckbDetectCalibBoard, &QCheckBox::stateChanged, this, &SingleCameraCalibrationDialog::detectBoardCheckboxStateChanged);
	connect(ui_.btnSaveImage, &QPushButton::clicked, this, &SingleCameraCalibrationDialog::saveImageButtonClicked);
	connect(ui_.btnCalibration, &QPushButton::clicked, this, &SingleCameraCalibrationDialog::calibrationButtonClicked);
	connect(ui_.btnGrabCalibImage, &QPushButton::clicked, this, &SingleCameraCalibrationDialog::grabCalibImageButtonClicked);
	connect(ui_.btnParameter, &QPushButton::clicked, this, &SingleCameraCalibrationDialog::parameterButtonClicked);

	camera_->setCapturingStartCallback([this]{ logger_.debug("Start capture!" );});
	camera_->setCapturingStopCallback([this]{ logger_.debug("Stop capture!" );});
	camera_->setFrameReadyCallback([this](cv::InputArray data) { cameraFrameReadyCallback(data); });
	
	ui_.canvas->installEventFilter(this);

	for(auto& pair: patters_map_)
		ui_.cmbCalibPattern->addItem(pair.first);

	camera_->open();
	cam_resolution_ =  camera_->getCurrentResolution();
}

SingleCameraCalibrationDialog::~SingleCameraCalibrationDialog()
{
	logger_.debug("SingleCameraCalibrationDialog deconstruct!");
}


void SingleCameraCalibrationDialog::cameraFrameReadyCallback(cv::InputArray image_data)
{
	if(image_data.empty())
		return;

	{
		std::lock_guard frame_lock(frame_buffer_mutex_);
		if(!frame_buffer_)
			frame_buffer_ = std::make_unique<cv::Mat>(cam_resolution_[1], cam_resolution_[0], camera_->getPixelType());
		image_data.copyTo(*frame_buffer_);

		{
			std::lock_guard q_image_lock(q_image_mutex_);
			if(!q_image_buffer_)
				Utils::createQImage(*frame_buffer_, q_image_buffer_);

			Utils::updateImageData(*frame_buffer_, *q_image_buffer_);
		}
	}

	QMetaObject::invokeMethod(this, [this](){ ui_.canvas->update(); }, Qt::QueuedConnection);
}


void SingleCameraCalibrationDialog::findingCalibBoardPattern()
{
	try
	{
		logger_.debug("Start finding runCalibration board pattern!");

		cv::Mat image_to_detect;
		auto flag = cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_ADAPTIVE_THRESH;
		cv::Size2i chessboard_size(5, 7);
		
		// If the frame_buffer_ is empty, wait the camera to grab image.
		while(true)
		{
			bool ready;
			{
				std::lock_guard lock(frame_buffer_mutex_);
				ready = frame_buffer_ != nullptr;
			}
			if(ready)
				break;

			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}

		while(check_calib_board_)
		{
			{
				std::lock_guard lock(frame_buffer_mutex_);
				if(!frame_buffer_)
					throw std::logic_error("Why frame_buffer_ is empty?");

				if(frame_buffer_->elemSize() == 3)
				{
					cvtColor(*frame_buffer_, image_to_detect, cv::COLOR_RGB2GRAY);
				}
				else if(frame_buffer_->elemSize() == 1)
				{
					frame_buffer_->copyTo(image_to_detect);
				}
				else
				{
					throw std::runtime_error("Unrecognized element size type: " + frame_buffer_->elemSize());
				}
			}

			std::vector<cv::Point2f> points;

			bool found;
			switch(calib_pattern_)
			{
			case Chessboard:
				found = findChessboardCorners(image_to_detect, chessboard_size, points, flag);
				break;
			default:
				throw std::runtime_error((boost::format("Unrecognized runCalibration board type: %1%") % calib_pattern_).str());
			}
			
			{
				std::lock_guard lock(pattern_points_mutex_);
				std::get<0>(pattern_points_) = found;
				std::get<1>(pattern_points_) = points;
			}

			std::string str = found ? "success" : "fails";
			logger_.debug("Find chessboard " + str);
		}

		logger_.debug("Stop finding runCalibration board pattern!");
	}
	catch(const std::exception& e)
	{
		logger_.error(e.what());
		throw e;
	}
}


void SingleCameraCalibrationDialog::startCalibBoardDetectThread()
{
	if(board_detect_thread_)
		throw std::logic_error("Why the runCalibration board detection thread is still alive?");

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

		// Paint runCalibration board points.
		if(check_calib_board_)
		{
			pattern_points_mutex_.lock();
			auto board_corners_copy = pattern_points_;
			pattern_points_mutex_.unlock();

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

void SingleCameraCalibrationDialog::oneShotButtonClicked()
{
	{
		std::lock_guard lock(frame_buffer_mutex_);
		if(!frame_buffer_)
			frame_buffer_ = std::make_unique<cv::Mat>();
		
		camera_->oneShot(*frame_buffer_);

		{
			std::lock_guard q_image_lock(q_image_mutex_);
			if(!q_image_buffer_)
				Utils::createQImage(*frame_buffer_, q_image_buffer_);
			Utils::updateImageData(*frame_buffer_, *q_image_buffer_);
		}
	}

	ui_.canvas->update();
	ui_.btnSaveImage->setEnabled(true);
}

void SingleCameraCalibrationDialog::captureButtonClicked()
{
	if(camera_->isCapturing())
	{
		camera_->stopCapture();
		ui_.btnCapture->setText("Start capturing");
		ui_.btnOneShot->setEnabled(true);
		ui_.ckbDetectCalibBoard->setChecked(false);	// This method will trigger slot [ detectBoardCheckboxStateChanged() ]
	}
	else
	{
		camera_->startCapture();
		ui_.btnCapture->setText("Stop capturing");
		ui_.btnOneShot->setEnabled(false);
		ui_.ckbDetectCalibBoard->setEnabled(true);
		ui_.btnSaveImage->setEnabled(true);
	}
}

void SingleCameraCalibrationDialog::saveImageButtonClicked()
{
	std::unique_ptr<cv::Mat> frame_copy;
	{
		std::lock_guard lock(frame_buffer_mutex_);
		if(!frame_buffer_)
			throw std::logic_error("Cannot find any captured frame! Please grab images first!");
		frame_copy = std::make_unique<cv::Mat>();
		*frame_copy = frame_buffer_->clone();
	}

	auto filename = QFileDialog::getSaveFileName(this, "Save image", "", "*.png");
	if(!filename.isEmpty())
	{
		logger_.debug("Save image to " + filename.toStdString());
		cv::imwrite(filename.toStdString(), *frame_copy);
	}
}


void SingleCameraCalibrationDialog::calibrationButtonClicked()
{
	if(!is_calibrating)
	{
		auto folder = QFileDialog::getExistingDirectory(this, "Please select a folder to store the runCalibration files!");
		if(folder.isEmpty())
			return;

		calib_files_folder_ = folder.toStdString();
		is_calibrating = true;

		// Change controls' status
		ui_.btnCalibration->setText("Stop runCalibration");
		ui_.gbOptions->setEnabled(false);
		ui_.gbCamOperations->setEnabled(false);
		ui_.btnGrabCalibImage->setEnabled(true);
		ui_.btnGrabCalibImage->setText("Grab runCalibration image (0)");

		// Enable real-time preview
		camera_->startCapture();
		startCalibBoardDetectThread();
	}
	else
	{
		calib_files_folder_ = "";
		is_calibrating = false;

		ui_.btnCalibration->setText("Start runCalibration");
		ui_.gbOptions->setEnabled(true);
		ui_.gbCamOperations->setEnabled(true);
		ui_.btnGrabCalibImage->setEnabled(false);
		ui_.btnGrabCalibImage->setText("Grab runCalibration image");

		// Disable real-time preview
		camera_->stopCapture();
		stopCalibBoardDetectThread();
	}

}

void SingleCameraCalibrationDialog::grabCalibImageButtonClicked()
{
	if(calib_files_folder_.empty())
		throw std::logic_error("Why the calib_files_folder_ is empty?");

	std::shared_ptr<cv::Mat> image = std::make_shared<cv::Mat>();
	{
		std::lock_guard lock(frame_buffer_mutex_);
		frame_buffer_->copyTo(*image);
	}
	
	calib_images_.push_back(image);
	ui_.btnGrabCalibImage->setText(
		QString::fromStdString(
			(boost::format("Grab runCalibration image (%1%/%2%)") % calib_images_.size() % MAX_CALIB_IMAGES_COUNT).str()
		)
	);
	
	if(calib_images_.size() == 8)
	{
		ui_.btnGrabCalibImage->setEnabled(false);
		cv::Mat cameraIntrinsicMatrix, distortionCoefficient, rotationMatrix, translationVector;

		float rms;
		bool success = runCalibration(
			calib_images_, cameraIntrinsicMatrix, distortionCoefficient, rotationMatrix, translationVector,
			PATTERN_PHY_WIDTH, cv::Size(KEY_POINTS_HORIZONTAL_COUNT, KEY_POINTS_VERTICAL_COUNT), 
			cv::Size(SUB_PIXEL_SEARCH_WINDOW, SUB_PIXEL_SEARCH_WINDOW),
			Chessboard, rms
		);

		logger_.info((boost::format("Calibration %1%.") % (success ? "Success" : "Failed")).str());
		logger_.info((boost::format("Calibration RMS: %1%.") % rms).str());

		ui_.btnGrabCalibImage->setEnabled(true);

		ui_.btnGrabCalibImage->setText(
			QString::fromStdString(
				(boost::format("Grab runCalibration image (0/%1%)") % MAX_CALIB_IMAGES_COUNT).str()
			)
		);
	}

	/*fs::path folder(calib_files_folder_);
	auto filename_str = (boost::format("calib_%1%.png") % (++calib_images_count)).str();
	auto save_path = folder / filename_str;
	auto save_path_str = save_path.string();

	auto saved = imwrite(save_path_str, frame_copy);
	if(!saved)
	{
		throw std::runtime_error("Failed to save image to " + save_path_str);
	}
	Logger::log("Save runCalibration image to " + save_path_str); */
}

void SingleCameraCalibrationDialog::parameterButtonClicked()
{
	auto button = dynamic_cast<QPushButton*>(sender());
	if(button != ui_.btnParameter)
		return;
	camera_->showParameterDialog();
}

/* -------------------- Slot methods --------------------*/
