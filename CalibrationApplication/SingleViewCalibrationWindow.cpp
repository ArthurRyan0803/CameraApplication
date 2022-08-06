#include "SingleViewCalibrationWindow.h"
#include "CalibrationParams.hpp"


namespace fs=boost::filesystem;
using namespace CameraLib;


SingleViewCalibrationWindow::SingleViewCalibrationWindow(
	std::shared_ptr<Camera> camera, QWidget* parent
):	QMainWindow(parent), q_image_buffer_(nullptr), cam_resolution_({}), calib_pattern_(Invalid), check_calib_board_(false),
    is_calibrating(false), logger_(GET_LOGGER()), camera_(std::move(camera))
{
	ui_.setupUi(this);

	connect(ui_.btnOneShot, &QPushButton::clicked, this, &SingleViewCalibrationWindow::oneShotButtonClicked);
	connect(ui_.btnCapture, &QPushButton::clicked, this, &SingleViewCalibrationWindow::captureButtonClicked);
	//connect(ui_.btnExit, &QPushButton::clicked, this, &SingleViewCalibrationWindow::close);
	connect(ui_.cmbCalibPattern, &QComboBox::currentTextChanged, this, &SingleViewCalibrationWindow::calibPatternChanged);
	connect(ui_.ckbDetectCalibBoard, &QCheckBox::stateChanged, this, &SingleViewCalibrationWindow::detectBoardCheckboxStateChanged);
	connect(ui_.btnSaveImage, &QPushButton::clicked, this, &SingleViewCalibrationWindow::saveImageButtonClicked);
	connect(ui_.btnCalibration, &QPushButton::clicked, this, &SingleViewCalibrationWindow::calibrationButtonClicked);
	connect(ui_.btnGrabCalibImage, &QPushButton::clicked, this, &SingleViewCalibrationWindow::grabCalibImageButtonClicked);
	connect(ui_.btnParameter, &QPushButton::clicked, this, &SingleViewCalibrationWindow::parameterButtonClicked);
	connect(ui_.btnCalibBoardSetting, &QPushButton::clicked, this, &SingleViewCalibrationWindow::calibBoardSettingsButtonClicked);

	camera_->setFrameReadyCallback([this](cv::InputArray data) { cameraFrameReadyCallback(data); });
	
	ui_.canvas->installEventFilter(this);

	for(auto& pair: UICommon::string_to_calib_pattern_map)
		ui_.cmbCalibPattern->addItem(QString::fromStdString(pair.first));

	camera_->open();
	cam_resolution_ = camera_->getCurrentResolution();
}


void SingleViewCalibrationWindow::cameraFrameReadyCallback(cv::InputArray image_data)
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
				UICommon::createQImage(*frame_buffer_, q_image_buffer_);

			UICommon::updateImageData(*frame_buffer_, *q_image_buffer_);
		}
	}

	QMetaObject::invokeMethod(this, [this](){ ui_.canvas->update(); }, Qt::QueuedConnection);
}


void SingleViewCalibrationWindow::findingCalibBoardPattern()
{
	try
	{
		logger_.debug("Start finding planarCalibration board pattern!");

		cv::Mat image_to_detect;
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
					cvtColor(*frame_buffer_, image_to_detect, cv::COLOR_RGB2GRAY);
				else if(frame_buffer_->elemSize() == 1)
					frame_buffer_->copyTo(image_to_detect);
				else
					throw std::runtime_error("Unrecognized element size!");
			}

			std::vector<cv::Point2f> points;

			cv::Size points_count(calib_board_settings_.horizontal_count, calib_board_settings_.vertical_count);
			bool found = findKeyPoints(image_to_detect, points, calib_pattern_, points_count,true);
			
			{
				std::lock_guard lock(pattern_points_mutex_);
				std::get<0>(pattern_points_) = found;
				std::get<1>(pattern_points_) = points;
			}

			std::string str = found ? "success" : "fails";
			//logger_.debug("Find chessboard " + str);
		}

		logger_.debug("Stop finding planarCalibration board pattern!");
	}
	catch(const std::exception& e)
	{
		logger_.error(e.what());
		throw e;
	}
}


void SingleViewCalibrationWindow::startCalibBoardDetectThread()
{
	if(board_detect_thread_)
		throw std::logic_error("Why the planarCalibration board detection thread is still alive?");

	check_calib_board_ = true;
	board_detect_thread_ = 
		std::make_unique<std::thread>(std::bind(&SingleViewCalibrationWindow::findingCalibBoardPattern, this));
}

void SingleViewCalibrationWindow::stopCalibBoardDetectThread()
{
	if(!board_detect_thread_)
		return;

	check_calib_board_ = false;
	board_detect_thread_->join();
	board_detect_thread_.reset();
}
/* -------------------- Events --------------------*/

void SingleViewCalibrationWindow::closeEvent(QCloseEvent* e)
{
	QMainWindow::closeEvent(e);
	camera_->close();
	if(board_detect_thread_)
	{
		check_calib_board_ = false;
		board_detect_thread_->join();
		board_detect_thread_.reset();
	}
}

bool SingleViewCalibrationWindow::eventFilter(QObject* obj, QEvent* e)
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
				UICommon::getImagePaintRegion(
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

		// Paint planarCalibration board points.
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
						UICommon::convertPaintPositions(cam_resolution_, paint_region, {point.x, point.y});

					painter.drawPoint(point_in_widget[0], point_in_widget[1]);
				}
			}
		}

		return true;
	}

	return QMainWindow::eventFilter(obj, e);
}

/* -------------------- Events --------------------*/

/* -------------------- Slot methods --------------------*/

void SingleViewCalibrationWindow::calibPatternChanged()
{
	auto box = dynamic_cast<QComboBox*>(sender());
	auto text  = box->currentText();
	calib_pattern_ = UICommon::string_to_calib_pattern_map.find(text.toStdString())->second;
}

void SingleViewCalibrationWindow::detectBoardCheckboxStateChanged()
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

void SingleViewCalibrationWindow::oneShotButtonClicked()
{
	{
		std::lock_guard lock(frame_buffer_mutex_);
		if(!frame_buffer_)
			frame_buffer_ = std::make_unique<cv::Mat>();
		
		camera_->oneShot(*frame_buffer_);

		{
			std::lock_guard q_image_lock(q_image_mutex_);
			if(!q_image_buffer_)
				UICommon::createQImage(*frame_buffer_, q_image_buffer_);
			UICommon::updateImageData(*frame_buffer_, *q_image_buffer_);
		}
	}

	ui_.canvas->update();
	ui_.btnSaveImage->setEnabled(true);
}

void SingleViewCalibrationWindow::captureButtonClicked()
{
	if(camera_->isCapturing())
	{
		camera_->stopCapture();
		ui_.btnCapture->setText("Start capturing");
		ui_.btnOneShot->setEnabled(true);
		ui_.ckbDetectCalibBoard->setChecked(false);	// This method will trigger slot [ detectBoardCheckboxStateChanged() ]
		ui_.ckbDetectCalibBoard->setEnabled(false);
	}
	else
	{
		camera_->startCapture();
		ui_.btnCapture->setText("Stop capturing");
		ui_.btnOneShot->setEnabled(false);
		ui_.ckbDetectCalibBoard->setEnabled(true);
		ui_.btnSaveImage->setEnabled(true);
		ui_.ckbDetectCalibBoard->setEnabled(true);
	}
}

void SingleViewCalibrationWindow::saveImageButtonClicked()
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


void SingleViewCalibrationWindow::calibrationButtonClicked()
{
	if(!is_calibrating)
	{
		auto folder = QFileDialog::getExistingDirectory(this, "Please select a folder to store the planarCalibration files!");
		if(folder.isEmpty())
			return;

		calib_files_folder_ = folder.toStdString();
		is_calibrating = true;

		// Change controls' status
		ui_.btnCalibration->setText("Stop planar calibration");
		ui_.gbOptions->setEnabled(false);
		ui_.gbCamOperations->setEnabled(false);
		ui_.btnGrabCalibImage->setEnabled(true);

		// Enable real-time preview
		camera_->startCapture();
		startCalibBoardDetectThread();
		
	}
	else
	{
		calib_files_folder_ = "";
		is_calibrating = false;

		ui_.btnCalibration->setText("Start planar calibration");
		ui_.gbOptions->setEnabled(true);
		ui_.gbCamOperations->setEnabled(true);
		ui_.btnGrabCalibImage->setEnabled(false);

		// Disable real-time preview
		camera_->stopCapture();
		stopCalibBoardDetectThread();

		calib_images_.clear();
	}
}

void SingleViewCalibrationWindow::grabCalibImageButtonClicked()
{
	//try
	//{
	//	if(calib_files_folder_.empty())
	//	throw std::logic_error("Why the calib_files_folder_ is empty?");

	//	// 1. take shot

	//	std::shared_ptr<cv::Mat> image = std::make_shared<cv::Mat>();
	//	{
	//		std::lock_guard lock(frame_buffer_mutex_);
	//		frame_buffer_->copyTo(*image);
	//	}
	//	
	//	calib_images_.push_back(image);
	//	ui_.btnGrabCalibImage->setText(
	//		QString::fromStdString(
	//			(boost::format("Grab Calibration Image %1%") % calib_images_.size()).str()
	//		)
	//	);

	//	// 2. calibration

	//	if(calib_images_.size() == calib_board_settings_.images_count)
	//	{
	//		ui_.btnGrabCalibImage->setEnabled(false);
	//		cv::Mat cameraIntrinsicMatrix, distortionCoefficient, rotationMatrix, translationVector;
	//		std::vector<std::vector<cv::Point2f>> key_points;
	//		std::vector<bool> key_points_found_flags;
	//		
	//		PlanarCalibrationParams params;
	//		std::string message = planarCalibration(
	//			calib_images_, calib_board_settings_, calib_pattern_, params, key_points, key_points_found_flags
	//		);
	//		bool success = message.empty();

	//		if(!success)
	//			logger_.info(message);
	//		logger_.info((boost::format("Calibration %1%.") % (success ? "Success" : "Failed")).str());
	//		logger_.info((boost::format("Calibration RMS: %1%.") % params.RMS).str());

	//		ui_.btnGrabCalibImage->setEnabled(true);
	//		ui_.btnGrabCalibImage->setText(
	//			QString::fromStdString(
	//				(boost::format("Grab planarCalibration image (0/%1%)") % calib_board_settings_.images_count).str()
	//			)
	//		);

	//		if(!success)
	//			return;

	//		// 3. save
	//		// 3.1 save calibration params
	//		fs::path folder(calib_files_folder_);
	//		fs::path file_path = folder / "params.json";
	//		params.save(file_path.string());

	//		// 3.2 save original images
	//		for(size_t i=0; i<calib_images_.size(); i++)
	//		{
	//			auto original_image = calib_images_[i];
	//			auto sub_folder = folder / "OriginalImages";
	//			Utils::createDirectory(sub_folder);
	//			file_path = sub_folder / (std::to_string(i) + ".png");
	//			if(!imwrite(file_path.string(), *original_image))
	//			{
	//				throw std::runtime_error("Failed to save original image!");
	//			}
	//		}

	//		// 3.3 save images with key points painted
	//		for(size_t i=0; i<calib_images_.size(); i++)
	//		{
	//			auto paint_image = calib_images_[i];
	//			cv::drawChessboardCorners(
	//				*paint_image, calib_board_settings_.count(),
	//				key_points[i], key_points_found_flags[i]
	//			);

	//			auto sub_folder = folder / "PaintImages";
	//			Utils::createDirectory(sub_folder);
	//			file_path = sub_folder / (std::to_string(i) + ".png");
	//			if(!imwrite(file_path.string(), *paint_image))
	//			{
	//				throw std::runtime_error("Failed to save paint image!");
	//			}
	//		}

	//		calibrationButtonClicked();
	//	}
	//}
	//catch (const std::exception& e)
	//{
	//	logger_.error(e.what());
	//	throw e;
	//}
}

void SingleViewCalibrationWindow::parameterButtonClicked()
{
	auto button = dynamic_cast<QPushButton*>(sender());
	if(button != ui_.btnParameter)
		return;
	camera_->showParameterDialog();
}

void SingleViewCalibrationWindow::calibBoardSettingsButtonClicked()
{
	auto button = dynamic_cast<QPushButton*>(sender());
	if(button != ui_.btnCalibBoardSetting)
		return;
	CalibBoardSettingsDialog dialog(calib_board_settings_, this);
	dialog.exec();
}

/* -------------------- Slot methods --------------------*/
