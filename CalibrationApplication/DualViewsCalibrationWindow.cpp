#include <opencv2/imgproc.hpp>
#include "DualViewsCalibrationWindow.h"



DualViewsCalibrationWindow::DualViewsCalibrationWindow(std::shared_ptr<Camera> camera, QWidget *parent)
	: QMainWindow(parent), logger_(GET_LOGGER()), camera_(camera), check_calib_board_(false)
{
	ui_.setupUi(this);

	connect(ui_.btnOneShot, &QPushButton::clicked, this, &DualViewsCalibrationWindow::oneShotButtonClicked);
	connect(ui_.btnCapture, &QPushButton::clicked, this, &DualViewsCalibrationWindow::captureButtonClicked);
	connect(ui_.btnExit, &QPushButton::clicked, this, &DualViewsCalibrationWindow::close);
	connect(ui_.cmbCalibPattern, &QComboBox::currentTextChanged, this, &DualViewsCalibrationWindow::calibPatternChanged);
	connect(ui_.ckbDetectCalibBoard, &QCheckBox::stateChanged, this, &DualViewsCalibrationWindow::detectBoardCheckboxStateChanged);
	connect(ui_.btnSaveImage, &QPushButton::clicked, this, &DualViewsCalibrationWindow::saveImageButtonClicked);
	connect(ui_.btnCalibration, &QPushButton::clicked, this, &DualViewsCalibrationWindow::calibrationButtonClicked);
	connect(ui_.btnGrabCalibImage, &QPushButton::clicked, this, &DualViewsCalibrationWindow::grabCalibImageButtonClicked);
	connect(ui_.btnParameter, &QPushButton::clicked, this, &DualViewsCalibrationWindow::parameterButtonClicked);
	connect(ui_.btnCalibBoardSetting, &QPushButton::clicked, this, &DualViewsCalibrationWindow::calibBoardSettingsButtonClicked);

	ui_.canvas_left->installEventFilter(this);
	ui_.canvas_right->installEventFilter(this);

	// Camera
	camera->setFrameReadyCallback([this](cv::InputArray data) { cameraFrameReadyCallback(data); });
	
	for(auto& pair: UICommon::string_to_clib_pattern_map)
		ui_.cmbCalibPattern->addItem(QString::fromStdString(pair.first));

	camera->open();
	cam_resolution_ = camera->getCurrentResolution();
}


void DualViewsCalibrationWindow::closeEvent(QCloseEvent* e)
{
	QMainWindow::closeEvent(e);
	camera_->close();
	
	check_calib_board_ = false;

	for(auto& thd_ptr: board_detect_threads_)
		if(thd_ptr)
		{
			thd_ptr->join();
			thd_ptr.reset();
		}
}


bool DualViewsCalibrationWindow::eventFilter(QObject* obj, QEvent* e)
{
	if(e->type() == QEvent::Paint)
		if(obj == ui_.canvas_left)
			return paintImage(0);
		else if(obj == ui_.canvas_right)
			return paintImage(1);
		else
			return false;
	else
		return false;

	return QMainWindow::eventFilter(obj, e);
}

void DualViewsCalibrationWindow::oneShotButtonClicked()
{
	cv::Mat frame;
	camera_->oneShot(frame);

	{
		std::lock_guard lock(frame_buffer_mutexes_[0]);
	}

	copyFrame(frame, 0);
	copyFrame(frame, 1);

	ui_.canvas_left->update();
	ui_.canvas_right->update();
	ui_.btnSaveImage->setEnabled(true);
}

void DualViewsCalibrationWindow::captureButtonClicked()
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

void DualViewsCalibrationWindow::detectBoardCheckboxStateChanged()
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

void DualViewsCalibrationWindow::calibPatternChanged()
{
	auto box = dynamic_cast<QComboBox*>(sender());
	auto text  = box->currentText();
	calib_pattern_ = UICommon::string_to_clib_pattern_map.find(text.toStdString())->second;
}

void DualViewsCalibrationWindow::saveImageButtonClicked()
{
}

void DualViewsCalibrationWindow::calibrationButtonClicked()
{
}

void DualViewsCalibrationWindow::grabCalibImageButtonClicked()
{
}

void DualViewsCalibrationWindow::parameterButtonClicked()
{

}

void DualViewsCalibrationWindow::calibBoardSettingsButtonClicked()
{
	auto button = dynamic_cast<QPushButton*>(sender());
	if(button != ui_.btnCalibBoardSetting)
		return;
	CalibBoardSettingsDialog dialog(calib_board_settings_, this);
	dialog.exec();
}

void DualViewsCalibrationWindow::copyFrame(const cv::Mat& frame, int buffer_index)
{
	assert(buffer_index >= 0 && buffer_index <= 1);

	size_t cols_half = frame.cols / 2;
	size_t col_start = buffer_index * cols_half;
	size_t col_end = col_start + cols_half;

	auto& frame_mutex = frame_buffer_mutexes_[buffer_index];
	auto& frame_buffer = frame_buffers_[buffer_index];

	auto& q_image_mutex = q_image_mutexes_[buffer_index];
	auto& q_image = q_images_[buffer_index];

	// Copy frame data to cv::Mat for process
	{
		std::lock_guard lock(frame_mutex);
		if(!frame_buffer)
			frame_buffer = std::make_unique<cv::Mat>();
	
		frame.rowRange(0, frame.rows).colRange(col_start, col_end).copyTo(*frame_buffer);
	}

	// Copy frame data to qimage for display
	{
		std::lock_guard lock(q_image_mutex);
		if(!q_image)
			Utils::createQImage(*frame_buffer, q_image);
		Utils::updateImageData(*frame_buffer, *q_image);
	}
}

bool DualViewsCalibrationWindow::paintImage(int index)
{
	auto canvas = index == 0 ? ui_.canvas_left: ui_.canvas_right;
	QPainter painter(canvas);

	std::array<int, 4> paint_region{};
	painter.setRenderHint(QPainter::Antialiasing, true);
	auto pen = QPen(QBrush(QColor::fromRgb(255, 0, 0)), 10.0f);
	painter.setPen(pen);

	{
		auto& qimag_mutex = q_image_mutexes_[index];
		auto& qimage = q_images_[index];
		// Paint image.
		std::lock_guard lock(qimag_mutex);
			
		if(qimage)
		{
			Utils::getImagePaintRegion(
				{ qimage->width(), qimage->height() }, 
				{ canvas->width(), canvas->height() }, 
				paint_region
			);
			QRectF region_rect_f(paint_region[0], paint_region[1], paint_region[2], paint_region[3]);
			painter.drawImage(region_rect_f, *qimage);
		}
		else
			return false;
	}

	{
		if(check_calib_board_)
		{
			auto& points_mutex = key_points_mutexes_[index];
			points_mutex.lock();
			auto points = frames_key_points_[index];
			points_mutex.unlock();

			for(auto& point: points)
			{
				auto point_in_widget =  
					Utils::convertPaintPositions(
						{cam_resolution_[0] / 2, cam_resolution_[1]}, 
						paint_region, {point.x, point.y}
				);

				painter.drawPoint(point_in_widget[0], point_in_widget[1]);
			}
		}

		return true;
	}

	return false;
}

void DualViewsCalibrationWindow::cameraFrameReadyCallback(cv::InputArray image_data)
{
	if(image_data.empty())
		return;

	copyFrame(image_data.getMat(), 0);
	copyFrame(image_data.getMat(), 1);

	QMetaObject::invokeMethod(this, [this](){ ui_.canvas_left->update(); }, Qt::QueuedConnection);
	QMetaObject::invokeMethod(this, [this](){ ui_.canvas_right->update(); }, Qt::QueuedConnection);
}

void DualViewsCalibrationWindow::startCalibBoardDetectThread()
{
	if(board_detect_threads_[0] || board_detect_threads_[1])
		throw std::logic_error("Why the calibration board detection thread is still alive?");

	check_calib_board_ = true;
	board_detect_threads_[0] = 
		std::make_unique<std::thread>(std::bind(&DualViewsCalibrationWindow::findingCalibBoardPattern, this, 0));

	board_detect_threads_[1] =
		std::make_unique<std::thread>(std::bind(&DualViewsCalibrationWindow::findingCalibBoardPattern, this, 1));
}

void DualViewsCalibrationWindow::stopCalibBoardDetectThread()
{
	if(!(board_detect_threads_[0] && board_detect_threads_[1]))
		return;

	check_calib_board_ = false;
	for(auto& thread_ptr: board_detect_threads_)
	{
		thread_ptr->join();
		thread_ptr.reset();
	}
}

void DualViewsCalibrationWindow::findingCalibBoardPattern(int index)
{
	auto& keypoints = frames_key_points_[index];
	auto& frame_mutex = frame_buffer_mutexes_[index];
	auto& frame_buffer = frame_buffers_[index];
	auto& keypoints_mutex = key_points_mutexes_[index];

	try
	{
		logger_.debug("Start finding calibration board pattern of view " + index);

		cv::Mat image_to_detect;
		cv::Size2i chessboard_size(5, 7);
		
		// If the frame_buffer_ is empty, wait the camera to grab image.
		while(true)
		{
			bool ready;
			{
				std::lock_guard lock(frame_mutex);
				ready = frame_buffer != nullptr;
			}
			if(ready)
				break;

			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}

		while(check_calib_board_)
		{
			{
				std::lock_guard lock(frame_mutex);
				if(!frame_buffer)
					throw std::logic_error("Why frame_buffer_ is empty?");

				if(frame_buffer->elemSize() == 3)
					cv::cvtColor(*frame_buffer, image_to_detect, cv::COLOR_RGB2GRAY);
				else if(frame_buffer->elemSize() == 1)
					frame_buffer->copyTo(image_to_detect);
				else
					throw std::runtime_error("Unrecognized element size!");
			}

			std::vector<cv::Point2f> points_temp;

			cv::Size points_count(calib_board_settings_.horizontal_count, calib_board_settings_.vertical_count);
			bool found = findKeyPoints(image_to_detect, points_temp, calib_pattern_, points_count,true);
			
			{
				std::lock_guard lock(keypoints_mutex);
				keypoints.clear();
				if(found)
					keypoints = points_temp;
			}
		}
		
		logger_.debug("Stop finding calibration board pattern of view " + index);
	}
	catch(const std::exception& e)
	{
		logger_.error(e.what());
		throw e;
	}

}
