#include <opencv2/imgproc.hpp>				 // Do not swap #include
#include <opencv2/imgcodecs.hpp>

#include <QPainter>
#include <QFileDialog>
#include <QMessageBox>

#include <vtk-9.0/vtkSmartPointer.h>
#include <vtk-9.0/vtkGenericOpenGLRenderWindow.h>
#include <vtk-9.0/vtkRenderer.h>

#include "DualViewsCalibrationWindow.h"
#include "FunctionalDialog.h"
#include "CalibrationMethods.h"
#include "CalibrationSettingDialog.h"
#include "CalibrationParams.hpp"
#include "UICommon.h"
#include "Utils.hpp"
#include "Logger.hpp"


using namespace CameraLib;
static Logger& logger_ = GET_LOGGER();


DualViewsCalibrationWindow::DualViewsCalibrationWindow(
	std::shared_ptr<Camera> camera, 
	QWidget *parent
)
	: QMainWindow(parent),  camera_(camera), check_calib_board_(false), is_calibrating_(false)
{
	ui_.setupUi(this);

	stereo_widget_ = std::make_unique<QVTKOpenGLStereoWidget>();
	stereo_widget_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
	ui_.widget_vtk->layout()->addWidget(stereo_widget_.get());

	auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(renderer);
	pcl_visualizer_ = std::make_unique<pcl::visualization::PCLVisualizer>(renderer, renderWindow, "viewer", false);
	stereo_widget_->setRenderWindow(renderWindow);
	
	connect(ui_.btnOneShot, &QPushButton::clicked, this, &DualViewsCalibrationWindow::oneShotButtonClicked);
	connect(ui_.btnCapture, &QPushButton::clicked, this, &DualViewsCalibrationWindow::captureButtonClicked);
	connect(ui_.cmbCalibPattern, &QComboBox::currentTextChanged, this, &DualViewsCalibrationWindow::calibPatternChanged);
	connect(ui_.ckbDetectCalibBoard, &QCheckBox::stateChanged, this, &DualViewsCalibrationWindow::detectBoardCheckboxStateChanged);
	connect(ui_.btnSaveImage, &QPushButton::clicked, this, &DualViewsCalibrationWindow::saveImageButtonClicked);
	connect(ui_.btnCalibration, &QPushButton::clicked, this, &DualViewsCalibrationWindow::calibrationButtonClicked);
	connect(ui_.btnGrabCalibImage, &QPushButton::clicked, this, &DualViewsCalibrationWindow::grabCalibImageButtonClicked);
	connect(ui_.btnParameter, &QPushButton::clicked, this, &DualViewsCalibrationWindow::parameterButtonClicked);
	connect(ui_.btnCalibBoardSetting, &QPushButton::clicked, this, &DualViewsCalibrationWindow::calibBoardSettingsButtonClicked);

	ui_.canvas_left->installEventFilter(this);
	ui_.canvas_right->installEventFilter(this);
	
	camera->setFrameReadyCallback([this](cv::InputArray data) { cameraFrameReadyCallback(data); });
	
	for(auto& pair: UICommon::string_to_calib_pattern_map)
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
		if(obj == ui_.canvas_right)
			return paintImage(1);
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
	calib_pattern_ = UICommon::string_to_calib_pattern_map.find(text.toStdString())->second;
}


void DualViewsCalibrationWindow::saveImageButtonClicked()
{
	auto filename = QFileDialog::getSaveFileName(this, "Save image", "", "*.png");
	cv::Mat left_image, right_image;
	{
		std::lock_guard lock(frame_buffer_mutexes_[0]);
		frame_buffers_[0]->copyTo(left_image);
	}
	{
		std::lock_guard lock(frame_buffer_mutexes_[1]);
		frame_buffers_[1]->copyTo(right_image);
	}

	assert(left_image.type() == right_image.type());
	cv::Mat entire_image(left_image.rows, left_image.cols + right_image.cols, CV_MAT_TYPE(left_image.type()));
	cv::Mat left_part(entire_image, cv::Rect(0, 0, left_image.cols, left_image.rows));
	cv::Mat right_part(entire_image, cv::Rect(left_image.cols, 0, right_image.cols, right_image.rows));
	left_image.copyTo(left_part);
	right_image.copyTo(right_part);
	cv::imwrite(filename.toStdString(), entire_image);
}


void DualViewsCalibrationWindow::calibrationButtonClicked()
{
	if(!is_calibrating_)
	{
		auto folder = QFileDialog::getExistingDirectory(this, "Please select a folder to store the planarCalibration files!");
		if(folder.isEmpty())
			return;

		calib_files_folder_ = folder.toStdString();
		is_calibrating_ = true;

		// Change controls' status
		ui_.btnCalibration->setText("Stop stereo calibration");
		ui_.btnGrabCalibImage->setText("Grab Calibration Image");
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
		is_calibrating_ = false;

		ui_.btnCalibration->setText("Start stereo calibration");
		ui_.gbOptions->setEnabled(true);
		ui_.gbCamOperations->setEnabled(true);
		ui_.btnGrabCalibImage->setEnabled(false);

		// Disable real-time preview
		camera_->stopCapture();
		stopCalibBoardDetectThread();
		left_calib_images_.clear();
		right_calib_images_.clear();
	}
}


void DualViewsCalibrationWindow::grabCalibImageButtonClicked()
{
	namespace fs=boost::filesystem;

	try
	{
		if(calib_files_folder_.empty())
			throw std::logic_error("Why the calib_files_folder_ is empty?");

		// 1. take shot
		std::shared_ptr<cv::Mat> left_image = std::make_shared<cv::Mat>(), right_image = std::make_shared<cv::Mat>();
		{
			std::lock_guard lock(frame_buffer_mutexes_[0]);
			frame_buffers_[0]->copyTo(*left_image);
		}
		{
			std::lock_guard lock(frame_buffer_mutexes_[1]);
			frame_buffers_[1]->copyTo(*right_image);
		}
		
		left_calib_images_.push_back(left_image);
		right_calib_images_.push_back(right_image);

		ui_.btnGrabCalibImage->setText(
			QString::fromStdString(
				(boost::format("Grab Calibration Image (%1%/%2%)") % left_calib_images_.size() % calib_board_settings_.images_count).str()
			)
		);
		
		if(left_calib_images_.size() != calib_board_settings_.images_count)
			return;

		bool success;
		// 2. calibration (if got enough images)
		
		std::vector<std::vector<cv::Point2f>> left_key_points, right_key_points;
		std::vector<bool> left_flags, right_flags;
		DualViewCalibrationParams params;

		FunctionalDialog dialog(nullptr,
		[this, &success, &left_key_points, &right_key_points, &left_flags, &right_flags, &params]
			{
				success = stereoCalibration(
					left_calib_images_, right_calib_images_, 
					0, calib_board_settings_, calib_pattern_,
					params,
					left_key_points, left_flags, 
					right_key_points, right_flags
				);

				logger_.info((boost::format("Calibration %1%.") % (success ? "Success" : "Failed")).str());
				logger_.info((boost::format("Left calibration RMS: %1%.") % params.left.RMS).str());
				logger_.info((boost::format("Right calibration RMS: %1%.") % params.right.RMS).str());
				logger_.info((boost::format("Stereo calibration RMS: %1%.") % params.stereo.RMS).str());

				if(success)
				{
					// 3. save
					// 3.1 save calibration params
					fs::path folder(calib_files_folder_);
					fs::path file_path = folder / "params.json";
					
					params.save(file_path.string());

					// 3.2 save images
					for(size_t i=0; i < left_calib_images_.size(); i++)
					{
						// original image
						cv::Mat left_image = *left_calib_images_[i], right_image = *right_calib_images_[i];
						auto sub_folder = folder / "OriginalImages";
						Utils::createDirectory(sub_folder);
						file_path = sub_folder / (std::to_string(i) + ".png");
						cv::Mat whole_image;
						Utils::stitch_image(left_image, right_image, whole_image);
						if(!cv::imwrite(file_path.string(), whole_image))
						throw std::runtime_error("Failed to save original image!");

						// image with key points markers
						cv::Mat left_paint_image, right_paint_image;
						cv::cvtColor(left_image, left_paint_image, cv::COLOR_GRAY2RGB);
						cv::cvtColor(right_image, right_paint_image, cv::COLOR_GRAY2RGB);

						cv::drawChessboardCorners(left_paint_image, calib_board_settings_.count(), left_key_points[i], left_flags[i]);
						cv::drawChessboardCorners(right_paint_image, calib_board_settings_.count(), right_key_points[i], right_flags[i]);

						Utils::stitch_image(left_paint_image, right_paint_image, whole_image);
						sub_folder = folder / "PaintImages";
						Utils::createDirectory(sub_folder);
						file_path = sub_folder / (std::to_string(i) + ".png");
						if(!cv::imwrite(file_path.string(), whole_image))
						throw std::runtime_error("Failed to save paint image!");

						// rectified image
						cv::Mat left_remap, right_remap;
						cv::remap(left_paint_image, left_remap, params.stereo.left_map1, params.stereo.left_map2, cv::INTER_LANCZOS4);
						cv::remap(right_paint_image, right_remap, params.stereo.right_map1, params.stereo.right_map2, cv::INTER_LANCZOS4);

						Utils::stitch_image(left_remap, right_remap, whole_image);
						sub_folder = folder / "RectifiedImages";
						Utils::createDirectory(sub_folder);
						file_path = sub_folder / (std::to_string(i) + ".png");
						if(!cv::imwrite(file_path.string(), whole_image))
						throw std::runtime_error("Failed to save paint image!");
					}
				}
			},
			"Calibrating..."
		);
		dialog.exec();
		
		ui_.btnGrabCalibImage->setText("Grab Calibration Image");
		
		if(success)
		{
			// 4. Visualize camera pos
			visualizeCalibPlanar(calib_board_settings_, 0.7, 0.7, 0.7, "calib_planar");
			visualizeCamera(params.left, params.stereo.R1, "left_camera", 0, 1, 0);
			visualizeCamera(params.right, params.stereo.R2, "right_camera", 0, 0, 1);
			pcl_visualizer_->addCoordinateSystem(100);
			pcl_visualizer_->resetCamera();
			stereo_widget_->update();
			stereo_widget_->renderWindow()->Render();
		}

		calibrationButtonClicked();

	}
	catch (const std::exception& e)
	{
		logger_.error(e.what());
		throw e;
	}
}


void DualViewsCalibrationWindow::parameterButtonClicked()
{
	camera_->showParameterDialog();
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
			UICommon::createQImage(*frame_buffer, q_image);
		UICommon::updateImageData(*frame_buffer, *q_image);
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
		auto& qimage_mutex = q_image_mutexes_[index];
		auto& qimage = q_images_[index];
		// Paint image.
		std::lock_guard lock(qimage_mutex);
			
		if(qimage)
		{
			UICommon::getImagePaintRegion(
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
					UICommon::convertPaintPositions(
						{cam_resolution_[0] / 2, cam_resolution_[1]}, 
						paint_region, {point.x, point.y}
				);

				painter.drawPoint(point_in_widget[0], point_in_widget[1]);
			}
		}
	}
	return true;
}


void DualViewsCalibrationWindow::visualizeCalibPlanar(
	const CalibrationBoardSettings& settings, double r, double g, double b, const std::string& id
)
{
	auto w = settings.horizontal_count * settings.interval,
			h = settings.vertical_count * settings.interval,
			z = settings.interval;

	pcl_visualizer_->addCube(
		Eigen::Vector3f(w / 2,  h / 2, -z/2), Eigen::Quaternionf::Identity(), 
		w, h, z, id
	);
	pcl_visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id);
	pcl_visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id);
}


void DualViewsCalibrationWindow::visualizeCamera(
	const PlanarCalibrationParams& camera_params, const cv::Mat& R, const std::string& id,
	double r, double g, double b
)
{
	static int w=20, h=20, d=60;

	cv::Mat r_mat;
	cv::transpose(camera_params.rmat, r_mat);
	cv::Mat t_mat = -r_mat*camera_params.tvec;

	cv::Mat rectified_r_mat = R * r_mat;

	auto t_eigen_vec = Utils::VectorCast<double, float, 3>(t_mat);
	auto r_eigen_mat = Utils::MatrixCast<double, float, 3>(r_mat);
	auto rectified_r_eigen_mat = Utils::MatrixCast<double, float, 3>(rectified_r_mat);

	pcl_visualizer_->addCube(t_eigen_vec, Eigen::Quaternionf(r_eigen_mat), w, h, d, id + "_original");
	pcl_visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id + "_original");
	
	pcl_visualizer_->addCube(t_eigen_vec, Eigen::Quaternionf(rectified_r_eigen_mat), w, h, d, id + "_rectified");
	pcl_visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id + "_rectified");
	pcl_visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id + "_rectified");
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
