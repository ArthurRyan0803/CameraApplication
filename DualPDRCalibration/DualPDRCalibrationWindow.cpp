#include "DualPDRCalibrationWindow.h"
#include <QMessageBox>
#include <QDebug>
#include <QPainter>
#include <QFileDialog>
#include <QMetaMethod>
#include <vtkObject.h>
#include <boost/filesystem.hpp>

#include "Utils.h"
#include "MVCamerasFactory.hpp"
#include "CalibrationMethods.h"
#include "Utils.hpp"

using namespace CameraLib;


void visualizeCamera(
	pcl::visualization::PCLVisualizer& pcl_visualizer,
	const PlanarCalibrationParams& camera_params, const std::string& id,
	double r, double g, double b
)
{
	static int w=60, h=15, d=25;

	auto r_eigen_mat = Utils::matrixCast<double, float, 3>(camera_params.rmat.t());
	auto t_eigen_vec = Utils::vectorCast<double, float, 3>(- camera_params.rmat.t() * camera_params.tvec);

	pcl_visualizer.addCube(t_eigen_vec, Eigen::Quaternionf(r_eigen_mat), w, h, d, id + "_original");
	pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id + "_original");

	pcl_visualizer.getRenderWindow()->Render();
}


DualPDRCalibrationWindow::DualPDRCalibrationWindow(QWidget *parent)
    : QMainWindow(parent)
{
    ui_.setupUi(this);

	connect(ui_.cbLeftCamera, &QComboBox::currentTextChanged, this, &DualPDRCalibrationWindow::comboboxSelectedTextChanged);
	connect(ui_.btnParamter, &QPushButton::clicked, this, &DualPDRCalibrationWindow::openParameterDialog);
	connect(ui_.btnShot, &QPushButton::clicked, this, &DualPDRCalibrationWindow::oneshotButtonClicked);
	connect(ui_.btnSaveFolder, &QPushButton::clicked, this, &DualPDRCalibrationWindow::selectSaveFolder);
	connect(ui_.btnSave, &QPushButton::clicked, this, &DualPDRCalibrationWindow::saveImages);
	connect(ui_.btnCalibration, &QPushButton::clicked, this, &DualPDRCalibrationWindow::calibration);

	connect(ui_.cbCalibPattern, &QComboBox::currentTextChanged, this, &DualPDRCalibrationWindow::calibPatternChanged);
	connect(ui_.sbHorizontalCount, &QSpinBox::textChanged, this, &DualPDRCalibrationWindow::calibHorizontalCountChanged);
	connect(ui_.sbVerticalCount, &QSpinBox::textChanged, this, &DualPDRCalibrationWindow::calibVerticalCountChanged);
	connect(ui_.dsbInterval, &QDoubleSpinBox::textChanged, this, &DualPDRCalibrationWindow::calibIntervalChanged);

	ui_.canvasLeft->installEventFilter(this);
	ui_.canvasRight->installEventFilter(this);

	ui_.btnSave->setEnabled(false);
	ui_.btnCalibration->setEnabled(false);

	// calibration board settings
	calib_settings_.horizontal_count = ui_.sbHorizontalCount->value();
	calib_settings_.vertical_count = ui_.sbVerticalCount->value();
	calib_settings_.interval = ui_.dsbInterval->value();

	ui_.cbCalibPattern->addItems({ "Chessboard", "CirclesArray"});
	ui_.cbCalibPattern->setCurrentIndex(0);

	// VTK Widgets
	vtkObject::GlobalWarningDisplayOff();

	auto layout = ui_.gbStereo->layout();
	assert(layout, "layout is null!");

	vtk_widget_ = std::make_unique<QVTKOpenGLStereoWidget>();
	vtk_widget_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
	layout->addWidget(vtk_widget_.get());
	
	auto render_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	auto renderer = vtkSmartPointer<vtkRenderer>::New();
	render_window->AddRenderer(renderer);
	vtk_widget_->setRenderWindow(render_window);

	pcl_visualizer_ = std::make_unique<pcl::visualization::PCLVisualizer>(renderer, render_window, "stereo", false);
	pcl_visualizer_->initCameraParameters();
	pcl_visualizer_->setBackgroundColor(0.5, 0.5, 0.5);
	pcl_visualizer_->addCoordinateSystem(100, "reference");
	pcl_visualizer_->resetCamera();

	// Calibratio board detect thraed.	
	is_alive_ = true;
	std::thread thd_detect_left_points(std::bind(&DualPDRCalibrationWindow::findingCalibBoardPattern, this, 0));
	std::thread thd_detect_right_points(std::bind(&DualPDRCalibrationWindow::findingCalibBoardPattern, this, 1));

	thd_detect_left_points.detach();
	thd_detect_right_points.detach();
	
	// Create cameras
	MVCameraFactory<PDRImageCamera> factory;
	auto ids = factory.enumerateCamerasIDs();
	
	if (ids.size() < 2)
	{
		QMessageBox::warning(this, "Warning!", "Cameras are insufficient!");
		setEnabled(false);
		return;
	}
	else if(ids.size() > 2)
	{
		QMessageBox::warning(this, "Warning!", "Too Much cameras!");
		setEnabled(false);
		return;
	}

	auto first_camaera = factory.createMVCamera(ids[0]);
	auto second_camera = factory.createMVCamera(ids[1]);

	cam_ips_ = {
		std::string(first_camaera->getCameraInfo().acPortType), 
		std::string(second_camera->getCameraInfo().acPortType)
	};

	cameras_ = {
		{cam_ips_[0], first_camaera},
		{cam_ips_[1], second_camera}
	};

	for(auto& pair: cameras_)
		pair.second->open();

	QStringList ip_ui_list = {QString::fromStdString(cam_ips_[0]), QString::fromStdString(cam_ips_[1])};
	ui_.cbLeftCamera->addItems(ip_ui_list);
	ui_.cbLeftCamera->setCurrentIndex(0);

}

DualPDRCalibrationWindow::~DualPDRCalibrationWindow()
{
	is_alive_ = false;

	for(auto& pair: cameras_)
		pair.second->close();
}

void DualPDRCalibrationWindow::updateImageBuffers(const cv::Mat& image, int cam_index)
{
	// Copy frame data to cv::Mat for process
	{
		auto& frame_mutex = frame_buffer_mutexes_[cam_index];
		auto& frame_buffer = frame_buffers_[cam_index];
		std::lock_guard lock(frame_mutex);
		if (!frame_buffer)
			frame_buffer = std::make_unique<cv::Mat>();

		image.copyTo(*frame_buffer);
	}

	// Copy frame data to qimage for display
	{
		auto& q_image_mutex = q_images_mutexes_[cam_index];
		auto& q_image = q_images_[cam_index];

		std::lock_guard lock(q_image_mutex);
		if (!q_image)
			createQImage(image, q_image);
		updateImageData(image, *q_image);
	}
}

bool DualPDRCalibrationWindow::paintImage(int cam_index)
{
	auto canvas = cam_index == 0 ? ui_.canvasLeft : ui_.canvasRight;
	QPainter painter(canvas);

	std::array<int, 4> paint_region{};
	painter.setRenderHint(QPainter::Antialiasing, true);
	auto pen = QPen(QBrush(QColor::fromRgb(255, 0, 0)), 10.0f);
	painter.setPen(pen);

	auto& qimage_mutex = q_images_mutexes_[cam_index];
	auto& qimage = q_images_[cam_index];
	int width, height;

	{
		std::lock_guard lock(qimage_mutex);

		if (qimage)
		{
			width = qimage->width();
			height = qimage->height();
			getImagePaintRegion({ width, height }, { canvas->width(), canvas->height() }, paint_region);
			QRectF region_rect_f(paint_region[0], paint_region[1], paint_region[2], paint_region[3]);
			painter.drawImage(region_rect_f, *qimage);
		}
		else
			return false;
	}

	{
		auto& points_mutex = key_points_mutexes_[cam_index];
		points_mutex.lock();
		auto points = frames_key_points_[cam_index];
		points_mutex.unlock();

		for (auto& point : points)
		{
			auto point_in_widget =
				convertPaintPositions(
					{ width, height }, paint_region, { point.x, point.y }
				);

			painter.drawPoint(point_in_widget[0], point_in_widget[1]);
		}
	}

	return true;
}


void DualPDRCalibrationWindow::findingCalibBoardPattern(int cam_index)
{
	
	auto& frame_mutex = frame_buffer_mutexes_[cam_index];
	auto& frame_buffer = frame_buffers_[cam_index];
	cv::Mat image_to_detect;

	// If the frame_buffer_ is empty, wait the camera to grab image.
	while (true)
	{
		bool ready;
		{
			std::lock_guard lock(frame_mutex);
			ready = frame_buffers_[cam_index] != nullptr && !frame_buffer->empty();
		}
		if (ready)
			break;

		std::this_thread::sleep_for(std::chrono::microseconds(100));
	}

	
	auto& keypoints = frames_key_points_[cam_index];

	while (is_alive_)
	{
		try
		{
			// Waiting for calibration finished.
			while(is_calibrting_) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

			{
				std::lock_guard lock(frame_mutex);
				if (!frame_buffer)
					throw std::logic_error("Why frame_buffer_ is empty?");

				if (frame_buffer->channels() == 3)
					cv::cvtColor(*frame_buffer, image_to_detect, cv::COLOR_RGB2GRAY);
				else if (frame_buffer->channels() == 1)
					frame_buffer->copyTo(image_to_detect);
				else
					throw std::runtime_error("Unrecognized element size!");
			}

			std::vector<cv::Point2f> points_temp;
			
			bool found = findKeyPoints(image_to_detect, points_temp, calib_parttern_, calib_settings_.count(), true);

			{
				std::lock_guard lock(key_points_mutexes_[cam_index]);
				keypoints.clear();
				if (found)
					keypoints = points_temp;
			}
			
			QMetaObject::invokeMethod(this, [this, cam_index]()
			{
				(cam_index == 0 ? ui_.canvasLeft: ui_.canvasRight)->update();
			}, Qt::QueuedConnection);
		}
		catch (const std::exception& e)
		{
			qDebug() << e.what();
			throw e;
		}
	}
}

bool DualPDRCalibrationWindow::eventFilter(QObject* obj, QEvent* e)
{
	if (e->type() == QEvent::Paint)
	{
		if (obj == ui_.canvasLeft)
			return paintImage(0);
		if (obj == ui_.canvasRight)
			return paintImage(1);
	}

	return QMainWindow::eventFilter(obj, e);
}

void DualPDRCalibrationWindow::comboboxSelectedTextChanged()
{
	left_cam_ip_ = ui_.cbLeftCamera->currentText().toStdString();
	right_cam_ip_ = cam_ips_[1 - ui_.cbLeftCamera->currentIndex()];
	
	leftCamera()->setFrameReadyCallback(
		std::bind(&DualPDRCalibrationWindow::leftCameraFrameReadyCallback, this, std::placeholders::_1)
	);

	rightCamera()->setFrameReadyCallback(
		std::bind(&DualPDRCalibrationWindow::rightCameraFrameReadyCallback, this, std::placeholders::_1)
	);
}

void DualPDRCalibrationWindow::oneshotButtonClicked()
{
	std::vector<cv::Mat> left_images(3), right_images(3);
	leftCamera()->onceCapture(left_images);
	rightCamera()->onceCapture(right_images);

	if(left_images.empty() || right_images.empty())
		return;

	for(size_t i=0; i<2; i++)
	{
		auto& q_image_mutex = q_images_mutexes_[i];
		auto& q_image = q_images_[i];

		auto& frame_buffer_mutex_ = frame_buffer_mutexes_[i];
		auto& frame = frame_buffers_[i];

		cv::Mat image = i == 0 ? left_images[2]: right_images[2];

		auto canvas = i == 0 ? ui_.canvasLeft: ui_.canvasRight;

		{
			std::lock_guard lock_qimage(q_image_mutex);
			if (!q_image)
				createQImage(image, q_image);
			updateImageData(image, *q_image);
		}

		{
			std::lock_guard lock_buffer(frame_buffer_mutex_);
			if(!frame)
			{
				frame = std::make_unique<cv::Mat>();
				*frame = image.clone();
			}
			else if(frame->empty())
				*frame = image.clone();
			else
				image.copyTo(*frame);

			canvas->update();
		}
	}

	ui_.btnSave->setEnabled(true);
}

void DualPDRCalibrationWindow::selectSaveFolder()
{
	auto folder = QFileDialog::getExistingDirectory(this, "Select image saving folder!");
	if(!folder.isEmpty())
		save_folder_ = folder.toStdString();
	ui_.btnCalibration->setEnabled(true);
}

void DualPDRCalibrationWindow::saveImages()
{
	if(save_folder_.empty())
		return;

	static int save_index = 0;

	std::string left_path = save_folder_ + "/left" + std::to_string(save_index) + ".png";
	std::string right_path = save_folder_ + "/right" + std::to_string(save_index) + ".png";
	save_index++;

	q_images_[0]->save(QString::fromStdString(left_path));
	q_images_[1]->save(QString::fromStdString(right_path));
}

void DualPDRCalibrationWindow::calibration()
{
	namespace fs=boost::filesystem;

	is_calibrting_ = true;

	fs::path folder = save_folder_;
	assert(fs::exists(folder));

	std::vector<std::shared_ptr<cv::Mat>> left_images, right_images;

	for(size_t i=0;; i++)
	{
		auto left_path = folder / ("left" + std::to_string(i) + ".png");
		auto right_path = folder / ("right" + std::to_string(i) + ".png");

		if(!fs::exists(left_path) && !fs::exists(right_path))
			break;

		if(!fs::exists(left_path) || !fs::exists(right_path))
			continue;

		left_images.push_back(std::make_shared<cv::Mat>(cv::imread(left_path.string(), cv::IMREAD_GRAYSCALE)));
		right_images.push_back(std::make_shared<cv::Mat>(cv::imread(right_path.string(), cv::IMREAD_GRAYSCALE)));
	}

	DualViewCalibrationParams params;
	std::vector<std::vector<cv::Point2f>> left_points, right_points;
	std::vector<bool> left_flags, right_flags;
	auto message = stereoCalibration(
		left_images, right_images, 0, calib_settings_, calib_parttern_, 
		params, left_points, left_flags, right_points, right_flags
	);

	is_calibrting_ = false;

	if(!message.empty())
	{
		QMessageBox box;
		box.setWindowTitle("Warning");
		box.setText(QString::fromStdString(message));
		box.exec();
		return;
	}

	auto params_path = folder / "params.json";
	params.save(params_path.string());

	visualizeCamera(*pcl_visualizer_, params.left, "left", 1.0, 0, 0);
	visualizeCamera(*pcl_visualizer_, params.right, "right", 0, 1.0, 0);
	pcl_visualizer_->resetCamera();
	pcl_visualizer_->getRenderWindow()->Render();
	vtk_widget_->update();

	fs::path marker_dir = fs::path(save_folder_) / "points_marker";
	if(!fs::exists(marker_dir))
		fs::create_directory(marker_dir);

	for(size_t i = 0; i < left_images.size(); i ++)
	{
		auto left_image = left_images[i], right_image = right_images[i];
		cv::drawChessboardCorners(*left_image, cv::Size(calib_settings_.horizontal_count, calib_settings_.vertical_count) , left_points[i], left_flags[i]);

		cv::drawChessboardCorners(*right_image, cv::Size(calib_settings_.horizontal_count, calib_settings_.vertical_count) , right_points[i], right_flags[i]);

		cv::imwrite((marker_dir / ("left_" + std::to_string(i) + ".png")).string(), *left_image);
		cv::imwrite((marker_dir / ("right_" + std::to_string(i) + ".png")).string(), *right_image);
	}
}

void DualPDRCalibrationWindow::calibPatternChanged()
{
	QComboBox* box = static_cast<QComboBox*>(sender());
	auto text = box->currentText();
	if(text.startsWith("Circle"))
		calib_parttern_ = CirclesArray;
	else
		calib_parttern_ = Chessboard;
}

void DualPDRCalibrationWindow::calibHorizontalCountChanged()
{
	auto control = static_cast<QSpinBox*>(sender());
	calib_settings_.horizontal_count = control->value();
}

void DualPDRCalibrationWindow::calibVerticalCountChanged()
{
	auto control = static_cast<QSpinBox*>(sender());
	calib_settings_.vertical_count = control->value();
}

void DualPDRCalibrationWindow::calibIntervalChanged()
{
	auto control = static_cast<QDoubleSpinBox*>(sender());
	calib_settings_.interval = control->value();
}

std::shared_ptr<PDRImageCamera>& DualPDRCalibrationWindow::leftCamera()
{
	return cameras_[left_cam_ip_];
}

std::shared_ptr<PDRImageCamera>& DualPDRCalibrationWindow::rightCamera()
{
	return cameras_[right_cam_ip_];
}

void DualPDRCalibrationWindow::leftCameraFrameReadyCallback(cv::InputArray images)
{
	std::vector<cv::Mat> mats;
	images.getMatVector(mats);
	updateImageBuffers(mats[2], 0);
	QMetaObject::invokeMethod(this, [this]() { ui_.canvasLeft->update(); }, Qt::QueuedConnection);
}

void DualPDRCalibrationWindow::rightCameraFrameReadyCallback(cv::InputArray images)
{
	std::vector<cv::Mat> mats;
	images.getMatVector(mats);
	updateImageBuffers(mats[2], 1);
	QMetaObject::invokeMethod(this, [this]() { ui_.canvasRight->update(); }, Qt::QueuedConnection);
}

void DualPDRCalibrationWindow::openParameterDialog()
{
	if(!param_dialog_)
		param_dialog_ = std::make_unique<ParameterDialog>(leftCamera(), rightCamera(), this);

	param_dialog_->show();
}
