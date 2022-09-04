#include "HumanFaceReconstruction.h"

#include <QDebug>
#include <QPainter>
#include <QMessageBox>
#include <QFileDialog>
#include <QMetaMethod>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/surface/mls.h>

#include "VSensorPointCloudCamerasFactory.hpp"
#include "Logger.hpp"
#include "UiUtils.h"
#include "Path.hpp"
#include "CloudProcessParamsDialog.h"
#include "PclUtils.hpp"
#include "ImageUtils.hpp"

namespace fs=boost::filesystem;
using namespace CameraLib;

auto& logger = GET_LOGGER();

#define LEFT_CLOUD_ID "left"
#define RIGHT_CLOUD_ID "right"
#define STITCHED_CLOUD_ID "stitched"
#define SEARCH_RADIUS 8

void HumanFaceReconstruction::openCameraAndStartCaptureImages(
	std::shared_ptr<CameraLib::VSensorPointCloudCamera> camera, const CameraParams& params, bool initialize
)
{
	camera->open(initialize);
	camera->setExposure(params.gray_exposure, CameraLib::VSensorPointCloudCamera::Gray);
	camera->setAnalogGain(params.gray_gain, CameraLib::VSensorPointCloudCamera::Gray);
	camera->setExposure(params.color_exposure, CameraLib::VSensorPointCloudCamera::Color);
	camera->setAnalogGain(params.color_gain, CameraLib::VSensorPointCloudCamera::Color);
	camera->setCaptureMode(VSensorPointCloudCamera::ImageCapture);

	camera->onceCapture(std::vector<cv::Mat>{cv::Mat(), cv::Mat(), cv::Mat()});

	// Set up background threads to capture color image in real-time.
	setupCaptureThread(camera);
}

HumanFaceReconstruction::HumanFaceReconstruction(QWidget *parent)
    : QMainWindow(parent), left_color_image_(), right_color_image_()
{
    ui.setupUi(this);

	addVtkWidget(*ui.gbStitchCloud, vtk_widget_, renderer_, render_window_, visualizer_, "stitched_cloud_visualizer");

	// Connect slots
	connect(ui.btnConstruct, &QPushButton::clicked, this, &HumanFaceReconstruction::constructHumanFace);
	connect(ui.btnClearLog, &QPushButton::clicked, ui.tbLog, &QTextEdit::clear);

	connect(ui.actSaveStitchedCloud, &QAction::triggered, this, &HumanFaceReconstruction::saveStitchedPointCloud);
	connect(ui.actSaveStitchedNormalCloud, &QAction::triggered, this, &HumanFaceReconstruction::saveStitchedPointNormalCloud);
	connect(ui.actSaveLeftCloud, &QAction::triggered, this, &HumanFaceReconstruction::saveLeftPointCloud);
	connect(ui.actSaveRightCloud, &QAction::triggered, this, &HumanFaceReconstruction::saveRightPointCloud);
	connect(ui.actSaveCoarseIntersectionCloud, &QAction::triggered, this, &HumanFaceReconstruction::saveInetersectionPointCloudBeforeICP);
	connect(ui.actSavePreciseIntersectionCloud, &QAction::triggered, this, &HumanFaceReconstruction::saveInetersectionPointCloudAfterICP);

	connect(ui.actOpenCloudProcessParamsDialog, &QAction::triggered, this, &HumanFaceReconstruction::openCloudProcessParamsDialog);
	connect(ui.actCameraSetting, &QAction::triggered, this, &HumanFaceReconstruction::openCamerasSettingWindow);

	// Widget canvas event filter
	ui.canvasLeft->installEventFilter(this);
	ui.canvasRight->installEventFilter(this);

	// Load point cloud processing params file.
	auto point_cloud_params_path = Path::getPointCloudProcessParamsPath();
	if(fs::exists(point_cloud_params_path))
		cloud_process_params_.load(point_cloud_params_path);
	else
		cloud_process_params_.save(point_cloud_params_path);

	// Load cameras params file.
	cameras_params_ = std::make_shared<DualCamerasParams>();
	auto cameras_params_path = Path::getCameraParamsPath();
	if(fs::exists(cameras_params_path))
		cameras_params_->load(cameras_params_path);
	else
		cameras_params_->save(cameras_params_path);

	auto visualization_params_path = Path::getVisualizationParamsPath();
	if(fs::exists(visualization_params_path))
		visualization_params_.load(visualization_params_path);
	else
		visualization_params_.save(visualization_params_path);

	ui.lbHint->setVisible(false);

	// Allocate memory
	pdr_results_ = { std::make_unique<VSensorResult>(), std::make_unique<VSensorResult>() };

	left_cloud_ = std::make_shared<PointCloud>();
	right_cloud_ = std::make_shared<PointCloud>();
	stitched_cloud_ = std::make_shared<PointCloud>();
	stitched_normal_cloud_ = std::make_shared<PointNormalCloud>();
	intersection_cloud_before_icp_ = std::make_shared<PointCloud>();
	intersection_cloud_after_icp_ = std::make_shared<PointCloud>();
	
	// Disable the window until other parameters and hardware devices successfully initialize.
	setEnabledThreadSafe(false);

	// Connect cameras
	auto factory = std::make_shared<CameraLib::VSensorPointCloudCamerasFactory>();
	auto ids = factory->enumerateCamerasIDs();
	if(ids.size() >= 3)
	{
		std::string message = "Find more than 2 PDR cameras!";
		QMessageBox::warning(nullptr, "Error", message.c_str());
		addLogThreadSafe(message);
		setEnabledThreadSafe(false);
		return;
	}
	else if(ids.size() < 2)
	{
		std::string message = "Find less than 2 PDR cameras!";
		QMessageBox::warning(nullptr, "Error", message.c_str());
		addLogThreadSafe(message);
		setEnabledThreadSafe(false);
		return;
	}

	auto camera_connection_thread = connectCamerasAsync(factory, ids);
	
	// Get stitching transformation params.
	auto transform_params_path = Path::getStitchingTransformParamsFilePath();

	// If the stitching params file exists, load it and wait for cameras connections establishment.
	if(fs::exists(transform_params_path))
	{
		stitch_params_.load(transform_params_path);
		stitch_transform_ = stitch_params_.eigenTransformation();
		std::thread thread(
			[camera_connection_thread, this] 
			{ 
				camera_connection_thread->join(); 
				setEnabledThreadSafe(true);
			}
		);
		thread.detach();
		return;
	}

	// If the stitching transformation file is not found, manually select one and load it.
	auto btn = QMessageBox::information(nullptr, "Warning", "Please select a camera calibration file!", QMessageBox::Ok);
	if(btn != QMessageBox::Ok)
	{
		addLogThreadSafe("Cannot find calibration file!");
		return;
	}

	auto calib_filepath = QFileDialog::getOpenFileName(nullptr, "Select a file to open...", QDir::homePath(), "*.json").toStdString();
	if(calib_filepath.empty())
	{
		addLogThreadSafe("Cannot find calibration file!");
		return;
	}

 	auto param_load_thread = loadCalibrationParamsAsync(calib_filepath);

	// Waiting for cammeras connection and calibration params loading.
	std::thread thd(
		[this, camera_connection_thread, param_load_thread]
		{
			camera_connection_thread->join();
			param_load_thread->join();
			if(left_camera_->isOpened() && right_camera_->isOpened())
				setEnabledThreadSafe(true);
		}
	);
	thd.detach();
}

HumanFaceReconstruction::~HumanFaceReconstruction()
{
	{ std::lock_guard lock(left_camera_mutex_); if(left_camera_) left_camera_->close(); }
	{ std::lock_guard lock(right_camera_mutex_); if(right_camera_) right_camera_->close(); }

	cloud_process_params_.save(Path::getPointCloudProcessParamsPath());
	cameras_params_->save(Path::getCameraParamsPath());
}

void HumanFaceReconstruction::addVtkWidget(
	const QGroupBox& group_box, std::unique_ptr<QVTKOpenGLStereoWidget>& vtk_widget,
	vtkSmartPointer<vtkRenderer>& renderer, vtkSmartPointer<vtkGenericOpenGLRenderWindow>& render_window,
	std::unique_ptr<pcl::visualization::PCLVisualizer>& visualizer, const std::string& id
) const
{
	auto layout = group_box.layout();
	assert(layout && "layout is null!");

	vtk_widget = std::make_unique<QVTKOpenGLStereoWidget>();
	vtk_widget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
	layout->addWidget(vtk_widget.get());
	
	render_window = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
	renderer = vtkSmartPointer<vtkRenderer>::New();
	render_window->AddRenderer(renderer);
	vtk_widget->setRenderWindow(render_window);
	visualizer = std::make_unique<pcl::visualization::PCLVisualizer>(renderer, render_window, id, false);
	visualizer->initCameraParameters();
	visualizer->setBackgroundColor(0.5, 0.5, 0.5);
	visualizer->resetCamera();
}

void HumanFaceReconstruction::constructHumanFace()
{
	setEnabledThreadSafe(false);
	std::thread(
		[this]
		{
			addLogThreadSafe("---- Fetching point clouds ----");
			setHintTextVisible(true);
			bool success = fetchPointCloudsFromCamera();
			setHintTextVisible(false);

			if(!success)
			{
				addLogThreadSafe("Failed to construct point clouds!");
				return;
			}
			
			addLogThreadSafe("---- Registering ----");
			auto transform = reigsterClouds(left_cloud_, right_cloud_);
			
			stitchClouds(left_cloud_, right_cloud_, transform, stitched_cloud_);

			// Process stitched cloud
			addLogThreadSafe("---- Processing ----");
			
			if(cloud_process_params_.process_stitched_cloud)
			{
				preProcessPointCloud(stitched_cloud_);

				auto tree = std::make_shared<pcl::search::KdTree<MyPoint>>();
				pcl::MovingLeastSquares<MyPoint, MyPointNormal> mls;

				mls.setInputCloud(stitched_cloud_);
				mls.setSearchMethod(tree);
				mls.setSearchRadius(SEARCH_RADIUS);
				mls.setComputeNormals(true);
				mls.setPolynomialOrder(2);
				mls.setSqrGaussParam(0.0001);
				mls.setNumberOfThreads(std::thread::hardware_concurrency());

				mls.process(*stitched_normal_cloud_);
				
				stitched_cloud_->clear();

				for(auto& pt_normal: stitched_normal_cloud_->points)
					stitched_cloud_->push_back(
						MyPoint(pt_normal.x, pt_normal.y, pt_normal.z, pt_normal.r, pt_normal.g, pt_normal.b)
					);

			}

			auto center = PclPointCloudUtils<MyPoint>::cloudsCentroid({ stitched_cloud_ });
			PclPointCloudUtils<MyPoint>::translatePointCloud(stitched_cloud_, center);

			visualizeStitchedPointCloudThreadSafe(stitched_cloud_);
			
			addLogThreadSafe("---- Finished ----");

			setEnabledThreadSafe(true);
		}
	).detach();
}

void HumanFaceReconstruction::saveStitchedPointCloud()
{
	openFileDialogToSaveCloud(stitched_cloud_);
}

void HumanFaceReconstruction::saveStitchedPointNormalCloud()
{
	openFileDialogToSaveCloud(stitched_normal_cloud_);
}

void HumanFaceReconstruction::saveLeftPointCloud()
{
	openFileDialogToSaveCloud(left_cloud_);
}

void HumanFaceReconstruction::saveRightPointCloud()
{
	openFileDialogToSaveCloud(right_cloud_);
}

void HumanFaceReconstruction::saveInetersectionPointCloudBeforeICP()
{
	openFileDialogToSaveCloud(intersection_cloud_before_icp_);
}

void HumanFaceReconstruction::saveInetersectionPointCloudAfterICP()
{
	openFileDialogToSaveCloud(intersection_cloud_after_icp_);
}

void HumanFaceReconstruction::openCloudProcessParamsDialog()
{
	CloudProcessParamsDialog dialog(cloud_process_params_);
	dialog.exec();
}

void HumanFaceReconstruction::openCamerasSettingWindow()
{
	if(!cameras_setting_window_)
	{
		cameras_setting_window_ = 
			std::make_unique<CamerasImagesPreviewWindow>(
				left_camera_, right_camera_, cameras_params_
		);
		cameras_setting_window_->setWindowModality(Qt::ApplicationModal);
		cameras_setting_window_->installEventFilter(this);
	}
	
	cameras_setting_window_->showMaximized();
}

bool HumanFaceReconstruction::eventFilter(QObject* obj, QEvent* e)
{
	try
	{
		if (e->type() == QEvent::Paint)
		{
			if(obj == ui.canvasLeft)
				return paintImage(ui.canvasLeft, left_color_image_, left_image_mutex_);
			else if (obj == ui.canvasRight)
				return paintImage(ui.canvasRight, right_color_image_, right_image_mutex_);
		}
		else if(e->type() == QEvent::Show)
		{
			if(obj == cameras_setting_window_.get())
			{
				left_camera_mutex_.lock();
				right_camera_mutex_.lock();
			}
		}
		else if(e->type() == QEvent::Close)
		{
			if(obj == cameras_setting_window_.get())
			{
				left_camera_mutex_.unlock();
				right_camera_mutex_.unlock();
			}
		}
	}
	catch(const std::exception& e)
	{
		auto message = "eventFilter exception!";
		logger.error(message);
		logger.error(e.what());
		
		addLogThreadSafe(message);
		addLogThreadSafe(e.what());
	}

	return QMainWindow::eventFilter(obj, e);
}

bool HumanFaceReconstruction::fetchPointCloudsFromCamera()
{
	std::lock_guard lock_left(left_camera_mutex_);
	std::lock_guard lock_right(right_camera_mutex_);

	try
	{
		left_camera_->setCaptureMode(CameraLib::VSensorPointCloudCamera::PointCloudCapture);
		right_camera_->setCaptureMode(CameraLib::VSensorPointCloudCamera::PointCloudCapture);

		left_camera_->captureStructureLightPatternImages();
		right_camera_->captureStructureLightPatternImages();
	}
	catch(const std::exception& e)
	{
		addLogThreadSafe("Failed to capture pattern frames!");
		addLogThreadSafe(e.what());
		logger.error(e.what());
		return false;
	}

	std::vector<std::shared_ptr<std::thread>> threads;
	volatile bool success = true;

	for(size_t i=0; i<2; i++)
	{
		threads.push_back(
			std::make_shared<std::thread>(
				[this, i, &success]
				{
					auto cameras = 
						std::vector<std::shared_ptr<VSensorPointCloudCamera>> { left_camera_, right_camera_ };
					auto clouds = 
						std::vector<PointCloudPtr> {left_cloud_, right_cloud_};

					try
					{
						cameras[i]->constructPointCloud(pdr_results_[i]);
						UiUtils::getPointCloudFromPDRResult(*pdr_results_[i], clouds[i]);
					}
					catch(const std::exception& e)
					{
						addLogThreadSafe("Failed to construct point cloud!");
						addLogThreadSafe(e.what());
						logger.error(e.what());
						success = false;
					}
				}	
			)
		);
	}

	for(auto& thd: threads)
		thd->join();

	return success;
}

bool HumanFaceReconstruction::paintImage(QWidget* canvas, std::unique_ptr<QImage>& qimage, std::mutex& mutex)
{
	try
	{
		QPainter painter(canvas);

		std::array<int, 4> paint_region{};
		painter.setRenderHint(QPainter::Antialiasing, true);
		auto pen = QPen(QBrush(QColor::fromRgb(255, 0, 0)), 10.0f);
		painter.setPen(pen);

		int width, height;

		{
			std::lock_guard lock(mutex);

			if (qimage)
			{
				width = qimage->width();
				height = qimage->height();
				UiUtils::getImagePaintRegion({ width, height }, { canvas->width(), canvas->height() }, paint_region);
				QRectF region_rect_f(paint_region[0], paint_region[1], paint_region[2], paint_region[3]);
				painter.drawImage(region_rect_f, *qimage);
				return true;
			}
		}

		return false;
	}
	catch(const std::exception& e)
	{
		logger.error(e.what());
		return false;
	}
}


void HumanFaceReconstruction::preProcessPointCloud(PointCloudPtr cloud_ptr)
{
	auto voxel_size = cloud_process_params_.voxel_size;
	auto filter_neighbors = cloud_process_params_.filter_neighbors;
	auto filter_stddev_thd = cloud_process_params_.filter_stddev_thd;
	PclPointCloudUtils<MyPoint>::voxelDownsample(cloud_ptr, *cloud_ptr, voxel_size, voxel_size, voxel_size);
	PclPointCloudUtils<MyPoint>::statisticalOutliersRemove(cloud_ptr, *cloud_ptr, filter_neighbors, filter_stddev_thd);
}


void HumanFaceReconstruction::preProcessPointClouds(PointCloudPtr left_cloud_ptr, PointCloudPtr right_cloud_ptr)
{
	std::vector<std::shared_ptr<std::thread>> threads;
	threads.push_back(
		std::make_shared<std::thread>(
			std::bind(&HumanFaceReconstruction::preProcessPointCloud, this, left_cloud_ptr)
		)
	);
	threads.push_back(
		std::make_shared<std::thread>(
			std::bind(&HumanFaceReconstruction::preProcessPointCloud, this, right_cloud_ptr)
		)
	);
	for(const auto& thd: threads)
		thd->join();
}


Eigen::Matrix4f HumanFaceReconstruction::reigsterClouds(PointCloudPtr left_cloud, PointCloudPtr right_cloud)
{
	PointCloudPtr left_processed_cloud = std::make_shared<PointCloud>(*left_cloud), 
		right_processed_cloud = std::make_shared<PointCloud>(*right_cloud);

	preProcessPointClouds(left_processed_cloud, right_processed_cloud);

	pcl::transformPointCloud(*left_processed_cloud, *left_processed_cloud, stitch_transform_);
	PointCloudPtr left_intersection_cloud = std::make_shared<PointCloud>(), right_intersection_cloud = std::make_shared<PointCloud>();

	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity(), prev_trans = Eigen::Matrix4f::Zero();
	// Calculating the intersections of the point clouds to save icp time cost.
	// Note that the intersection regions are not point correspondences.
	float max_distance = 0, dist_thread = 50;
	PclPointCloudUtils<MyPoint>::cloudsIntersection(
		left_processed_cloud, right_processed_cloud, 
		*left_intersection_cloud, *right_intersection_cloud, 
		dist_thread, max_distance
	);

	PclColorPointCloudUtils<MyPoint>::pclSetColor(*left_intersection_cloud, 255, 0, 0);
	PclColorPointCloudUtils<MyPoint>::pclSetColor(*right_intersection_cloud, 0, 255, 0);
	intersection_cloud_before_icp_->clear();
	intersection_cloud_before_icp_->insert(
		intersection_cloud_before_icp_->end(), left_intersection_cloud->begin(), left_intersection_cloud->end()
	);
	intersection_cloud_before_icp_->insert(
		intersection_cloud_before_icp_->end(), right_intersection_cloud->begin(), right_intersection_cloud->end()
	);

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

	icp.setMaximumIterations(cloud_process_params_.icp_iters);
	icp.setMaxCorrespondenceDistance(dist_thread);
	icp.setInputSource(left_intersection_cloud);
	icp.setInputTarget(right_intersection_cloud);
	icp.align(*left_intersection_cloud);

	return icp.getFinalTransformation() * stitch_transform_;
}

void HumanFaceReconstruction::stitchClouds(
	PointCloudPtr left_cloud, PointCloudPtr right_cloud, const Eigen::Matrix4f transform,
    PointCloudPtr& stitched_cloud
)
{
	// Transform the left cloud to the reference system of right cloud.
	stitched_cloud->clear();
	pcl::transformPointCloud(*left_cloud, *stitched_cloud, transform);
	PointCloudPtr left_cloud_copy = std::make_shared<PointCloud>(*stitched_cloud), 
		right_cloud_copy =  std::make_shared<PointCloud>(*right_cloud);
	stitched_cloud->insert(stitched_cloud->end(), right_cloud->begin(), right_cloud->end());
}

void HumanFaceReconstruction::openFileDialogToSaveCloud(const PointCloudPtr& cloud)
{
	if(!cloud || cloud->empty())
	{
		QMessageBox::information(this, "Tip", "Please construct point cloud first!");
		return;
	}

	auto save_path = QFileDialog::getSaveFileName(nullptr, "Select a path to save point cloud.", QDir::homePath(), "*.ply");
	pcl::io::savePLYFileASCII(save_path.toStdString(), *cloud);
}


void HumanFaceReconstruction::openFileDialogToSaveCloud(const PointNormalCloudPtr& cloud)
{
	if(!cloud || cloud->empty())
	{
		QMessageBox::information(this, "Tip", "Please construct point cloud first!");
		return;
	}

	auto save_path = QFileDialog::getSaveFileName(nullptr, "Select a path to save point cloud.", QDir::homePath(), "*.ply");
	pcl::io::savePLYFileASCII(save_path.toStdString(), *cloud);
}

void HumanFaceReconstruction::addLogThreadSafe(const std::string& message)
{
	QMetaObject::invokeMethod(this, [this, message]
	{
		ui.tbLog->append(QString::fromStdString(message));
	}, Qt::QueuedConnection);
}

void HumanFaceReconstruction::setEnabledThreadSafe(bool enabled)
{
	QMetaObject::invokeMethod(this, [this, enabled] 
	{ 
		setEnabled(enabled); 
		ui.tbLog->setEnabled(true);
	}, 	Qt::QueuedConnection);
}

void HumanFaceReconstruction::visualizeStitchedPointCloudThreadSafe(PointCloudPtr stitched_cloud)
{
	QMetaObject::invokeMethod(
		this, 
		[this, stitched_cloud] 
		{ 
			UiUtils::visualizePointCloud( { {"stitched", stitched_cloud} }, *visualizer_, visualization_params_.point_size);
			vtk_widget_->update();
		}, 
		Qt::BlockingQueuedConnection
	);
}

void HumanFaceReconstruction::updatePCLPointSize()
{
	QMetaObject::invokeMethod(
		this, 
		[this] 
		{ 
			UiUtils::updatePointSize(STITCHED_CLOUD_ID, visualization_params_.point_size, *visualizer_);
		}, 
		Qt::BlockingQueuedConnection
	);
}

void HumanFaceReconstruction::setHintTextVisible(bool visible)
{
	QMetaObject::invokeMethod(
		this, 
		[this, visible] 
		{ 
			ui.lbHint->setVisible(visible);
		}, 
		Qt::QueuedConnection
	);
}

std::shared_ptr<std::thread> HumanFaceReconstruction::connectCamerasAsync(
	std::shared_ptr<CameraLib::VSensorPointCloudCamerasFactory> factory, 
	std::vector<std::string> ids
)
{
	addLogThreadSafe("Connecting cameras...");
	auto thread_connect = std::make_shared<std::thread>(
		[this, factory, ids]
		{
			left_camera_ = factory->createPointCloudCamera(ids[0]);
			right_camera_ = factory->createPointCloudCamera(ids[1]);
			
			std::vector<std::shared_ptr<std::thread>> threads;
			cameras_ = {left_camera_, right_camera_};
			bool success = true;

			for(auto& camera: {left_camera_, right_camera_})
			{
				threads.push_back(std::make_shared<std::thread>(
					[this, camera, &success]
					{
						try
						{
							CameraParams& params = 
								camera == left_camera_ ? cameras_params_->left_camera: cameras_params_->right_camera;
							openCameraAndStartCaptureImages(camera, params, true);
						}
						catch (const std::exception& e)
						{
							std::string message = "Failed to initialize camera!\n";
							message += e.what();
							logger.error(message);
							addLogThreadSafe(message);
							setEnabled(false);
							success = false;
						}
					})
				);
			}

			for(auto& thd: threads)
				thd->join();
			
			int gain_r, gain_g, gain_b;
			left_camera_->getAnalogGain(gain_r, gain_g, gain_b);
			right_camera_->setAnalogGain(gain_r, gain_g, gain_b);

			if(success)
			{
				addLogThreadSafe("Camera connected!");
				addLogThreadSafe("Left camera IP: " + left_camera_->getIP());
				addLogThreadSafe("Right camera IP: " + right_camera_->getIP());
			}
		}
	);

	return thread_connect;
}

std::shared_ptr<std::thread> HumanFaceReconstruction::loadCalibrationParamsAsync(const std::string& path)
{
	auto thread = std::make_shared<std::thread>(
		[this, path]
		{
			DualViewCalibrationParams calib_params;
			calib_params.load(path);
			addLogThreadSafe("Loading calibration params...");

			stitch_params_.copyFromStereoCalibParams(calib_params);

			stitch_transform_ = stitch_params_.eigenTransformation();
			stitch_params_.save(Path::getStitchingTransformParamsFilePath());
		}
	);
	return thread;
}

void HumanFaceReconstruction::setupCaptureThread(
	std::shared_ptr<CameraLib::VSensorPointCloudCamera> camera
)
{
	auto thread = std::make_shared<std::thread>(

		[this, camera]
		{
			auto& camera_mutex = camera == left_camera_ ? left_camera_mutex_: right_camera_mutex_;
			auto& image_mutex = camera == left_camera_ ? left_image_mutex_: right_image_mutex_;
			auto& q_image = camera == left_camera_ ? left_color_image_: right_color_image_;
			auto canvas = camera == left_camera_ ? ui.canvasLeft: ui.canvasRight;

			while(true)
			{
				std::vector<cv::Mat> images(3);
				
				try
				{
					{
						std::lock_guard camera_lock(camera_mutex);

						if(!camera->isOpened())
							return;

						camera->setCaptureMode(CameraLib::VSensorPointCloudCamera::ImageCapture);
						camera->onceCapture(images);

					}

					cv::cvtColor(images[2], images[2], cv::COLOR_BGR2RGB);
					cv::Mat color_image;
					cv::transpose(images[2], color_image);
					cv::flip(color_image, color_image, -1);

					{
						std::lock_guard image_lock(image_mutex);
						if(!q_image)
							UiUtils::createQImage(color_image, q_image);

						UiUtils::updateImageData(color_image, *q_image);
						QMetaObject::invokeMethod(this, [this, canvas] 
						{ 
							canvas->update();
						}, 	Qt::QueuedConnection);
					}
				}
				catch(const std::exception& e)
				{
					logger.error("Capture thread exception:");
					logger.error(e.what());
					addLogThreadSafe("Capture thread exception:");
					addLogThreadSafe(e.what());
					return;
				}
			}
		}

	);

	thread->detach();
}