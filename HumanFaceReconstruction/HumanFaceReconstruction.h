#pragma once

#include "ui_HumanFaceReconstruction.h"
#include "VSensorPointCloudCamera.h"
#include "CalibrationParams.hpp"
#include "Params.hpp"

#include <QVTKOpenGLStereoWidget.h>
#include <QtWidgets/QMainWindow>

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "VSensorPointCloudCamerasFactory.hpp"
#include "CamerasImagesPreviewWindow.h"


class HumanFaceReconstruction : public QMainWindow
{
    Q_OBJECT

public:
    HumanFaceReconstruction(QWidget *parent = nullptr);
    ~HumanFaceReconstruction() override;

private:
    typedef pcl::PointXYZRGB MyPoint;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;

    Ui::HumanFaceReconstructionClass ui;

	// Vars for point cloud visualize
    std::unique_ptr<pcl::visualization::PCLVisualizer> visualizer_;
    std::unique_ptr<QVTKOpenGLStereoWidget> vtk_widget_;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> render_window_;
    vtkSmartPointer<vtkRenderer> renderer_;
    PointCloudPtr left_cloud_, right_cloud_, stitched_cloud_, intersection_cloud_before_icp_, intersection_cloud_after_icp_;
    std::array<std::unique_ptr<VSensorResult>, 2> pdr_results_;
    VisualizationParams visualization_params_;

    std::array<cv::Mat, 2> color_images_with_mask_;
    
    std::unique_ptr<QImage> left_color_image_, right_color_image_;
    std::mutex left_image_mutex_, right_image_mutex_;

    // Cameras
    std::shared_ptr<CameraLib::VSensorPointCloudCamera> left_camera_, right_camera_;
    std::array<std::shared_ptr<CameraLib::VSensorPointCloudCamera>, 2> cameras_;
    std::shared_ptr<DualCamerasParams> cameras_params_;
    std::unique_ptr<CamerasImagesPreviewWindow> cameras_setting_window_;

    // Vars for point clouds stitching.
    Eigen::Matrix4f stitch_transform_;
    StitchingTransformParams stitch_params_;
    PointCloudProcessParams cloud_process_params_;

    // Vars for continues capture
    std::shared_ptr<std::thread> left_capture_thread_, right_capture_thread_;
    std::mutex left_camera_mutex_, right_camera_mutex_;

    // Thread safe ui access methods.
    void addLogThreadSafe(const std::string& message);
    void setEnabledThreadSafe(bool enabled);
    void visualizeStitchedPointCloudThreadSafe(PointCloudPtr stitched_cloud);
    void updatePCLPointSize();
    void setHintTextVisible(bool visible);

    // Async methods.
    std::shared_ptr<std::thread> connectCamerasAsync(
		std::shared_ptr<CameraLib::VSensorPointCloudCamerasFactory> factory, 
		std::vector<std::string> ids
	);
    std::shared_ptr<std::thread> loadCalibrationParamsAsync(const std::string& path);

    // Ui modification methods (not thread safe).
    void addVtkWidget(
		const QGroupBox& group_box, std::unique_ptr<QVTKOpenGLStereoWidget>& vtk_widget,
		vtkSmartPointer<vtkRenderer>& renderer, vtkSmartPointer<vtkGenericOpenGLRenderWindow>& render_window,
		std::unique_ptr<pcl::visualization::PCLVisualizer>& visualizer, const std::string& id
	) const;

	// Slots
    void constructHumanFace();
    void saveStitchedPointCloud();
    void saveLeftPointCloud();
    void saveRightPointCloud();
    void saveInetersectionPointCloudBeforeICP();
    void saveInetersectionPointCloudAfterICP();
    void openCloudProcessParamsDialog();
    void openCamerasSettingWindow();

    bool eventFilter(QObject* obj, QEvent* e);
    bool paintImage(QWidget* canvas, std::unique_ptr<QImage>& qimage, std::mutex& mutex);

    // Point clouds process
    bool fetchPointCloudsFromCamera();
    void preProcessPointCloud(PointCloudPtr cloud_ptr);
    void preProcessPointClouds(PointCloudPtr left_cloud_ptr, PointCloudPtr right_cloud_ptr);
    Eigen::Matrix4f reigsterClouds(PointCloudPtr left_cloud, PointCloudPtr right_cloud);
    void stitchClouds(
	    PointCloudPtr left_cloud, PointCloudPtr right_cloud, const Eigen::Matrix4f transform,
        PointCloudPtr& stitched_cloud
    );

    void openFileDialogToSaveCloud(const PointCloudPtr& cloud);

    void openCameraAndStartCaptureImages(
        std::shared_ptr<CameraLib::VSensorPointCloudCamera> camera, const CameraParams& params, bool initialize
    );
    void continuesCapture();
    void setupCaptureThread(std::shared_ptr<CameraLib::VSensorPointCloudCamera> camera);
    
};
