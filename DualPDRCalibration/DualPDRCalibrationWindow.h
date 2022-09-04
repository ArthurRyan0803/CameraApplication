#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_DualPDRCalibrationWindow.h"

#include "PDRImageCamera.h"
#include "ParameterDialog.h"
#include "CalibrationBoardSettings.hpp"
#include <opencv2/opencv.hpp>
#include <CalibrationPatternMethod.h>
#include <QVTKOpenGLStereoWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <pcl/visualization/pcl_visualizer.h>


class DualPDRCalibrationWindow : public QMainWindow
{
    Q_OBJECT

public:
    DualPDRCalibrationWindow(QWidget *parent = nullptr);
    ~DualPDRCalibrationWindow() override;
    
private:
    Ui::DualPDRCalibrationWindowClass ui_ {};

    std::unique_ptr<QVTKOpenGLStereoWidget> vtk_widget_;
    std::unique_ptr<pcl::visualization::PCLVisualizer> pcl_visualizer_;

    volatile bool is_capture_ {false};
    volatile bool is_alive_ {true};
    volatile bool is_calibrting_ { false };
    std::map<std::string, std::shared_ptr<CameraLib::PDRImageCamera>> cameras_;
    std::vector<std::string> cam_ips_;
    std::string left_cam_ip_, right_cam_ip_;

    std::array<std::unique_ptr<QImage>, 2> q_images_;
    std::array<std::unique_ptr<cv::Mat>, 2> frame_buffers_;
    std::array<std::vector<cv::Point2f>, 2> frames_key_points_;
    
    std::array<std::mutex, 2> q_images_mutexes_;
    std::array<std::mutex, 2> frame_buffer_mutexes_;
    std::array<std::mutex, 2> key_points_mutexes_;

	std::unique_ptr<ParameterDialog> param_dialog_;

    std::string save_folder_;
    CalibrationBoardSettings calib_settings_;
    Pattern calib_parttern_;

    void updateImageBuffers(const cv::Mat& image, int cam_index);
    bool paintImage(int cam_index);

    void findingCalibBoardPattern(int cam_index);

    // Events
    bool eventFilter(QObject* obj, QEvent* e) override;

	// Slots
    void comboboxSelectedTextChanged();
    void captureButtonClicked();
    void oneshotButtonClicked();
    void selectSaveFolder();
    void saveImages();
    void calibration();

    void calibPatternChanged();
    void calibHorizontalCountChanged();
    void calibVerticalCountChanged();
    void calibIntervalChanged();

    // Camera
    std::shared_ptr<CameraLib::PDRImageCamera>& leftCamera();
    std::shared_ptr<CameraLib::PDRImageCamera>& rightCamera();

    void leftCameraFrameReadyCallback(cv::InputArray images);
    void rightCameraFrameReadyCallback(cv::InputArray images);
    void openParameterDialog();
};
