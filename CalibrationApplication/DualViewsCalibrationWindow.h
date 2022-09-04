#pragma once
#include "AbstractCalibrationWindow.h"
#include "vtkRenderer.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include <memory>


class DualViewsCalibrationWindow: public AbstractCalibrationWindow
{
private:
	class UI
	{
	public:
        QGroupBox* gbLeft;
        QHBoxLayout* gbLeftLayout;
        QWidget* canvasLeft;

        QGroupBox* gbRight;
        QHBoxLayout* gbRightLayout;
        QWidget* canvasRight;

        QGroupBox* gbVtk;
        QHBoxLayout* gbVtkLayout;
        std::unique_ptr<QVTKOpenGLStereoWidget> stereoWidget;

        vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
        vtkSmartPointer<vtkRenderer> renderer;

        
        void setupUi(DualViewsCalibrationWindow& window)
        {
            gbLeft = new QGroupBox(window.ui_.centralWidget);
            gbLeft->setObjectName(QString::fromUtf8("gbLeft"));
            gbLeftLayout = new QHBoxLayout(gbLeft);
            gbLeftLayout->setSpacing(6);
            gbLeftLayout->setContentsMargins(11, 11, 11, 11);
            gbLeftLayout->setObjectName(QString::fromUtf8("gbLeftLayout"));

            canvasLeft = new QWidget(gbLeft);
            canvasLeft->setObjectName(QString::fromUtf8("canvasLeft"));
            QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
            sizePolicy.setHorizontalStretch(0);
            sizePolicy.setVerticalStretch(0);
            sizePolicy.setHeightForWidth(canvasLeft->sizePolicy().hasHeightForWidth());
            canvasLeft->setSizePolicy(sizePolicy);
            gbLeftLayout->addWidget(canvasLeft);
            
            window.ui_.viewsGridLayout->addWidget(gbLeft, 0, 0, 1, 1);

            gbRight = new QGroupBox(window.ui_.centralWidget);
            gbRight->setObjectName(QString::fromUtf8("gbRight"));
            gbRightLayout = new QHBoxLayout(gbRight);
            gbRightLayout->setSpacing(6);
            gbRightLayout->setContentsMargins(11, 11, 11, 11);
            gbRightLayout->setObjectName(QString::fromUtf8("horizontalLayout_3"));
            canvasRight = new QWidget(gbRight);
            canvasRight->setObjectName(QString::fromUtf8("canvasRight"));
            sizePolicy.setHeightForWidth(canvasRight->sizePolicy().hasHeightForWidth());
            canvasRight->setSizePolicy(sizePolicy);

            gbRightLayout->addWidget(canvasRight);
            
            window.ui_.viewsGridLayout->addWidget(gbRight, 0, 1, 1, 1);


            gbVtk = new QGroupBox(window.ui_.centralWidget);
            gbVtk->setObjectName(QString::fromUtf8("gbVtk"));
            sizePolicy.setHeightForWidth(gbVtk->sizePolicy().hasHeightForWidth());
            gbVtk->setSizePolicy(sizePolicy);
            gbVtkLayout = new QHBoxLayout(gbVtk);
            gbVtkLayout->setSpacing(6);
            gbVtkLayout->setContentsMargins(11, 11, 11, 11);
            gbVtkLayout->setObjectName(QString::fromUtf8("gbVtkLayout"));

            window.ui_.viewsGridLayout->addWidget(gbVtk, 1, 0, 1, 2);

            stereoWidget = std::make_unique<QVTKOpenGLStereoWidget>();
            stereoWidget->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
            gbVtk->layout()->addWidget(stereoWidget.get());

            renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
            renderer = vtkSmartPointer<vtkRenderer>::New();
            renderWindow->AddRenderer(renderer);
            stereoWidget->setRenderWindow(renderWindow);

            retranslateUi(window);
        }

        void retranslateUi(QMainWindow& DualViewsCalibrationWindow)
        {
            gbLeft->setTitle(QCoreApplication::translate("DualViewsCalibrationWindow", "Left Camera", nullptr));
            gbRight->setTitle(QCoreApplication::translate("DualViewsCalibrationWindow", "Right Camera", nullptr));
            gbVtk->setTitle(QCoreApplication::translate("DualViewsCalibrationWindow", "Stereo Position", nullptr));
        }
	};
    
    UI custom_ui_ {};
    volatile bool check_calib_board_;
    std::unique_ptr<pcl::visualization::PCLVisualizer> pcl_visualizer_;
    std::vector<std::shared_ptr<cv::Mat>> left_calib_images_, right_calib_images_;

    std::array<std::unique_ptr<QImage>, 2> q_images_;
    std::array<std::unique_ptr<cv::Mat>, 2> frame_buffers_;
    std::array<std::vector<cv::Point2f>, 2> frames_key_points_;

    std::array<std::mutex, 2> q_image_mutexes_;
    std::array<std::mutex, 2> frame_buffer_mutexes_;
    std::array<std::mutex, 2> key_points_mutexes_;

    std::array<std::unique_ptr<std::thread>, 2> board_detect_threads_;
    
    bool eventFilter(QObject* obj, QEvent* e) override;
    bool paintImage(int index);

    void startCalibBoardDetectThread() override;
    void stopCalibBoardDetectThread() override;
    void findingCalibBoardPattern(int index);
    void closeEvent(QCloseEvent* event) override;
    void shotImage() override;
    void saveImage(const std::string& filename) override;
    void abortCalibration() override;
    void copyFrame(const std::vector<cv::Mat>& images, int index);
    void shotCalibImage() override;
    void calibrate(const std::string& folder) override;
    void cameraFrameReadyCallback(cv::InputArray image_data) override;

    void DualViewsCalibrationWindow::visualizeCalibPlanar(
	    const CalibrationBoardSettings& settings, double r, double g, double b, const std::string& id_prefix
    ) const;

    void DualViewsCalibrationWindow::visualizeCamera(
        const PlanarCalibrationParams& camera_params, const cv::Mat& R, const std::string& id,
        double r, double g, double b
    ) const;

public:
    DualViewsCalibrationWindow(const std::shared_ptr<CameraLib::Camera>& camera, QWidget* parent = nullptr);
    ~DualViewsCalibrationWindow() override = default;
};

