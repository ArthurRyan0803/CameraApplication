#pragma once

#include <mutex>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>

#include "AbstractCalibrationWindow.h"
#include "PDRCamera.h"

class TripleViewsCalibrationWindow :
    public AbstractCalibrationWindow
{
private:
    class UI
	{
	public:
        QGroupBox* gbVtk {};
        QHBoxLayout* gbVtkLayout {};
        std::unique_ptr<QVTKOpenGLStereoWidget> stereoWidget {};

        vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
        vtkSmartPointer<vtkRenderer> renderer;
        
        std::array<QHBoxLayout*, 3> canvas_layouts_;
        std::array<QGroupBox*, 3> canvas_gbs_;
        std::array<QWidget*, 3> canvases_;
        
        void setupUi(TripleViewsCalibrationWindow& window)
        {
            for(size_t i=0; i<canvases_.size(); i++)
            {
                canvas_gbs_[i] = new QGroupBox(window.ui_.centralWidget);
				canvas_gbs_[i]->setObjectName(QString::fromStdString("gb_" + std::to_string(i)));

                canvas_layouts_[i] = new QHBoxLayout(canvas_gbs_[i]);
	            canvas_layouts_[i]->setSpacing(6);
	            canvas_layouts_[i]->setContentsMargins(11, 11, 11, 11);
	            canvas_layouts_[i]->setObjectName(QString::fromStdString("gb_layout_" + std::to_string(i)));

            	canvases_[i] = new QWidget(canvas_gbs_[i]);
	            canvas_layouts_[i]->setObjectName(QString::fromStdString("canvas_" + std::to_string(i)));
	            QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
	            sizePolicy.setHorizontalStretch(0);
	            sizePolicy.setVerticalStretch(0);
	            sizePolicy.setHeightForWidth(canvases_[i]->sizePolicy().hasHeightForWidth());
	            canvases_[i]->setSizePolicy(sizePolicy);
	            canvas_layouts_[i]->addWidget(canvases_[i]);
	            window.ui_.viewsGridLayout->addWidget(canvas_gbs_[i], 0, i, 1, 1);

            }
            
            gbVtk = new QGroupBox(window.ui_.centralWidget);
            gbVtk->setObjectName(QString::fromUtf8("gbVtk"));
            QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
            sizePolicy.setHeightForWidth(gbVtk->sizePolicy().hasHeightForWidth());
            gbVtk->setSizePolicy(sizePolicy);
            gbVtkLayout = new QHBoxLayout(gbVtk);
            gbVtkLayout->setSpacing(6);
            gbVtkLayout->setContentsMargins(11, 11, 11, 11);
            gbVtkLayout->setObjectName(QString::fromUtf8("gbVtkLayout"));

            window.ui_.viewsGridLayout->addWidget(gbVtk, 1, 0, 1, 3);

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
            canvas_gbs_[0]->setTitle(QCoreApplication::translate("DualViewsCalibrationWindow", "Left Camera", nullptr));
            canvas_gbs_[1]->setTitle(QCoreApplication::translate("DualViewsCalibrationWindow", "Right Camera", nullptr));
            canvas_gbs_[2]->setTitle(QCoreApplication::translate("DualViewsCalibrationWindow", "Color Camera", nullptr));
            gbVtk->setTitle(QCoreApplication::translate("DualViewsCalibrationWindow", "Stereo Position", nullptr));
        }
	};
    
    UI custom_ui_ {};

    std::vector<std::unique_ptr<QImage>> q_images_;
    std::vector<std::mutex> q_images_mutex_;

	void cameraFrameReadyCallback(cv::InputArray image_data) override;
	void startCalibBoardDetectThread() override;
	void stopCalibBoardDetectThread() override;

    bool paintImage(int index);
    
protected:
	void shotImage() override;
	void saveImage(const std::string& filename) override;
	void abortCalibration() override;
	void shotCalibImage() override;
	void calibrate(const std::string& folder) override;
    
    bool eventFilter(QObject* obj, QEvent* e) override;


public:
	TripleViewsCalibrationWindow(const std::shared_ptr<CameraLib::Camera>& camera, QWidget* parent = nullptr);
};

