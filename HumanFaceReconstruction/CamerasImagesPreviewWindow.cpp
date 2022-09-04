#include "CamerasImagesPreviewWindow.h"
#include <functional>
#include <QPainter>


#include "UiUtils.h"

using namespace CameraLib;


static Logger& logger = GET_LOGGER();


void loadCameraParam(const VSensorPointCloudCamera& camera, const CameraParams& params)
{
	camera.setExposure(params.gray_exposure, VSensorPointCloudCamera::Gray);
	camera.setExposure(params.color_exposure, VSensorPointCloudCamera::Color);

	camera.setAnalogGain(params.gray_gain, VSensorPointCloudCamera::Gray);
	camera.setAnalogGain(params.color_gain, VSensorPointCloudCamera::Color);
}

void updateValueController(
	QSpinBox* spinbox, double value, double min, double max, double nominator=1
)
{
	int value_int = static_cast<int>(value / nominator),
	min_int = static_cast<int>(min / nominator),
	max_int = static_cast<int>(max / nominator);
	
	spinbox->setValue(value_int);
	spinbox->setMinimum(min_int);
	spinbox->setMaximum(max_int);
}

CamerasImagesPreviewWindow::CamerasImagesPreviewWindow(
	std::shared_ptr<VSensorPointCloudCamera> left_camera, std::shared_ptr<VSensorPointCloudCamera> right_camera, 
	std::shared_ptr<DualCamerasParams> cameras_params, QWidget *parent
):
	QMainWindow(parent), cameras_params_(cameras_params),
	left_camera_(left_camera), right_camera_(right_camera)
{
	try
	{
		ui.setupUi(this);
	
		connect(ui.btnShot, &QPushButton::clicked, this, &CamerasImagesPreviewWindow::oneShot);
		connect(ui.btnOK, &QPushButton::clicked, this, &CamerasImagesPreviewWindow::overwriteParams);
		connect(ui.btnRestore, &QPushButton::clicked, this, &CamerasImagesPreviewWindow::restoreParams);
		connect(ui.btnCancel, &QPushButton::clicked, this, &CamerasImagesPreviewWindow::close);

		connect(ui.rbLeftCamera, &QPushButton::toggled, this, &CamerasImagesPreviewWindow::selectedCameraChanged);
		connect(ui.rbRightCamera, &QPushButton::toggled, this, &CamerasImagesPreviewWindow::selectedCameraChanged);

		connect(ui.sbGrayExposure, &QSpinBox::textChanged, this, &CamerasImagesPreviewWindow::grayExpValueChanged);
		connect(ui.sbColorExposure, &QSpinBox::textChanged, this, &CamerasImagesPreviewWindow::colorExpValueChanged);

		loadCameraParam(*left_camera_, cameras_params_->left_camera);
		loadCameraParam(*right_camera_, cameras_params_->right_camera);
	
		ui.rbLeftCamera->toggle();

		canvas_ = {
			ui.canvasLeftGray1, ui.canvasLeftGray2, ui.canvasLeftColor, 
			ui.canvasRightGray1, ui.canvasRightGray2, ui.canvasRightColor
		};

		for(auto canvas: canvas_)
			canvas->installEventFilter(this);
	}
	catch(const std::exception& e)
	{
		logger.error(e.what());
		throw e;
	}
}

CamerasImagesPreviewWindow::~CamerasImagesPreviewWindow() = default;

void CamerasImagesPreviewWindow::oneShot()
{
	std::vector<cv::Mat> left_images(3), right_images(3);
	left_camera_->onceCapture(left_images);
	right_camera_->onceCapture(right_images);

	if(left_images.empty() || right_images.empty())
		return;

	cv::cvtColor(left_images[2], left_images[2], cv::COLOR_BGR2RGB);
	cv::cvtColor(right_images[2], right_images[2], cv::COLOR_BGR2RGB);

	std::vector<cv::Mat> images;
	images.insert(images.end(), left_images.begin(), left_images.end());
	images.insert(images.end(), right_images.begin(), right_images.end());

	for(size_t i=0; i<images.size(); i++)
	{
		try
		{
			{
				std::lock_guard lock_qimage(mutexes_[i]);
				if (!q_images_[i])
					UiUtils::createQImage(images[i], q_images_[i]);
				UiUtils::updateImageData(images[i], *q_images_[i]);
			}

			canvas_[i]->update();
		}
		catch(const std::exception& e)
		{
			logger.error(e.what());
			throw e;
		}
	}
}

void CamerasImagesPreviewWindow::selectedCameraChanged()
{
	selected_camera_ = ui.rbLeftCamera->isChecked() ? left_camera_: right_camera_;
	updateParametersUi(*selected_camera_);
}

void CamerasImagesPreviewWindow::grayExpValueChanged()
{
	QSpinBox* box = static_cast<QSpinBox*>(sender());
	selected_camera_->setExposure(box->value(), VSensorPointCloudCamera::Gray);
}

void CamerasImagesPreviewWindow::colorExpValueChanged()
{
	QSpinBox* box = static_cast<QSpinBox*>(sender());
	selected_camera_->setExposure(box->value(), VSensorPointCloudCamera::Color);
}

void CamerasImagesPreviewWindow::overwriteParams()
{
	double exp = 0;
	int gain = 0;
	 
	left_camera_->getExposure(exp, CameraLib::VSensorPointCloudCamera::Gray);
	cameras_params_->left_camera.gray_exposure = exp;

	left_camera_->getExposure(exp, CameraLib::VSensorPointCloudCamera::Color);
	cameras_params_->left_camera.color_exposure = exp;

	right_camera_->getExposure(exp, CameraLib::VSensorPointCloudCamera::Gray);
	cameras_params_->right_camera.gray_exposure = exp;

	right_camera_->getExposure(exp, CameraLib::VSensorPointCloudCamera::Color);
	cameras_params_->right_camera.color_exposure = exp;

	close();
}

void CamerasImagesPreviewWindow::restoreParams()
{
	loadCameraParam(*left_camera_, cameras_params_->left_camera);
	loadCameraParam(*right_camera_, cameras_params_->right_camera);
}

bool CamerasImagesPreviewWindow::eventFilter(QObject* obj, QEvent* e)
{
	if (e->type() == QEvent::Paint)
	{
		for(size_t i=0; i<6; i++)
		{
			if(obj != canvas_[i]) continue;

			{
				std::lock_guard lock(mutexes_[i]);
				return paintImage(canvas_[i], q_images_[i]);
			}
		}
	}

	return QMainWindow::eventFilter(obj, e);
}

bool CamerasImagesPreviewWindow::paintImage(QWidget* canvas, std::unique_ptr<QImage>& qimage)
{
	try
	{
		QPainter painter(canvas);

		std::array<int, 4> paint_region{};
		painter.setRenderHint(QPainter::Antialiasing, true);
		auto pen = QPen(QBrush(QColor::fromRgb(255, 0, 0)), 10.0f);
		painter.setPen(pen);

		int width, height;

		if (qimage)
		{
			width = qimage->width();
			height = qimage->height();
			UiUtils::getImagePaintRegion({ width, height }, { canvas->width(), canvas->height() }, paint_region);
			QRectF region_rect_f(paint_region[0], paint_region[1], paint_region[2], paint_region[3]);
			painter.drawImage(region_rect_f, *qimage);
			return true;
		}

		return false;
	}
	catch(const std::exception& e)
	{
		logger.error(e.what());
		return false;
	}
}

void CamerasImagesPreviewWindow::updateParametersUi(const VSensorPointCloudCamera& camera) const
{
	// Exposure
	// Exposure Gray
	double exposure = 0, exp_min = 0, exp_max = 150, exp_step = 1;
	camera.getExposure(exposure, VSensorPointCloudCamera::Gray);
	updateValueController(ui.sbGrayExposure, exposure, exp_min, exp_max);
	
	// Exposure Color
	camera.getExposure(exposure, VSensorPointCloudCamera::Color);
	updateValueController(ui.sbColorExposure, exposure, exp_min, exp_max);
}

void CamerasImagesPreviewWindow::showEvent( QShowEvent* e )
{
	left_camera_->setCaptureMode(CameraLib::VSensorPointCloudCamera::ImageCapture);
	right_camera_->setCaptureMode(CameraLib::VSensorPointCloudCamera::ImageCapture);
}

void CamerasImagesPreviewWindow::closeEvent( QCloseEvent* e )
{
	left_camera_->setCaptureMode(CameraLib::VSensorPointCloudCamera::PointCloudCapture);
	right_camera_->setCaptureMode(CameraLib::VSensorPointCloudCamera::PointCloudCapture);
}
