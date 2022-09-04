#include "ProjectionPatternPreview.h"
#include "UICommon.hpp"

#include <QPainter>
#include <QSpinbox>
#include <QSlider>
#include <QMessageBox>


void updateValueController(QSpinBox* spinbox, QSlider* slider, double value, double min, double max, double nominator=1)
{
	int value_int = static_cast<int>(value / nominator),
	min_int = static_cast<int>(min / nominator),
	max_int = static_cast<int>(max / nominator);
	
	spinbox->setValue(value_int);
	spinbox->setMinimum(min_int);
	spinbox->setMaximum(max_int);

	slider->setValue(value_int);
	slider->setMinimum(min_int);
	slider->setMaximum(max_int);
}


ProjectionPatternPreview::ProjectionPatternPreview(QWidget *parent)
    : QMainWindow(parent), patterns_count_(4), current_pattern_index_(0)
{
    ui.setupUi(this);

    auto factory = CameraLib::MVCameraFactory<CameraLib::PDNImageCamera>();
    auto ids = factory.enumerateCamerasIDs();
    assert(ids.size() == 1);

    camera_ = factory.createMVCamera(ids[0]);
    camera_->open();
	camera_->setExposure(5 * 1000);
	camera_->setGain(1);

	projector_ = std::make_shared<CameraLib::MVCameraProjector>(camera_);

	connect(ui.btnOneShot, &QPushButton::clicked, this, &ProjectionPatternPreview::shotImage);
	connect(ui.btnCapture, &QPushButton::clicked, this, &ProjectionPatternPreview::captureButtonClicked);
	connect(ui.btnNextPattern, &QPushButton::clicked, this, &ProjectionPatternPreview::nextPatternButtonClicked);
	connect(ui.btnContinuesProjection, &QPushButton::clicked, this, &ProjectionPatternPreview::continuesProjectionButtonClicked);

	connect(ui.sliderExposure, &QSlider::valueChanged, this, &ProjectionPatternPreview::sliderValueChanged);
    connect(ui.sbExposure, SIGNAL(valueChanged(int)), ui.sliderExposure, SLOT(setValue(int)));
	connect(ui.sliderGain, &QSlider::valueChanged, this, &ProjectionPatternPreview::sliderValueChanged);
    connect(ui.sbGain, SIGNAL(valueChanged(int)), ui.sliderGain, SLOT(setValue(int)));

	//connect(ui.btnSaveImage, &QPushButton::clicked, this, &ProjectionPatternPreview::saveImage);

	ui.canvs_left->installEventFilter(this);
	ui.canvs_right->installEventFilter(this);

	updateParametersUi(camera_);

	projector_->loadPattern();

	label_rec_frames_ = std::make_unique<QLabel>();
    label_rec_frames_->setMinimumSize(label_rec_frames_->sizeHint());
    label_rec_frames_->setAlignment(Qt::AlignHCenter);



    statusBar()->addWidget(label_rec_frames_.get());
}

ProjectionPatternPreview::~ProjectionPatternPreview()
{
	camera_->close();
}

void ProjectionPatternPreview::shotImage()
{
	std::vector<cv::Mat> images(2);
	
	try
	{
		camera_->onceCapture(images);
	}
	catch(const std::exception& e)
	{
		QMessageBox::warning(this, "Warning", e.what());
		return;
	}

	copyFrame(images, 0);
	copyFrame(images, 1);

	ui.canvs_left->update();
	ui.canvs_right->update();
}

void ProjectionPatternPreview::saveImage(const std::string& filename)
{
}

void ProjectionPatternPreview::captureButtonClicked()
{
	if(camera_->isCapturing())
	{
		camera_->stopContinuesCapture();
		ui.btnCapture->setText("Start capturing");
		ui.btnOneShot->setEnabled(true);
	}
	else
	{
		camera_->startContinuesCapture();
		ui.btnCapture->setText("Stop capturing");
		ui.btnOneShot->setEnabled(false);
		ui.btnSaveImage->setEnabled(true);
	}
}

void ProjectionPatternPreview::sliderValueChanged() const
{
	auto slider = dynamic_cast<QSlider*>(sender());
	assert(slider);

	auto value = slider->value();

	if(slider == ui.sliderExposure)
	{
		ui.sbExposure->setValue(value);
		camera_->setExposure(value * 1000);
	}
	else if(slider == ui.sliderGain)
	{
		ui.sbGain->setValue(value);
		camera_->setGain(value);
	}
}

void ProjectionPatternPreview::frameReadyCallback(cv::InputArray image_data)
{
	static size_t frames_count = 0;

	std::vector<cv::Mat> images(2);
	image_data.getMatVector(images);

	copyFrame(images, 0);
	copyFrame(images, 1);

	frames_count += 1;

	//auto path = R"(C:\Users\ryan\Desktop\images\)";
	//cv::imwrite(path + std::to_string(frames_count) + "left.png", images[0]);
	//cv::imwrite(path + std::to_string(frames_count) + "right.png", images[1]);

	QMetaObject::invokeMethod(this, [this]() { ui.canvs_left->repaint(); }, Qt::BlockingQueuedConnection);	// repaint is done immediately.
	QMetaObject::invokeMethod(this, [this]() { ui.canvs_right->repaint(); }, Qt::BlockingQueuedConnection);
	QMetaObject::invokeMethod(this, [this]() { label_rec_frames_->setText(QString::fromStdString(std::to_string(frames_count))); }, Qt::QueuedConnection);
}

bool ProjectionPatternPreview::eventFilter(QObject* obj, QEvent* e)
{
	if (e->type() == QEvent::Paint)
	{
		if (obj == ui.canvs_left)
			return paintImage(0);
		if (obj == ui.canvs_right)
			return paintImage(1);
	}

	return QMainWindow::eventFilter(obj, e);
}

void ProjectionPatternPreview::copyFrame(const std::vector<cv::Mat>& images, int index)
{
	assert(index >= 0 && index <= 1);

	auto& q_image_mutex = q_image_mutexes_[index];
	auto& q_image = q_images_[index];

	// Copy frame data to qimage for display
	{
		std::lock_guard lock(q_image_mutex);
		if (!q_image)
			UICommon::createQImage(images[index], q_image);
		UICommon::updateImageData(images[index], *q_image);
	}
}

bool ProjectionPatternPreview::paintImage(int index)
{
	auto canvas = index == 0 ? ui.canvs_left: ui.canvs_right;
	QPainter painter(canvas);
	std::array<int, 4> paint_region{};

	auto& qimage_mutex = q_image_mutexes_[index];
	auto& qimage = q_images_[index];
	int width, height;

	{
		std::lock_guard lock(qimage_mutex);

		if (qimage)
		{
			width = qimage->width();
			height = qimage->height();
			UICommon::getImagePaintRegion(
				{ width, height },
				{ canvas->width(), canvas->height() },
				paint_region
			);
			QRectF region_rect_f(paint_region[0], paint_region[1], paint_region[2], paint_region[3]);
			painter.drawImage(region_rect_f, *qimage);
		}
		else
			return false;
	}

	return true;
}

void ProjectionPatternPreview::updateParametersUi(const std::shared_ptr<CameraLib::PDNImageCamera>& camera) const
{
	// Exposure
	double exposure = 0, exp_min = 0, exp_max = 0, exp_step = 0;
	camera_->getExposure(exposure);
	camera_->getExposureRange(exp_min, exp_max, exp_step);
	updateValueController(ui.sbExposure, ui.sliderExposure, exposure, exp_min, exp_max, 1000);
	
	// Gain
	int gain = 0, gain_min = 0, gain_max = 0, gain_step = 0;
	camera_->getGain(gain);
	camera_->getGainRange(gain_min, gain_max, gain_step);
	updateValueController(ui.sbGain, ui.sliderGain, gain, gain_min, gain_max);
}

void ProjectionPatternPreview::nextPatternButtonClicked()
{
	projector_->projectPattern(current_pattern_index_);
	current_pattern_index_ += 1;
	current_pattern_index_ = current_pattern_index_ % patterns_count_;
}

void ProjectionPatternPreview::continuesProjectionButtonClicked()
{
	if(is_capturing_)
	{
		QMessageBox::warning(this, "", "Please stop continues capturing first!");
		return;
	}

	try
	{
		std::thread([this] { projector_->projectPatterns(0, patterns_count_-1); }).detach();
	}
	catch(const std::exception& e)
	{
		QMessageBox::warning(this, "warning!", e.what());
	}

	//current_pattern_index_ = (projector_->getPatternIndex() + 1) % patterns_count_;

}

void ProjectionPatternPreview::showEvent(QShowEvent* event)
{
	if(!is_listener_registered_)
	{
		camera_->registerFrameListener(shared_from_this());
		is_listener_registered_ = true;
	}
}
