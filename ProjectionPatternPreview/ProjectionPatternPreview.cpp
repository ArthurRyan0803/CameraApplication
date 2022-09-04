#include "ProjectionPatternPreview.h"
#include "UICommon.hpp"

#include <QPainter>
#include <QSpinbox>
#include <QSlider>
#include <QMessageBox>
#include <QDebug>
#include <QFileDialog>

#include <cmath>
#include <math.h>


#define M_PI       3.14159265358979323846
#define GRAY_CODE_INVALID 0xff
#define FB_TOLERENCE 20


cv::Mat ProjectionPatternPreview::calPhaseMat(const std::vector<cv::Mat>& images)
{
	cv::Mat phase_image(images[0].rows, images[0].cols, CV_64FC1);

	for(int y=0; y<images[0].rows; y++)
	{
		for(int x=0; x<images[0].cols; x++)
		{
			auto i1 = images[0].at<uint8_t>(y, x);
			auto i2 = images[1].at<uint8_t>(y, x);
			auto i3 = images[2].at<uint8_t>(y, x);
			auto i4 = images[3].at<uint8_t>(y, x);

			auto phase = std::atan2((i4 - i2), (i1 - i3));

			phase_image.at<double>(y, x) = phase;
		}
	}

	return phase_image;
}


cv::Mat ProjectionPatternPreview::calGrayCodeOrderMap(const std::vector<cv::Mat>& images, cv::OutputArray thd_mat)
{
	cv::Mat dark_image = images[0], bright_image = images[1];

	cv::Mat thd_image(dark_image.rows, dark_image.cols, CV_8UC1);

	for(size_t r=0; r<dark_image.rows; r++)
	{
		for(size_t c=0; c<dark_image.cols; c++)
		{
			auto max = bright_image.at<uint8_t>(r, c), min = dark_image.at<uint8_t>(r, c);

			thd_image.at<uint8_t>(r, c) = std::abs(max - min) > FB_TOLERENCE ? (max - min) / 2: 0;
		}
	}

	//std::vector<cv::Mat> binary_images;
	cv::Mat gray_code_mat(thd_image.rows, thd_image.cols, CV_8UC1, cv::Scalar(0));

	for(size_t i=6; i<patterns_count_; i++)
	{
		cv::Mat image = images.at(i);

		for(size_t r=0; r<image.rows; r++)
			for(size_t c=0; c<image.cols; c++)
			{
				auto thd = thd_image.at<uint8_t>(r, c);
				auto& code = gray_code_mat.at<uint8_t>(r, c);

				if(thd == 0)
					code = GRAY_CODE_INVALID;
				else
				{
					code = (code << 1) | (image.at<uint8_t>(r, c) > thd ? 0x01: 0);
				}	
			}
	}
	
	if(thd_mat.kind() == cv::_InputArray::MAT)
		thd_mat.assign(thd_image);

	return gray_code_mat;
}


cv::Mat ProjectionPatternPreview::phaseMatToImage(const cv::Mat& phase_mat)
{
	cv::Mat gray_image(phase_mat.rows, phase_mat.cols, CV_8UC1);

	for(int y=0; y<phase_mat.rows; y++)
	{
		for(int x=0; x<phase_mat.cols; x++)
		{
			gray_image.at<uint8_t>(y, x) = static_cast<uint8_t>(phase_mat.at<double>(y, x) / 2 * M_PI * 255);
		}
	}

	return gray_image;
}


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
    : QMainWindow(parent), patterns_count_(10), current_pattern_index_(0)
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

	connect(ui.btnCapture, &QPushButton::clicked, this, &ProjectionPatternPreview::captureButtonClicked);
	connect(ui.btnNextPattern, &QPushButton::clicked, this, &ProjectionPatternPreview::nextPatternButtonClicked);
	connect(ui.btnPrevPattern, &QPushButton::clicked, this, &ProjectionPatternPreview::prevPatternButtonClicked);
	connect(ui.btnContinuesProjection, &QPushButton::clicked, this, &ProjectionPatternPreview::continuesProjectionButtonClicked);

	connect(ui.sliderExposure, &QSlider::valueChanged, this, &ProjectionPatternPreview::sliderValueChanged);
    connect(ui.sbExposure, SIGNAL(valueChanged(int)), ui.sliderExposure, SLOT(setValue(int)));
	connect(ui.sliderGain, &QSlider::valueChanged, this, &ProjectionPatternPreview::sliderValueChanged);
    connect(ui.sbGain, SIGNAL(valueChanged(int)), ui.sliderGain, SLOT(setValue(int)));

	connect(ui.act_save_seq_patterns, &QAction::triggered, this, &ProjectionPatternPreview::saveSeqPatterns);
	connect(ui.act_cal_phase, &QAction::triggered, this, &ProjectionPatternPreview::openDirAndCalPhase);

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
	if(camera_->isCapturing())
		camera_->stopContinuesCapture();
	camera_->close();
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
	}
	else
	{
		camera_->startContinuesCapture();
		ui.btnCapture->setText("Stop capturing");
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

void ProjectionPatternPreview::saveSeqPatterns()
{
	if(pattern_images_[0].empty() || pattern_images_[1].empty())
		return;

	auto path = QFileDialog::getExistingDirectory(this, "Select folder!");
	if(path.isEmpty())
		return;

	for(size_t i = 0; i < patterns_count_; i++)
	{
		auto left_path = path + "/left_" + std::to_string(i).c_str() + ".png";
		auto right_path = path + "/right_" + std::to_string(i).c_str() + ".png";

		cv::imwrite(left_path.toStdString(), pattern_images_[0][i]);
		cv::imwrite(right_path.toStdString(), pattern_images_[1][i]);
	}
}

void ProjectionPatternPreview::openDirAndCalPhase()
{
	auto path = QFileDialog::getExistingDirectory(this, "Select folder!");
	if(path.isEmpty())
		return;

	std::vector<cv::Mat> left_images, right_images;
	for(size_t i = 0; i < patterns_count_; i++)
	{
		auto left_path = path + "/left_" + std::to_string(i).c_str() + ".png";
		auto right_path = path + "/right_" + std::to_string(i).c_str() + ".png";

		cv::Mat left_image, right_image;
		left_image = cv::imread(left_path.toStdString(), cv::IMREAD_GRAYSCALE);
		right_image = cv::imread(right_path.toStdString(), cv::IMREAD_GRAYSCALE);

		left_images.push_back(left_image);
		right_images.push_back(right_image);
	}

	try
	{
		auto left_phase_mat = calPhaseMat(left_images);
		auto left_phase_image = phaseMatToImage(left_phase_mat);
		auto right_phase_mat = calPhaseMat(right_images);
		auto right_phase_image = phaseMatToImage(right_phase_mat);

		cv::imwrite((path + "/left_phase.png").toStdString(), left_phase_image);
		cv::imwrite((path + "/right_phase.png").toStdString(), right_phase_image);

		cv::Mat left_thd_map, right_thd_map;
		auto left_gray_order_mat = calGrayCodeOrderMap(left_images, left_thd_map);
		auto right_gray_order_mat = calGrayCodeOrderMap(right_images, right_thd_map);

		cv::imwrite((path + "/left_code_mat.png").toStdString(), left_gray_order_mat);
		cv::imwrite((path + "/right_code_mat.png").toStdString(), right_gray_order_mat);

		cv::imwrite((path + "/left_thd_image.png").toStdString(), left_thd_map);
		cv::imwrite((path + "/right_thd_image.png").toStdString(), right_thd_map);

	}
	catch(const std::exception& e)
	{
		QMessageBox::warning(this, "warning!", e.what());
	}
}

void ProjectionPatternPreview::frameReadyCallback(cv::InputArray image_data)
{
	volatile static size_t frames_count = 0;

	std::vector<cv::Mat> images(2);
	image_data.getMatVector(images);

	copyFrame(images, 0);
	copyFrame(images, 1);

	frames_count += 1;

	//auto path = R"(C:\Users\ryan\Desktop\images\)";
	//cv::imwrite(path + std::to_string(frames_count) + "left.png", images[0]);
	//cv::imwrite(path + std::to_string(frames_count) + "right.png", images[1]);

	QMetaObject::invokeMethod(this, [this]() { ui.canvs_left->repaint(); }, Qt::QueuedConnection);	// repaint is done immediately.
	QMetaObject::invokeMethod(this, [this]() { ui.canvs_right->repaint(); }, Qt::QueuedConnection);
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

	if(is_continus_projecting_)
	{
		assert(pattern_images_[index].size() <= patterns_count_);
		pattern_images_[index].emplace_back(images[index].clone());
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
	try
	{
		current_pattern_index_ += 1;
		current_pattern_index_ = current_pattern_index_ % patterns_count_;
		projector_->projectPattern(current_pattern_index_);
		//qDebug() << current_pattern_index_;
	}
	catch(const std::exception& e)
	{
		QMessageBox::warning(this, "Warning!", e.what());
	}
}

void ProjectionPatternPreview::prevPatternButtonClicked()
{
	try
	{
		current_pattern_index_ = current_pattern_index_ - 1 + patterns_count_;
		current_pattern_index_ = current_pattern_index_ % patterns_count_;
		projector_->projectPattern(current_pattern_index_);
	}
	catch(const std::exception& e)
	{
		QMessageBox::warning(this, "Warning!", e.what());
	}
}

void ProjectionPatternPreview::continuesProjectionButtonClicked()
{
	if(is_capturing_)
	{
		QMessageBox::warning(this, "", "Please stop continues capturing first!");
		return;
	}

	pattern_images_[0].clear();
	pattern_images_[1].clear();

	setEnabled(false);
	std::thread([this] 
	{ 
		try
		{
			is_continus_projecting_ = true;
			projector_->projectPatterns(0, patterns_count_-1);
		}
		catch(const std::exception& e)
		{
			QMetaObject::invokeMethod(this, [this, &e] { QMessageBox::warning(this, "Warning!", e.what()); }, Qt::BlockingQueuedConnection);
		}

		is_continus_projecting_ = false;

		QMetaObject::invokeMethod(this, [this] { setEnabled(true); }, Qt::QueuedConnection);

		current_pattern_index_ = patterns_count_-1;
	}).detach();

}

void ProjectionPatternPreview::showEvent(QShowEvent* event)
{
	if(!is_listener_registered_)
	{
		camera_->registerFrameListener(shared_from_this());
		is_listener_registered_ = true;
	}
}
