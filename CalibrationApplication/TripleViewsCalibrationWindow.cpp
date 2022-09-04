#include <QPainter>
#include "TripleViewsCalibrationWindow.h"

#include <QMetaMethod>

#include "Utils.hpp"
#include "UICommon.h"


TripleViewsCalibrationWindow::TripleViewsCalibrationWindow(const std::shared_ptr<CameraLib::Camera>& camera, QWidget* parent)
	:AbstractCalibrationWindow(camera, parent), q_images_(3), q_images_mutex_(3)
{
	custom_ui_.setupUi(*this);

	for(auto canvas: custom_ui_.canvases_)
		canvas->installEventFilter(this);
	
}

void TripleViewsCalibrationWindow::cameraFrameReadyCallback(cv::InputArray image_data)
{
	assert(image_data.kind() == cv::_InputArray::STD_VECTOR_MAT);

	cv::Mat left, right, color;
	std::vector images {left, right, color};
	image_data.getMatVector(images);
	
	for(size_t i=0; i<images.size(); i++)
	{
		{
			std::lock_guard lock(q_images_mutex_[0]);
			auto& q_image = q_images_[i];
			if(!q_image)
				UICommon::createQImage(images[i], q_image);
			UICommon::updateImageData(images[i], *q_image);
		}
		QMetaObject::invokeMethod(this, [this, i]() { custom_ui_.canvases_[i]->update(); }, Qt::QueuedConnection);
	}
}

void TripleViewsCalibrationWindow::startCalibBoardDetectThread()
{

}

void TripleViewsCalibrationWindow::stopCalibBoardDetectThread()
{

}

bool TripleViewsCalibrationWindow::paintImage(int index)
{
	auto canvas = custom_ui_.canvases_[index];
	QPainter painter(canvas);

	std::array<int, 4> paint_region{};
	painter.setRenderHint(QPainter::Antialiasing, true);
	auto pen = QPen(QBrush(QColor::fromRgb(255, 0, 0)), 10.0f);
	painter.setPen(pen);
	
	{
		std::lock_guard lock(q_images_mutex_[index]);
		if (auto& qimage = q_images_[index])
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

	return true;
}

void TripleViewsCalibrationWindow::shotImage()
{
	std::vector<cv::Mat> images(3);
	camera_->onceCapture(images);
	if(images.empty())
		return;
	
	for(size_t i=0; i<images.size(); i++)
	{
		{
			std::lock_guard lock(q_images_mutex_[0]);
			auto& q_image = q_images_[i];
			if(!q_image)
				UICommon::createQImage(images[i], q_image);
			UICommon::updateImageData(images[i], *q_image);
		}
		custom_ui_.canvases_[i]->update();
	}
}


void TripleViewsCalibrationWindow::saveImage(const std::string& filename)
{

}


void TripleViewsCalibrationWindow::abortCalibration()
{

}


void TripleViewsCalibrationWindow::shotCalibImage()
{

}


void TripleViewsCalibrationWindow::calibrate(const std::string& folder)
{

}

bool TripleViewsCalibrationWindow::eventFilter(QObject* obj, QEvent* e)
{
	for(size_t i=0; i<custom_ui_.canvases_.size(); i++)
	{
		if(obj == custom_ui_.canvases_[i])
		{
			if (e->type() == QEvent::Paint)
				paintImage(i);
			break;
		}
	}
	
	return AbstractCalibrationWindow::eventFilter(obj, e);
}
