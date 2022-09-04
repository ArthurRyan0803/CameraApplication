#include "VSensorPointCloudCamera.h"
#include "VSensor/VSensor.h"
#include "VSensorDef.h"

using namespace CameraLib;


VSensorPointCloudCamera::VSensorPointCloudCamera(
	const std::shared_ptr<VSENSOR::VSensor>& api, int index, std::string ip
)
	:index_(index), vsensor_api_(api), ip_(ip), capture_mode_(ImageCapture)
{
}

void VSensorPointCloudCamera::open()
{
	open(false);
}

void VSensorPointCloudCamera::open(bool initialize)
{
	VSENSOR_SDK_CHECK(vsensor_api_->DeviceConnect(index_));

	if(initialize)
		VSENSOR_SDK_CHECK(vsensor_api_->DeviceParameterInit());

	VSENSOR_SDK_CHECK(vsensor_api_->SetCaptureMode(capture_mode_));

	if(!gray_images_buffers1_)
		gray_images_buffers1_ = std::make_unique<std::array<std::array<BYTE, IMAGE_HEIGHT * IMAGE_WIDTH>, IMAGES_NUM>>();

	if(!gray_images_buffers2_)
		gray_images_buffers2_ = std::make_unique<std::array<std::array<BYTE, IMAGE_HEIGHT * IMAGE_WIDTH>, IMAGES_NUM>>();
	
	if(!rgb_image_buffer_)
		rgb_image_buffer_ = std::make_unique<std::array<BYTE, IMAGE_WIDTH * IMAGE_HEIGHT * 3>>();

	is_opened_ = true;
}

void VSensorPointCloudCamera::close()
{
	VSENSOR_SDK_CHECK(vsensor_api_->DeviceUnInit());
	is_opened_ = false;
}

bool VSensorPointCloudCamera::isOpened()
{
	return is_opened_;
}

void VSensorPointCloudCamera::startContinuesCapture()
{
	throw std::logic_error("Not implemented!");
}

void VSensorPointCloudCamera::stopContinuesCapture()
{	
	throw std::logic_error("Not implemented!");
}

bool VSensorPointCloudCamera::isCapturing()
{
	return false;
}

void VSensorPointCloudCamera::onceCapture(cv::OutputArray data)
{
	if(capture_mode_ != ImageCapture)
		throw std::logic_error("Please set capture mode to ImageCapture!");

	if(data.empty())
		return;

	assert(data.kind() == cv::_InputArray::STD_VECTOR_MAT);

	std::vector<cv::Mat> images;
	data.getMatVector(images);
	assert(images.size() == 3);
	
	cv::Mat gray_new(IMAGE_HEIGHT, IMAGE_WIDTH * 2, CV_8UC1);

	cv::Mat& color_mat = images[2];
	if(color_mat.empty() || color_mat.rows != IMAGE_HEIGHT || color_mat.cols != IMAGE_WIDTH || color_mat.channels() != 3)
	{
		color_mat.release();
		color_mat = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
	}
		
	VSENSOR_SDK_CHECK(vsensor_api_->CameraOut(gray_new.data, color_mat.data));
	images[0] = gray_new.rowRange(0, IMAGE_HEIGHT).colRange(0, IMAGE_WIDTH).clone();
	images[1] = gray_new.rowRange(0, IMAGE_HEIGHT).colRange(IMAGE_WIDTH, IMAGE_WIDTH * 2).clone();
	
	images[2] = color_mat;

	data.assign(images);
}

void VSensorPointCloudCamera::setExposure(double value, SensorType type) const
{
	VSENSOR_SDK_CHECK(vsensor_api_->SetExposureTime(value, type));
}

void VSensorPointCloudCamera::getExposure(double& value, SensorType type) const
{
	VSENSOR_SDK_CHECK(vsensor_api_->GetExposureTime(value, type));
}

void VSensorPointCloudCamera::setExposure(double us)
{
}

void CameraLib::VSensorPointCloudCamera::getExposure(double& us) const
{
}

void CameraLib::VSensorPointCloudCamera::getExposureRange(double& min, double& max, double& step) const
{
}

void CameraLib::VSensorPointCloudCamera::getGainRange(int& min, int& max, int& step) const
{
}

void CameraLib::VSensorPointCloudCamera::setGain(int value)
{
}

void CameraLib::VSensorPointCloudCamera::getGain(int& value) const
{
}

void VSensorPointCloudCamera::registerFrameListener(std::weak_ptr<FrameListener> listener)
{

}

void VSensorPointCloudCamera::setAnalogGain(const int& r, const int& g, const int& b) const
{
	VSENSOR_SDK_CHECK(vsensor_api_->SetAnalogGain(r, g, b));
}


void VSensorPointCloudCamera::setAnalogGain(const int& value, SensorType type) const
{
	VSENSOR_SDK_CHECK(vsensor_api_->SetAnalogGain(value, type));
}

void VSensorPointCloudCamera::getAnalogGain(int& r, int& g, int& b) const
{
	VSENSOR_SDK_CHECK(vsensor_api_->GetAnalogGain(r, g, b));
}

void VSensorPointCloudCamera::getAnalogGain(int& value, SensorType type) const
{
	// TODO
	//VSENSOR_SDK_CHECK(vsensor_api_->GetAnalogGain(value, 0));
}

void VSensorPointCloudCamera::setZAxisRange(int min, int max) const
{
	VSENSOR_SDK_CHECK(vsensor_api_->SetZaxisRange(min, max));
}

//void VSensorPointCloudCamera::whiteBalance()
//{
//	VSENSOR_SDK_CHECK(vsensor_api_->SetCameraOnceWB());
//}

void VSensorPointCloudCamera::setCaptureMode(CaptureMode mode)
{
	VSENSOR_SDK_CHECK(vsensor_api_->SetCaptureMode(mode));
	capture_mode_ = mode;
}

void VSensorPointCloudCamera::setLight(int open) const
{
	VSENSOR_SDK_CHECK(vsensor_api_->SetLight(open));
}

void VSensorPointCloudCamera::captureStructureLightPatternImages() const
{
	if(capture_mode_ != PointCloudCapture)
		throw std::logic_error("Please set capture mode to PointCloudCapture!");

	auto gray_buffer1 = std::array<BYTE*, IMAGES_NUM>(), gray_buffer2 = std::array<BYTE*, IMAGES_NUM>();
	for(size_t i = 0; i < IMAGES_NUM; i++)
	{
		gray_buffer1[i] = gray_images_buffers1_->at(i).data();
		gray_buffer2[i] = gray_images_buffers2_->at(i).data();
	}
	
	VSENSOR_SDK_CHECK(
		vsensor_api_->SingleCapture(
			gray_buffer1.data(), gray_buffer2.data(), rgb_image_buffer_.get()->data()
		)
	);
}

void VSensorPointCloudCamera::constructPointCloud(std::unique_ptr<VSensorResult>& rst) const
{
	if(!rst)
		rst = std::make_unique<VSensorResult>();

	auto gray_buffer1 = std::array<BYTE*, IMAGES_NUM>(), gray_buffer2 = std::array<BYTE*, IMAGES_NUM>();
	for(size_t i = 0; i < IMAGES_NUM; i++)
	{
		gray_buffer1[i] = gray_images_buffers1_->at(i).data();
		gray_buffer2[i] = gray_images_buffers2_->at(i).data();
	}

	VSENSOR_SDK_CHECK(
		vsensor_api_->SingleRestruction( 
			capture_mode_, rst.get(), gray_buffer1.data(), gray_buffer2.data(), rgb_image_buffer_.get()->data()
		)
	);

	memcpy(rst->RGBMap, rgb_image_buffer_.get(), rgb_image_buffer_->size());
}

std::string VSensorPointCloudCamera::getIP() const
{
	return ip_;
}
