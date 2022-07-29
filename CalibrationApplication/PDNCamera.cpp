#include "PDNCamera.h"
#include "Logger.hpp"

#include <stdexcept>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <thread>
#include <opencv2/opencv.hpp>
#include <array>


#define CAMERA_SDK_TRACK(OP) if(auto CODE = OP; SDK_UNSUCCESS(CODE)) throw PDNCameraException(#OP, CODE, __FILE__, __LINE__)

#define ONE_SHOT_WAIT_MS 1000
#define ISP_BUFFER_ALIGNMENT 16


void PDNCamera::frameCallback(CameraHandle camera_handle, BYTE *frame_buffer, tSdkFrameHead* frame_head, PVOID context)
{
	if(!context)
		return;

	auto camera = static_cast<PDNCamera*>(context);
	if(camera->frame_ready_callback_)
	{
		
		CAMERA_SDK_TRACK(CameraImageProcess(camera_handle, frame_buffer, camera->isp_buffer_, frame_head));
		CAMERA_SDK_TRACK(CameraFlipFrameBuffer(camera->isp_buffer_, frame_head, 3));
	
		cv::Mat mat = cv::Mat(cv::Size(frame_head->iWidth, frame_head->iHeight), frame_head->uBytes, camera->isp_buffer_);

		if(camera->sensor_mode_ == Both)
			camera->frame_ready_callback_(mat);
		else if (camera->sensor_mode_ == Left)
			camera->frame_ready_callback_(
				mat
				.rowRange(0, frame_head->iHeight)
				.colRange(0, frame_head->iWidth / 2)
			);
		else if (camera->sensor_mode_ == Right)
			camera->frame_ready_callback_(
				mat
				.rowRange(0, frame_head->iHeight)
				.colRange(frame_head->iWidth / 2,  frame_head->iWidth)
			);
	}
}


PDNCamera::PDNCamera(const tSdkCameraDevInfo& info, SensorMode mode): Camera(),
sensor_mode_(mode), camera_info_(info), cam_handle_(0), is_opened_(false), isp_buffer_(nullptr)
{
	std::memset(&capability_, 0, sizeof(capability_));
}


PDNCamera::~PDNCamera()
{
	this->close();
	GET_LOGGER().debug("PDN Camera release!");
}


void PDNCamera::open()
{
	CAMERA_SDK_TRACK(CameraInit(&camera_info_, -1, -1, &cam_handle_));
	CAMERA_SDK_TRACK(CameraGetCapability(cam_handle_, &capability_));
	
    auto isp_buffer_size = capability_.sResolutionRange.iWidthMax * capability_.sResolutionRange.iHeightMax * 3;
    isp_buffer_ = CameraAlignMalloc(isp_buffer_size, ISP_BUFFER_ALIGNMENT);
	assert(isp_buffer_ && "Why allocated isp_buffer is null?");

	if(capability_.sIspCapacity.bMonoSensor)
	{
		CAMERA_SDK_TRACK(CameraSetIspOutFormat(cam_handle_, CAMERA_MEDIA_TYPE_MONO8));
	}

	char camera_name[64];
	strcpy_s(camera_name,camera_info_.acFriendlyName);
	CAMERA_SDK_TRACK(CameraCreateSettingPage(cam_handle_, nullptr, camera_name,nullptr,nullptr,0));
	CAMERA_SDK_TRACK(CameraSetTriggerMode(cam_handle_, SOFT_TRIGGER));
	CAMERA_SDK_TRACK(CameraPlay(cam_handle_));

	is_opened_ = true;
}


void PDNCamera::close()
{
	if(!is_opened_)
		return;

	//CAMERA_SDK_TRACK(CameraPause(cam_handle_));
	CAMERA_SDK_TRACK(CameraUnInit(cam_handle_));

	CameraAlignFree(isp_buffer_);
	is_opened_ = false;
	cam_handle_ = 0;
	isp_buffer_ = nullptr;
}


bool PDNCamera::isOpened()
{
	return is_opened_;
}


void PDNCamera::startCapture()
{
	CHECK_IS_OPENED();
	CHECK_NOT_CAPTURING();
	
	CAMERA_SDK_TRACK(CameraSetTriggerMode(cam_handle_, CONTINUATION));
	CAMERA_SDK_TRACK(CameraSetCallbackFunction(cam_handle_, frameCallback, this, nullptr));
}


void PDNCamera::stopCapture()
{
	CHECK_IS_OPENED();

	if(!isCapturing())
		return;
	
	CAMERA_SDK_TRACK(CameraSetTriggerMode(cam_handle_, SOFT_TRIGGER));
	CAMERA_SDK_TRACK(CameraSetCallbackFunction(cam_handle_, nullptr, nullptr, nullptr));
}


bool PDNCamera::isCapturing()
{
	INT mode;
	CAMERA_SDK_TRACK(CameraGetTriggerMode(cam_handle_, &mode));
	return mode == CONTINUATION;
}


std::vector<std::array<int, 2>> PDNCamera::enumerateAvailableResolutions()
{
	CHECK_IS_OPENED();

	auto resolution = getCurrentResolution();
	return { {resolution[0], resolution[1]} };
}


// This method get the virtual resolution
std::array<int, 2> PDNCamera::getCurrentResolution()
{
	CHECK_IS_OPENED();

	tSdkImageResolution resolution;
	CAMERA_SDK_TRACK(CameraGetImageResolution(cam_handle_, &resolution));
	
	return std::array<int, 2> {sensor_mode_ == Both ? resolution.iWidth * 2: resolution.iWidth, resolution.iHeight};
}


//void PDNCamera::setCurrentResolution(const std::array<int, 2>& resolution)
//{
//	CHECK_IS_OPENED();
//
//	tSdkImageResolution resolution_s;
//	CAMERA_SDK_TRACK(CameraGetImageResolution(cam_handle_, &resolution_s));
//
//	resolution_s.iWidth = useBothSensor() ? resolution[0] / 2 : resolution[0];
//	resolution_s.iHeight = resolution[1];
//	
//	CAMERA_SDK_TRACK(CameraSetImageResolution(cam_handle_, &resolution_s));
//}


size_t PDNCamera::getPixelType()
{
	CHECK_IS_OPENED();

	UINT format;
	CAMERA_SDK_TRACK(CameraGetIspOutFormat(cam_handle_, &format));
	switch(format)
	{
	case CAMERA_MEDIA_TYPE_MONO8: 
		return CV_8U;
	case CAMERA_MEDIA_TYPE_RGB8:
	case CAMERA_MEDIA_TYPE_BGR8:
		return CV_8UC3;
	default:
		throw std::runtime_error("Unrecognized format of isp!");
	}
}


void PDNCamera::oneShot(cv::OutputArray image)
{
	assert(image.kind() == cv::_InputArray::MAT);

	CHECK_IS_OPENED();
	CHECK_NOT_CAPTURING();

	CAMERA_SDK_TRACK(CameraSoftTrigger(cam_handle_));
	
	tSdkFrameHead frame_head;
	BYTE* p_buffer = nullptr;
	CAMERA_SDK_TRACK(CameraGetImageBuffer(cam_handle_, &frame_head, &p_buffer, ONE_SHOT_WAIT_MS));
	CAMERA_SDK_TRACK(CameraImageProcess(cam_handle_, p_buffer, isp_buffer_, &frame_head));
	CAMERA_SDK_TRACK(CameraFlipFrameBuffer(isp_buffer_, &frame_head, 3));
	
	auto mat = cv::Mat(
		cv::Size(frame_head.iWidth, frame_head.iHeight), 
		frame_head.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
		isp_buffer_
	);
	
	if(sensor_mode_ == Left)
		image.getMatRef() = mat.rowRange(0, frame_head.iHeight).colRange(0, frame_head.iWidth / 2).clone();
	else if(sensor_mode_ == Right)
		image.getMatRef() = mat.rowRange(0, frame_head.iHeight).colRange(frame_head.iWidth / 2, frame_head.iWidth).clone();
	else
		image.getMatRef() = mat;
	
	CAMERA_SDK_TRACK(CameraReleaseImageBuffer(cam_handle_, p_buffer));
}

void PDNCamera::showParameterDialog()
{
	CameraShowSettingPage(cam_handle_, true);
}

void PDNCamera::setFrameReadyCallback(std::function<void(cv::InputArray)> callback)
{
	frame_ready_callback_ = callback;
}
