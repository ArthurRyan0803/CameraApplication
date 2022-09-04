#include "MVImageCamera.h"
#include "MVDef.hpp"


using namespace CameraLib;


CameraHandle MVImageCamera::getCameraHandle() const
{
	return cam_handle_;
}

void MVImageCamera::registerFrameListener(std::weak_ptr<FrameListener> listener)
{
	std::lock_guard lock(listeners_mutex_);
	frame_listeners_.push_back(listener);
}

uint8_t MVImageCamera::getViews()
{
	return 1;
}

const tSdkCameraCapbility& MVImageCamera::getCameraCapability() const
{
	return capability_;
}

std::shared_ptr<BYTE> MVImageCamera::getIspBuffer() const
{
	return isp_buffer_;
}

void MVImageCamera::frameCallback(CameraHandle camera_handle, BYTE *frame_buffer, tSdkFrameHead* frame_head, PVOID context)
{
	if(!context)
		return;

	auto camera = static_cast<MVImageCamera*>(context);
	camera->notifyFrameListeners(frame_buffer, *frame_head);
}

MVImageCamera::MVImageCamera(const tSdkCameraDevInfo& info):
	is_opened_(false), camera_info_(info), cam_handle_(0), capability_({})
{
	static bool has_init_ = false;

	if (has_init_) return;

	MV_SDK_CHECK(CameraSetSysOption("NumBuffers", "8"));
	MV_SDK_CHECK(CameraSdkInit(0));
	has_init_ = true;
}

MVImageCamera::~MVImageCamera()
{
	this->close();
}

void MVImageCamera::open()
{
	MV_SDK_CHECK(CameraInit(&camera_info_, -1, -1, &cam_handle_));
	MV_SDK_CHECK(CameraGetCapability(cam_handle_, &capability_));
	MV_SDK_CHECK(CameraSetIspOutFormat(cam_handle_, capability_.sIspCapacity.bMonoSensor ? CAMERA_MEDIA_TYPE_MONO8: CAMERA_MEDIA_TYPE_BGR8));

	MV_SDK_CHECK(CameraSetTriggerMode(cam_handle_, SOFT_TRIGGER));
	MV_SDK_CHECK(CameraPlay(cam_handle_));
	
	is_opened_ = true;

	try
	{
		allocateBuffer(isp_buffer_, isp_buffer_size_);
	}
	catch(const std::exception& e)
	{
		is_opened_ = false;
		throw e;
	}
}

void MVImageCamera::allocateBuffer(std::shared_ptr<BYTE>& buffer, int& size)
{
	auto element_size = capability_.sIspCapacity.bMonoSensor ? 1: 3;
	auto new_size = capability_.sResolutionRange.iWidthMax * capability_.sResolutionRange.iHeightMax * element_size;

	if(!buffer || size != new_size)
	{
		auto isp_buffer = CameraAlignMalloc(new_size, ISP_BUFFER_ALIGNMENT);
		assert(isp_buffer && "CameraAlignMalloc failed");
		buffer = std::shared_ptr<BYTE>(isp_buffer, CameraAlignFree);
		size = new_size;
	}
}

void MVImageCamera::wrapIspBuffer(
	BYTE* frame_buffer, tSdkFrameHead& frame_head, const std::shared_ptr<BYTE>& buffer, cv::OutputArray data
)
{
	assert(data.kind() == cv::_InputArray::MAT);
	
	MV_SDK_CHECK(CameraImageProcess(cam_handle_, frame_buffer, buffer.get(), &frame_head));
	MV_SDK_CHECK(CameraFlipFrameBuffer(isp_buffer_.get(), &frame_head, IMAGE_VERTICAL_FLIP_FLAG));

	// It's just wrapper of isp_buffer_
	auto image = cv::Mat(
		cv::Size(frame_head.iWidth, frame_head.iHeight), 
		frame_head.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
		isp_buffer_.get()
	);

	data.assign(image);
}

void MVImageCamera::createOutputContainer(cv::_OutputArray& data)
{
	data = cv::Mat();
}

void MVImageCamera::notifyFrameListeners(BYTE* frame_buffer, tSdkFrameHead& frame_head)
{
#ifdef _DEBUG
	callback_frames += 1;
#endif

	std::lock_guard lock(listeners_mutex_);

	if(!frame_listeners_.empty())
	{
		cv::_OutputArray data;

		if(getViews() == 1)
			data = cv::Mat();
		else
		{
			data = std::vector<cv::Mat>();		// Just assign type without capacity.
			auto vector = static_cast<std::vector<cv::Mat>*>(data.getObj());
			vector->resize(getViews());
		}

		wrapIspBuffer(frame_buffer, frame_head, isp_buffer_, data);

		auto iter = frame_listeners_.begin();
		while(iter != frame_listeners_.end())
		{
			auto shared_ptr = iter->lock();
			if(shared_ptr)
			{
				shared_ptr->frameReadyCallback(data);
				++iter;
			}
			else
			{
				iter = frame_listeners_.erase(iter);
			}
		}
	}
}

void MVImageCamera::close()
{
	if(!is_opened_)
		return;
	
	MV_SDK_CHECK(CameraUnInit(cam_handle_));

	is_opened_ = false;
	cam_handle_ = 0;
}

bool MVImageCamera::isOpened()
{
	return is_opened_;
}

void MVImageCamera::startContinuesCapture()
{
	CHECK_IS_OPENED();
	CHECK_NOT_CAPTURING();
	
	MV_SDK_CHECK(CameraSetCallbackFunction(cam_handle_, frameCallback, this, nullptr));
	MV_SDK_CHECK(CameraSetTriggerMode(cam_handle_, CONTINUATION));
}

void MVImageCamera::stopContinuesCapture()
{
	CHECK_IS_OPENED();

	if(!isCapturing())
		return;
	
	MV_SDK_CHECK(CameraSetCallbackFunction(cam_handle_, nullptr, nullptr, nullptr));
	MV_SDK_CHECK(CameraSetTriggerMode(cam_handle_, SOFT_TRIGGER));
}

bool MVImageCamera::isCapturing()
{
	INT mode;
	MV_SDK_CHECK(CameraGetTriggerMode(cam_handle_, &mode));
	return mode == CONTINUATION;
}

void MVImageCamera::onceCapture(cv::OutputArray image)
{
#ifdef _DEBUG
	INT mode;
	MV_SDK_CHECK(CameraGetTriggerMode(cam_handle_, &mode));
	assert(mode == SOFT_TRIGGER);
#endif

	assert(image.kind() == cv::_InputArray::MAT);

	CHECK_IS_OPENED();
	CHECK_NOT_CAPTURING();

	MV_SDK_CHECK(CameraSoftTrigger(cam_handle_));
	
	tSdkFrameHead frame_head;
	BYTE* p_buffer = nullptr;
	cv::Mat isp_mat;
	auto& out_mat = image.getMatRef();

	MV_SDK_CHECK(CameraGetImageBuffer(cam_handle_, &frame_head, &p_buffer, ONE_SHOT_WAIT_MS));

	wrapIspBuffer(p_buffer, frame_head, isp_buffer_, isp_mat);
	
	isp_mat.copyTo(out_mat);

	MV_SDK_CHECK(CameraReleaseImageBuffer(cam_handle_, p_buffer));
}

void MVImageCamera::enableHardwareTrigger()
{
	int mode;
	MV_SDK_CHECK(CameraGetTriggerMode(cam_handle_, &mode));
	if(mode == EXTERNAL_TRIGGER)
		return;
	if(mode == CONTINUATION)
		throw std::logic_error("Please close continues capture first!");

	MV_SDK_CHECK(CameraSetTriggerMode(cam_handle_, EXTERNAL_TRIGGER));		
	MV_SDK_CHECK(CameraSetCallbackFunction(cam_handle_, frameCallback, this, nullptr));
	//MV_SDK_CHECK(CameraSetExtTrigSignalType(cam_handle_, EXT_TRIG_LEADING_EDGE));
}

void MVImageCamera::disableHardwareTrigger()
{
	int mode;
	MV_SDK_CHECK(CameraGetTriggerMode(cam_handle_, &mode));
	if(mode != EXTERNAL_TRIGGER)
		return;

	MV_SDK_CHECK(CameraSetTriggerMode(cam_handle_, SOFT_TRIGGER));
	MV_SDK_CHECK(CameraSetCallbackFunction(cam_handle_, frameCallback, nullptr, nullptr));
}

void MVImageCamera::setExposure(double us)
{
	MV_SDK_CHECK(CameraSetExposureTime(cam_handle_, us));
}

void MVImageCamera::getExposure(double& us) const
{
	MV_SDK_CHECK(CameraGetExposureTime(cam_handle_, &us));
}

void MVImageCamera::getExposureRange(double& min, double& max, double& step) const
{
	MV_SDK_CHECK(CameraGetExposureTimeRange(cam_handle_, &min, &max, &step));
}

void MVImageCamera::setGain(int value)
{
	MV_SDK_CHECK(CameraSetAnalogGain(cam_handle_, value));
}

void MVImageCamera::getGain(int& value) const
{
	MV_SDK_CHECK(CameraGetAnalogGain(cam_handle_, &value));
}

void MVImageCamera::getGainRange(int& min, int& max, int& step) const
{
	min = 0;
	max = 10;
	step = 1;
}

void MVImageCamera::whiteBalance() const
{
	MV_SDK_CHECK(CameraSetOnceWB(cam_handle_));
}
