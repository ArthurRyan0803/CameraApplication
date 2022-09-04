#include "PDRImageCamera.h"
#include "MindVision/CameraApi.h"
#include "MVDef.hpp"

using namespace CameraLib;

class Sensor: boost::noncopyable
{
private:
	CameraHandle handle_;

public:
	Sensor(CameraHandle handle, PDRImageCamera::SensorType type): handle_(handle)
	{
		switch (type)
		{
		case PDRImageCamera::Gray:
			MV_SDK_CHECK(CameraCommonCall(handle_, "set_target(CCD_ALL)", nullptr, 0));
			break;
		case PDRImageCamera::Color:
			MV_SDK_CHECK(CameraCommonCall(handle_, "set_target(CMOS)", nullptr, 0));
			break;
		case PDRImageCamera::All:
			MV_SDK_CHECK(CameraCommonCall(handle_, "set_target(SENSOR_ALL)", nullptr, 0));
			break;
		}
	}

	~Sensor()
	{
		MV_SDK_CHECK(CameraCommonCall(handle_, "set_target(SENSOR_ALL)", nullptr, 0));
	}
};



static void fillFrameHead(tSdkFrameHead& pHead, UINT MediaType, int w, int h)
{
	memset(&pHead, 0, sizeof(tSdkFrameHead));
	pHead.uiMediaType = MediaType;
	pHead.iWidth = w;
	pHead.iHeight = h;
	pHead.uBytes = w * CAMERA_MEDIA_TYPE_PIXEL_SIZE(MediaType) / 8 * h;
}


void copyFrame(CameraHandle cam_handle, BYTE* src, BYTE* gray, BYTE* rgb)
{
	tSdkFrameHead gray_head {};
	fillFrameHead(gray_head, CAMERA_MEDIA_TYPE_MONO8, MV_FRAME_WIDTH * 2, MV_FRAME_HEIGHT); // Two frames
	MV_SDK_CHECK(CameraSetIspOutFormat(cam_handle, CAMERA_MEDIA_TYPE_MONO8));
	MV_SDK_CHECK(CameraImageProcess(cam_handle, src, gray, &gray_head));
	CameraFlipFrameBuffer(gray, &gray_head, 2);

	tSdkFrameHead rgb_head {};
	auto rgb_buffer_ = MV_FRAME_WIDTH * MV_FRAME_HEIGHT * 2 + src;
	fillFrameHead(rgb_head, CAMERA_MEDIA_TYPE_BAYRG8, MV_FRAME_WIDTH, MV_FRAME_HEIGHT);
	MV_SDK_CHECK(CameraSetIspOutFormat(cam_handle, CAMERA_MEDIA_TYPE_BGR8));
	MV_SDK_CHECK(CameraImageProcess(cam_handle, rgb_buffer_, rgb, &rgb_head));
	
	CameraFlipFrameBuffer(rgb, &rgb_head, 3);
}


//void PDRImageCamera::frameCallback(CameraHandle handle, BYTE* buffer, tSdkFrameHead* frame_head, PVOID context)
//{
//	auto camera = static_cast<PDRImageCamera*>(context);
//	
//	auto gray_isp_buffer = camera->gray_isp_buffer_;
//	auto rgb_isp_buffer = camera->rgb_isp_buffer_;
//
//	copyFrame(handle, buffer, gray_isp_buffer, rgb_isp_buffer);
//	
//	std::vector<cv::Mat> out_images(3);
//	toMat<MV_FRAME_WIDTH, MV_FRAME_HEIGHT>(
//		gray_isp_buffer, rgb_isp_buffer, out_images[0], out_images[1], out_images[2]
//	);
//	
//	if(camera->frame_ready_callback_)
//		camera->frame_ready_callback_(out_images);
//}

PDRImageCamera::PDRImageCamera(const tSdkCameraDevInfo& info)
	: CoupledMVImageCamera(info)
{
}

PDRImageCamera::~PDRImageCamera() = default;

//void PDRImageCamera::open()
//{
//	//MV_SDK_CHECK(CameraInit(&info_, -1, -1, &cam_handle_));
//	MV_SDK_CHECK(CameraSetTriggerMode(cam_handle_, SOFT_TRIGGER));
//	MV_SDK_CHECK(CameraSetTriggerCount(cam_handle_, 1));
//	MV_SDK_CHECK(CameraSetAeState(cam_handle_, false));
//	MV_SDK_CHECK(CameraSetGain(cam_handle_, 100, 100, 100));
//	MV_SDK_CHECK(CameraSetOnceWB(cam_handle_));
//
//	gray_isp_buffer_ = CameraAlignMalloc(MV_FRAME_WIDTH * MV_FRAME_HEIGHT * 2, ISP_BUFFER_ALIGNMENT);
//	if(!gray_isp_buffer_)
//		throw std::runtime_error("Failed to malloc gray isp buffer!");
//	
//	rgb_isp_buffer_ = CameraAlignMalloc(MV_FRAME_WIDTH * MV_FRAME_HEIGHT * 3, ISP_BUFFER_ALIGNMENT);
//	if(!rgb_isp_buffer_)
//		throw std::runtime_error("Failed to malloc rgb isp buffer!");
//
//	MV_SDK_CHECK(CameraPlay(cam_handle_));
//
//	is_opened_ = true;
//}

uint8_t PDRImageCamera::getViews()
{
	return 3;
}

void PDRImageCamera::onceCapture(cv::OutputArray data)
{
	auto handle = getCameraHandle();

	assert(data.kind() == cv::_InputArray::STD_VECTOR_MAT);

	std::vector<cv::Mat> out_images;
	data.getMatVector(out_images);
	assert(out_images.size() == 3);

	CHECK_IS_OPENED();
	CHECK_NOT_CAPTURING();

	MV_SDK_CHECK(CameraSoftTrigger(handle));

	tSdkFrameHead frame_head;
	BYTE* buffer_;

	{
		if(SDK_UNSUCCESS(CameraGetImageBuffer(handle, &frame_head, &buffer_, 2000)))
		{
			data.release();
			return;
		}
		
		copyFrame(handle, buffer_, gray_isp_buffer_, rgb_isp_buffer_);
		MV_SDK_CHECK(CameraReleaseImageBuffer(handle, buffer_));
	}

	cv::Mat left, right, color;
	toMat<MV_FRAME_WIDTH, MV_FRAME_HEIGHT>(
		gray_isp_buffer_, rgb_isp_buffer_, left, right, color
	);

	data.assign({left.clone(), right.clone(), color.clone()});
}

void PDRImageCamera::setExposure(double value, SensorType type) const
{
	Sensor sensor(getCameraHandle(), type);
	MV_SDK_CHECK(CameraSetExposureTime(getCameraHandle(), value));
}

void PDRImageCamera::getExposure(double& value, SensorType type) const
{
	Sensor sensor(getCameraHandle(), type);
	MV_SDK_CHECK(CameraGetExposureTime(getCameraHandle(), &value));
}

void PDRImageCamera::getExposureRange(double& min, double& max, double& step, SensorType type) const
{
	Sensor sensor(getCameraHandle(), type);
	MV_SDK_CHECK(CameraGetExposureTimeRange(getCameraHandle(), &min, &max, &step));
}


void PDRImageCamera::setAnalogGain(int value, SensorType type) const
{
	Sensor sensor(getCameraHandle(), type);
	MV_SDK_CHECK(CameraSetAnalogGain(getCameraHandle(), value));
}


void PDRImageCamera::getAnalogGain(int& value, SensorType type) const
{
	Sensor sensor(getCameraHandle(), type);
	MV_SDK_CHECK(CameraGetAnalogGain(getCameraHandle(), &value));
}


void PDRImageCamera::getAnalogGainRange(int& min, int& max, int& step, SensorType type) const
{
	min = 3;
	max = 10;
	step = 1;
}

void PDRImageCamera::turnOnFillLight() const
{
	MV_SDK_CHECK(CameraSetIOState(getCameraHandle(), 1, 1));
}

void PDRImageCamera::turnOffFillLight() const
{
	MV_SDK_CHECK(CameraSetIOState(getCameraHandle(), 1, 0));
}
