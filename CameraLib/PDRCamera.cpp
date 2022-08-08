#include "PDRBuffer.h"
#include "PDRCamera.h"
#include "MindVision/CameraApi.h"
#include "CameraGrabber.h"
#include "VSensorDef.h"

using namespace CameraLib;


#define SELECT_RGB_SENSOR(HANDLE) VSENSOR_SDK_TRACK(CameraCommonCall((HANDLE), "set_target(CCD_ALL)", nullptr, 0))
#define SELECT_GRAY_SENSOR(HANDLE) VSENSOR_SDK_TRACK(CameraCommonCall((HANDLE), "set_target(CMOS)", nullptr, 0))


#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 1024


static void copyRawData(BYTE* dst, BYTE* src, int w, int h, int step)
{
	for (int i = 0; i < h; ++i)
	{
		memcpy(dst, src, w);
		dst += w;
		src += step;
	}
}


static void fillFrameHead(tSdkFrameHead& pHead, UINT MediaType, int w, int h)
{
	memset(&pHead, 0, sizeof(tSdkFrameHead));
	pHead.uiMediaType = MediaType;
	pHead.iWidth = w;
	pHead.iHeight = h;
	pHead.uBytes = w * CAMERA_MEDIA_TYPE_PIXEL_SIZE(MediaType) / 8 * h;
}


int PDRCamera::vsensor_frame_ready_callback(void* grabber, int phase, BYTE* frame_buffer, tSdkFrameHead* frame_head, void* context)
{
	auto camera = static_cast<PDRCamera*>(context);

	if(camera->is_snap_)
	{
		// get data
		camera->snap_frame_head_ = frame_head;
		camera->snap_src_buffer_ = frame_buffer;

		// Notify thread of oneShot()
		camera->snap_condition_var_.notify_one();

		// Waiting thread of oneShot() completing copy data.
		std::unique_lock lock(camera->snap_mutex_);

		camera->snap_condition_var_.wait(lock);
		
		camera->is_snap_ = false;
	}
	else if (camera->isCapturing())
	{
		auto& callback = camera->frame_ready_callback_;
		if(!callback)
			return 0;

		auto& isp_buffer = camera->isp_buffer_;
		auto& camera_handle = camera->cam_handle_;
		
		// parse and copy gray data
		copyRawData(isp_buffer->left.get(), frame_buffer, FRAME_WIDTH, FRAME_HEIGHT, FRAME_WIDTH * 2);
		copyRawData(isp_buffer->right.get(), frame_buffer + FRAME_WIDTH, FRAME_WIDTH, FRAME_HEIGHT, FRAME_WIDTH * 2);
		
		tSdkFrameHead rgb_head;
		// directly process rgb data in camera core buffer.
		CameraSetIspOutFormat(camera_handle, CAMERA_MEDIA_TYPE_BGR8);
		fillFrameHead(rgb_head, CAMERA_MEDIA_TYPE_BAYRG8, FRAME_WIDTH, FRAME_HEIGHT);
		CameraImageProcess(camera_handle, FRAME_WIDTH * FRAME_HEIGHT * 2 + frame_buffer, isp_buffer->rgb_isp.get(), &rgb_head);

		// process gray data
		tSdkFrameHead gray_head;
		CameraSetIspOutFormat(camera_handle, CAMERA_MEDIA_TYPE_MONO8);
		fillFrameHead(gray_head, CAMERA_MEDIA_TYPE_MONO8, FRAME_WIDTH, FRAME_HEIGHT);
		
		CameraImageProcess(camera_handle, isp_buffer->left.get(), isp_buffer->left_isp.get(), &gray_head);
		CameraImageProcess(camera_handle, isp_buffer->right.get(), isp_buffer->right_isp.get(), &gray_head);

		auto left_mat = cv::Mat(cv::Size(gray_head.iWidth, gray_head.iHeight), CV_8UC1, isp_buffer->left_isp.get());
		auto right_mat = cv::Mat(cv::Size(gray_head.iWidth, gray_head.iHeight), CV_8UC1, isp_buffer->right_isp.get());
		auto rgb_mat = cv::Mat(cv::Size(gray_head.iWidth, gray_head.iHeight), CV_8UC3, isp_buffer->rgb_isp.get());

		callback(std::vector {left_mat, right_mat, rgb_mat});
	}

	return 0;
}

PDRCamera::PDRCamera(const tSdkCameraDevInfo& info)
	:Camera(), isp_buffer_(std::make_unique<PDRBuffer>(cv::Size(FRAME_WIDTH, FRAME_HEIGHT))), info_(info)
{
}

PDRCamera::~PDRCamera()
{
	close();
}

void PDRCamera::open()
{
	VSENSOR_SDK_TRACK(CameraGrabber_Create(&grabber_handle_, &info_));
	VSENSOR_SDK_TRACK(CameraGrabber_GetCameraHandle(grabber_handle_, &cam_handle_));
	VSENSOR_SDK_TRACK(CameraCreateSettingPage(cam_handle_, nullptr, info_.acFriendlyName, nullptr, nullptr, 0));
	VSENSOR_SDK_TRACK(CameraGrabber_SetFrameListener(grabber_handle_, vsensor_frame_ready_callback, this));

	tSdkCameraCapbility cap{};

	/* --------- gray sensor -------*/
	// Set rgb sensor properties
	SELECT_GRAY_SENSOR(cam_handle_);
	VSENSOR_SDK_TRACK(CameraGetCapability(cam_handle_, &cap));
	if (cap.sIspCapacity.bMonoSensor)
		CameraSetIspOutFormat(cam_handle_, CAMERA_MEDIA_TYPE_MONO8);

	// Maximize roi
	tSdkImageResolution roi{};
	roi.iIndex = 0xff;
	roi.iWidth = roi.iWidthFOV = FRAME_WIDTH;
	roi.iHeight = roi.iHeightFOV = FRAME_HEIGHT * 3;
	VSENSOR_SDK_TRACK(CameraSetImageResolution(cam_handle_, &roi));
	VSENSOR_SDK_TRACK(CameraSetAeState(cam_handle_, FALSE));				// Manual exposure.
	VSENSOR_SDK_TRACK(CameraSetTriggerMode(cam_handle_, SOFT_TRIGGER));	// Soft trigger
	
	/* --------- rgb sensor -------*/
	SELECT_RGB_SENSOR(cam_handle_);
	VSENSOR_SDK_TRACK(CameraSetAeState(cam_handle_, FALSE));				// Manual exposure.
	VSENSOR_SDK_TRACK(CameraSetTriggerMode(cam_handle_, SOFT_TRIGGER));	// Soft trigger

	VSENSOR_SDK_TRACK(CameraGrabber_StartLive(grabber_handle_));

	is_opened_ = true;
}


void PDRCamera::close()
{
	if(isOpened())
	{
		VSENSOR_SDK_TRACK(CameraGrabber_StopLive(grabber_handle_));
		VSENSOR_SDK_TRACK(CameraGrabber_Destroy(grabber_handle_));
		is_opened_ = false;
		grabber_handle_ = nullptr;
		cam_handle_ = 0;
	}
}

bool PDRCamera::isOpened()
{
	return is_opened_;
}

void PDRCamera::startCapture()
{
	CHECK_IS_OPENED();
	CHECK_NOT_CAPTURING();
	
	VSENSOR_SDK_TRACK(CameraSetTriggerMode(cam_handle_, CONTINUATION));
	is_capturing_ = true;
}

void PDRCamera::stopCapture()
{
	CHECK_IS_OPENED();

	if (!isCapturing())
		return;
	
	VSENSOR_SDK_TRACK(CameraSetTriggerMode(cam_handle_, SOFT_TRIGGER));
	is_capturing_ = false;
	
}

bool PDRCamera::isCapturing()
{
	return is_capturing_;
}

size_t PDRCamera::getViews()
{
	return 3;
}

void PDRCamera::showParameterDialog()
{
}

void PDRCamera::oneShot(cv::OutputArray images)
{
	assert(images.kind() == cv::_InputArray::STD_VECTOR_MAT);

	CHECK_IS_OPENED();
	CHECK_NOT_CAPTURING();

	VSENSOR_SDK_TRACK(CameraSoftTrigger(cam_handle_));

	assert(!is_snap_ && "Why is_snap_ is true?");

	is_snap_ = true;

	{
		// waiting frame call back function
		std::unique_lock lock(snap_mutex_);
		if(snap_condition_var_.wait_for(lock, std::chrono::milliseconds(1000)) == std::cv_status::timeout)
			throw std::runtime_error("Snap timeout!");

		// parse and copy gray data
		copyRawData(isp_buffer_->left.get(), snap_src_buffer_, FRAME_WIDTH, FRAME_HEIGHT, FRAME_WIDTH * 2);
		copyRawData(isp_buffer_->right.get(), snap_src_buffer_ + FRAME_WIDTH, FRAME_WIDTH, FRAME_HEIGHT, FRAME_WIDTH * 2);
		
		tSdkFrameHead rgb_head;
		// directly process rgb data in camera core buffer.
		CameraSetIspOutFormat(cam_handle_, CAMERA_MEDIA_TYPE_BGR8);
		fillFrameHead(rgb_head, CAMERA_MEDIA_TYPE_BAYRG8, FRAME_WIDTH, FRAME_HEIGHT);
		CameraImageProcess(cam_handle_, FRAME_WIDTH * FRAME_HEIGHT * 2 + snap_src_buffer_, isp_buffer_->rgb_isp.get(), &rgb_head);
	}

	// notify callback function to continue
	snap_condition_var_.notify_one();

	// process gray data
	tSdkFrameHead gray_head;
	CameraSetIspOutFormat(cam_handle_, CAMERA_MEDIA_TYPE_MONO8);
	fillFrameHead(gray_head, CAMERA_MEDIA_TYPE_MONO8, FRAME_WIDTH, FRAME_HEIGHT);
	
	CameraImageProcess(cam_handle_, isp_buffer_->left.get(), isp_buffer_->left_isp.get(), &gray_head);
	CameraImageProcess(cam_handle_, isp_buffer_->right.get(), isp_buffer_->right_isp.get(), &gray_head);

	auto left_mat = cv::Mat(cv::Size(gray_head.iWidth, gray_head.iHeight), CV_8UC1, isp_buffer_->left_isp.get()).clone();
	auto right_mat = cv::Mat(cv::Size(gray_head.iWidth, gray_head.iHeight), CV_8UC1, isp_buffer_->right_isp.get()).clone();
	auto rgb_mat = cv::Mat(cv::Size(gray_head.iWidth, gray_head.iHeight), CV_8UC3, isp_buffer_->rgb_isp.get()).clone();
	
	images.assign(std::vector{left_mat, right_mat, rgb_mat});
}

void PDRCamera::setFrameReadyCallback(std::function<void(cv::InputArray)> callback)
{
	frame_ready_callback_ = callback;
}
