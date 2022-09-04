#include "Framework.h"
#include "PDNImageCamera.h"
#include "MVDef.hpp"


using namespace CameraLib;

PDNImageCamera::PDNImageCamera(const tSdkCameraDevInfo& info): CoupledMVImageCamera(info)
{
}

uint8_t PDNImageCamera::getViews()
{
	return 2;
}

void PDNImageCamera::allocateBuffer(std::shared_ptr<BYTE>& buffer, int& size)
{
	auto capability = getCameraCapability();
	auto new_size = capability.sResolutionRange.iWidthMax * capability.sResolutionRange.iHeightMax * 2;

	if(!buffer || size != new_size)
	{
		auto isp_buffer = CameraAlignMalloc(new_size, ISP_BUFFER_ALIGNMENT);
		assert(isp_buffer && "CameraAlignMalloc failed");
		buffer = std::shared_ptr<BYTE>(isp_buffer, CameraAlignFree);
		size = new_size;
	}
}

void PDNImageCamera::wrapIspBuffer(BYTE* frame_buffer, tSdkFrameHead& frame_head, const std::shared_ptr<BYTE>& buffer, cv::OutputArray data)
{
	assert(data.kind() == cv::_InputArray::STD_VECTOR_MAT);

	std::vector<cv::Mat> mats;
	data.getMatVector(mats);

	assert(mats.size() == 2);

	MV_SDK_CHECK(CameraImageProcess(getCameraHandle(), frame_buffer, buffer.get(), &frame_head));
	MV_SDK_CHECK(CameraFlipFrameBuffer(buffer.get(), &frame_head, IMAGE_VERTICAL_FLIP_FLAG));

	// It's just wrapper of isp_buffer_
	auto image = cv::Mat(
		frame_head.iHeight, frame_head.iWidth,
		frame_head.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
		buffer.get()
	);

	mats[0] = image
		.rowRange(0, frame_head.iHeight)
		.colRange(0, frame_head.iWidth / 2);

	mats[1] = image
		.rowRange(0, frame_head.iHeight)
		.colRange(frame_head.iWidth / 2, frame_head.iWidth);

	data.assign(mats);
}

void PDNImageCamera::onceCapture(cv::OutputArray data)
{
	assert(data.kind() == cv::_InputArray::STD_VECTOR_MAT);
	std::vector<cv::Mat> isp_images_mats(2), out_images_mats(2);
	data.getMatVector(out_images_mats);

	assert(out_images_mats.size() == getViews());

	CHECK_IS_OPENED();
	CHECK_NOT_CAPTURING();

	MV_SDK_CHECK(CameraSoftTrigger(getCameraHandle()));
	
	tSdkFrameHead frame_head;
	BYTE* p_buffer = nullptr;


	MV_SDK_CHECK(CameraGetImageBuffer(getCameraHandle(), &frame_head, &p_buffer, ONE_SHOT_WAIT_MS));

	wrapIspBuffer(p_buffer, frame_head, getIspBuffer(), isp_images_mats);
	
	for(int i=0; i<2; i++)
	{
		isp_images_mats[i].copyTo(out_images_mats[i]);
	}

	data.assign(out_images_mats);

	MV_SDK_CHECK(CameraReleaseImageBuffer(getCameraHandle(), p_buffer));
}
