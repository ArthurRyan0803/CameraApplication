#pragma once

#include "Framework.h"
#include "Camera.hpp"
#include "CameraDefine.H"
#include "CameraApi.h"
#include "CoupledMVImageCamera.hpp"

namespace CameraLib
{
	class CAMERALIB_SDK PDNImageCamera: public CoupledMVImageCamera
	{
	protected:
		void wrapIspBuffer(BYTE* frame_buffer, tSdkFrameHead& frame_head, const std::shared_ptr<BYTE>& buffer, cv::OutputArray data) override;

	public:
		 //void onceCapture(cv::OutputArray image) override;

		explicit PDNImageCamera(const tSdkCameraDevInfo& info);

		uint8_t getViews() override;
		void allocateBuffer(std::shared_ptr<BYTE>& buffer, int& size) override;
		
		void onceCapture(cv::OutputArray data) override;
	};
	
}
