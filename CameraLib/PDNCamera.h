#pragma once

#include "Framework.h"
#include "Camera.h"
#include "CameraDefine.H"
#include "CameraApi.h"

namespace CameraLib
{
	class CAMERALIB_DLL PDNCamera: public Camera
	{

	private:
		tSdkCameraDevInfo camera_info_;
		CameraHandle cam_handle_;
		volatile bool is_opened_;
		BYTE* isp_buffer_;
		std::function<void(cv::InputArray)> frame_ready_callback_;
		tSdkCameraCapbility capability_{};
		std::array<int, 10> temp2_ {};
	
		//void mvCameraCallback(CameraHandle cam_handle, BYTE *frame_buffer, tSdkFrameHead* frame_head,PVOID context);
	
		static void frameCallback(CameraHandle camera_handle, BYTE *frame_buffer, tSdkFrameHead* frame_head, PVOID context);

	public:

		 PDNCamera(const tSdkCameraDevInfo& info);
		 ~PDNCamera() override;

		 void open()  override;
		 void close() override;
		 bool isOpened() override;

		 void startCapture() override;
		 void stopCapture() override;
		 bool isCapturing() override;
		 void oneShot(cv::OutputArray image) override;

		 size_t getViews() override;
		 void showParameterDialog() override;
	
		 void setFrameReadyCallback(std::function<void(cv::InputArray)> callback) override;
	};
	
}
