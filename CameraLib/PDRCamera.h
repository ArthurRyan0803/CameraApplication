#pragma once

#include <semaphore>

#include "Camera.h"
#include "CameraApi.h"
#include "CameraDefine.H"


namespace CameraLib
{
	class PDRBuffer;

	class CAMERALIB_DLL PDRCamera: public Camera
	{
	private:
		volatile bool is_opened_{ false };
		volatile bool is_snap_{ false };
		volatile bool is_capturing_ {false};

		// Members for one shot (snap)
		std::mutex snap_mutex_;
		std::condition_variable snap_condition_var_ {};
		BYTE* snap_src_buffer_{};
		tSdkFrameHead* snap_frame_head_{};

		std::unique_ptr<PDRBuffer> isp_buffer_;

		void* grabber_handle_{};
		CameraHandle cam_handle_{};
		tSdkCameraDevInfo info_;
		static int vsensor_frame_ready_callback(void* grabber, int phase, BYTE* frame_buffer, tSdkFrameHead* frame_head, void* context);
		
		std::function<void(cv::InputArray)> frame_ready_callback_;

	public:
		PDRCamera(const tSdkCameraDevInfo& info);
		~PDRCamera() override;
		void open() override;
		void close() override;
		bool isOpened() override;
		void startCapture() override;
		void stopCapture() override;
		bool isCapturing() override;
		size_t getViews() override;
		void showParameterDialog() override;
		void oneShot(cv::OutputArray data) override;
		void setFrameReadyCallback(std::function<void(cv::InputArray)> callback) override;
	};
}

