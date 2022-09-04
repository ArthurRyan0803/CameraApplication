#pragma once

#include <unordered_set>

#include "Framework.h"
#include "Camera.hpp"
#include "CameraDefine.H"
#include "CameraApi.h"

namespace CameraLib
{
	class CAMERALIB_SDK MVImageCamera: public Camera
	{

	private:
		volatile bool is_opened_;
		int isp_buffer_size_ {0};
		tSdkCameraDevInfo camera_info_;
		CameraHandle cam_handle_;
		std::shared_ptr<BYTE> isp_buffer_ { nullptr }; // Unique ptr does not support custom deleter.
		
		tSdkCameraCapbility capability_{};
		std::mutex listeners_mutex_;

		static void frameCallback(CameraHandle camera_handle, BYTE *frame_buffer, tSdkFrameHead* frame_head, PVOID context);
		void notifyFrameListeners(BYTE* frame_buffer, tSdkFrameHead& frame_head);

	protected:

		std::vector<std::weak_ptr<FrameListener>> frame_listeners_;

		const tSdkCameraCapbility& getCameraCapability() const;
		std::shared_ptr<BYTE> getIspBuffer() const;
		
		virtual void allocateBuffer(std::shared_ptr<BYTE>& buffer, int& size);
		virtual void wrapIspBuffer(BYTE* frame_buffer, tSdkFrameHead& frame_head, const std::shared_ptr<BYTE>& buffer, cv::OutputArray data);
		virtual void createOutputContainer(cv::_OutputArray& data);
		
	public:

		MVImageCamera(const tSdkCameraDevInfo& info);
		~MVImageCamera() override;

		void open()  override;
		void close() override;
		bool isOpened() override;

		void startContinuesCapture() override;
		void stopContinuesCapture() override;
		bool isCapturing() override;

		void onceCapture(cv::OutputArray image) override;

		void enableHardwareTrigger();
		void disableHardwareTrigger();

		//void showParameterDialog() override;

		void setExposure(double us) override;
		void getExposure(double& us) const override;
		void getExposureRange(double& min, double& max, double& step) const override;

		void setGain(int value) override;
		void getGain(int& value) const override;
		void getGainRange(int& min, int& max, int& step) const override;

		void whiteBalance() const;
		
		CameraHandle getCameraHandle() const;

		void registerFrameListener(std::weak_ptr<FrameListener> listener) override;

		virtual uint8_t getViews();

#ifdef _DEBUG
		volatile size_t callback_frames {0};
#endif
	};
	
}
