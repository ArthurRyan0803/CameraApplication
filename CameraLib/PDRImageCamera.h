#pragma once

#include <semaphore>

#include "Camera.hpp"
#include "CameraApi.h"
#include "CameraDefine.H"
#include "MVImageCamera.h"


namespace CameraLib
{
	class CAMERALIB_SDK PDRImageCamera: public MVImageCamera
	{
	public:
		enum SensorType {Gray, Color, All};
		
		PDRImageCamera(const tSdkCameraDevInfo& info);
		~PDRImageCamera() override;

		uint8_t getViews() override;

		void onceCapture(cv::OutputArray data) override;

		// unit: us
		void setExposure(double value, SensorType type) const;
		void getExposure(double& value, SensorType type) const;
		void getExposureRange(double& min, double& max, double& step, SensorType type) const;

		void setAnalogGain(int value, SensorType type) const;
		void getAnalogGain(int& value, SensorType type) const;
		void getAnalogGainRange(int& min, int& max, int& step, SensorType type) const;

		void turnOnFillLight() const;
		void turnOffFillLight() const;

	private:

		// Members for one shot (snap)
		tSdkFrameHead* snap_frame_head_{};
		BYTE* gray_isp_buffer_ {nullptr}, *rgb_isp_buffer_ {nullptr};
		
	};
}

