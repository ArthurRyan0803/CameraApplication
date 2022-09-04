#pragma once
#include "Framework.h"
#include "Camera.hpp"
#include "VSensor/VSensor.h"


namespace CameraLib
{
	class CAMERALIB_SDK VSensorPointCloudCamera: public Camera
	{
	public:
		
		enum SensorType
		{
			Gray = 0,
			Color = 1,
			All = 2
		};
		
		enum CaptureMode
		{
			ImageCapture = 0,
			PointCloudCapture = 1,
		};
		
		explicit VSensorPointCloudCamera(
			const std::shared_ptr<VSENSOR::VSensor>& api, int index, std::string ip
		);

		void open() override;
		void open(bool initialize);
		void close() override;
		bool isOpened() override;
		void startContinuesCapture() override;
		void stopContinuesCapture() override;
		bool isCapturing() override;
		void onceCapture(cv::OutputArray data) override;


		void setExposure(double value, SensorType type) const;
		void getExposure(double& value, SensorType type) const;

		void setExposure(double us) override;
		void getExposure(double& us) const override;
		void getExposureRange(double& min, double& max, double& step) const override;

		void getGainRange(int& min, int& max, int& step) const override;
		void setGain(int value) override;
		void getGain(int& value) const override;

		void registerFrameListener(std::weak_ptr<FrameListener> listener) override;

		void setAnalogGain(const int& r, const int& g, const int& b) const;
		void setAnalogGain(const int& value, SensorType type) const;
		void getAnalogGain(int& r, int& g, int& b) const;
		void getAnalogGain(int& value, SensorType type) const;
		void setZAxisRange(int min, int max) const;
		//void whiteBalance();
		void setCaptureMode(CaptureMode mode);
		void setLight(int open) const;

		void captureStructureLightPatternImages() const;
		void constructPointCloud(std::unique_ptr<VSensorResult>& rst) const;

		std::string getIP() const;
		


	private:
		int index_;
		std::shared_ptr<VSENSOR::VSensor> vsensor_api_;
		volatile bool is_opened_{};

		std::string ip_;
		CaptureMode capture_mode_;
		
		static constexpr uint8_t IMAGES_NUM = 11;

		std::unique_ptr<std::array<std::array<BYTE, IMAGE_HEIGHT * IMAGE_WIDTH>, IMAGES_NUM>> gray_images_buffers1_, gray_images_buffers2_;
		std::unique_ptr<std::array<BYTE, IMAGE_HEIGHT * IMAGE_WIDTH * 3>> rgb_image_buffer_;

		//void allocateGrayImagesBuffers() const;
	};
}
