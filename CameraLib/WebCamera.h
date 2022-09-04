#pragma once

#include "Camera.hpp"


namespace CameraLib
{
	class CAMERALIB_SDK WebCamera: public Camera
	{
	private:
		const static std::vector<std::array<int, 2>> RESOLUTIONS_TABLE;

		int id_;
		cv::VideoCapture capture_;
		volatile bool is_capturing_ = false;
		volatile bool stop_capturing_request_ = false;
		std::unique_ptr<std::thread> capture_thread_ = nullptr;
		std::vector<std::array<int, 2>> resolutions_table_;

		std::function<void(cv::InputArray)> frame_ready_callback_;
		
		void continuouslyCapture();

	public:
		explicit WebCamera(std::string id);
		~WebCamera() override;

		// Operations
		void open() override;
		void close() override;
		bool isOpened() override;

		void startContinuesCapture() override;
		void stopContinuesCapture() override;
		bool isCapturing() override;
		void onceCapture(cv::OutputArray data) override;

		// callbacks

		void setExposure(double us) override;
		void getExposure(double& us) const override;
		void getExposureRange(double& min, double& max, double& step) const override;

		void setGain(int value) override;
		void getGain(int& value) const override;
		void getGainRange(int& min, int& max, int& step) const override;

		void registerFrameListener(std::weak_ptr<FrameListener> listener) override;
	};
}
