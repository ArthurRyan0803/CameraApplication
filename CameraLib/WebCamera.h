#pragma once

#include "Camera.h"


namespace CameraLib
{
	class CAMERALIB_DLL WebCamera: public Camera
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

		void startCapture() override;
		void stopCapture() override;
		bool isCapturing() override;
		void oneShot(cv::OutputArray data) override;

		void showParameterDialog() override;

		std::vector<std::array<int, 2>> enumerateAvailableResolutions() override;

		// Properties
		std::array<int, 2> getCurrentResolution() override;
		//void setCurrentResolution(const std::array<int, 2>& resolution) override;

		size_t getPixelType() override;

		// callbacks
		void setFrameReadyCallback(std::function<void(cv::InputArray)> callback) override;
	};
}
