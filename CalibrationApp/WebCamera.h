#pragma once

#include <opencv2/videoio.hpp>
#include "Camera.h"
#include <boost/format.hpp>

#include "ImageData.h"

class WebCamera: public Camera
{
private:
	int id_;
	cv::VideoCapture capture_;
	volatile bool is_capturing_ = false;
	volatile bool stop_capturing_request_ = false;
	std::unique_ptr<std::thread> capture_thread_ = nullptr;

	std::function<void()> capture_start_callback_;
	std::function<void()> capture_stop_callback_;
	std::function<void(const cv::Mat&)> frame_ready_callback_;

	void continuouslyCapture();
	void checkIsOpened();
	void checkNotCapturing();
	void checkIsCapturing();

public:
	explicit WebCamera(std::string id);

	void open() override;
	void close() override;
	bool isOpened() override;

	void startCapture() override;
	void stopCapture() override;
	bool isCapturing() override;
	bool oneShot(cv::OutputArray data) override;

	std::array<int, 2> getResolution() override;

	void setCapturingStartCallback(std::function<void()>&& callback) override;
	void setCapturingStopCallback(std::function<void()>&& callback) override;
	void setFrameReadyCallback(std::function<void(const cv::Mat&)>&& callback) override;

	~WebCamera() override;
};

