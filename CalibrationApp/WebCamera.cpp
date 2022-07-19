#include "WebCamera.h"
#include <opencv2/videoio.hpp>

WebCamera::WebCamera(std::string id): Camera()
{
	if(id.empty())
		throw std::invalid_argument("id is empty!");

	if(!std::all_of(id.begin(), id.end(), ::isdigit))
		throw std::invalid_argument((boost::format("id [%1%] is not valid numerical string!") % 1).str());

	this->id_ = std::stoi(id);
}

WebCamera::~WebCamera()
{
	close();
}

void WebCamera::open()
{
	if(!capture_.open(id_))
	{
		auto message = boost::format("Failed to open camera of id [%1%]") % id_;
		throw std::runtime_error(message.str());
	}
}

void WebCamera::close()
{
	if(isCapturing())
		stopCapture();

	if(isOpened())
		capture_.release();
}

bool WebCamera::isOpened()
{
	return capture_.isOpened();
}

bool WebCamera::oneShot(cv::OutputArray data)
{
	checkIsOpened();
	checkNotCapturing();
	
	auto success = capture_.read(data);
	if(!success)
		return false;
	
	return true;
}

void WebCamera::startCapture()
{
	checkIsOpened();
	checkNotCapturing();

	if(capture_thread_)
		throw std::runtime_error("Why capture thread is still alive?");

	capture_thread_ = std::make_unique<std::thread>(std::bind(&WebCamera::continuouslyCapture, this));
}

void WebCamera::stopCapture()
{
	checkIsOpened();

	if(!isCapturing())
		return;

	if(!capture_thread_)
		throw std::runtime_error("Why capture thread is not alive?");

	stop_capturing_request_ = true;
	capture_thread_->join();
	capture_thread_.reset();
	stop_capturing_request_ = false;
}

bool WebCamera::isCapturing()
{
	return is_capturing_;
}

void WebCamera::continuouslyCapture()
{
	is_capturing_ = true;
	if(capture_start_callback_)
		capture_start_callback_();
	
	while(!stop_capturing_request_)
	{
		cv::Mat frame;
		if(!capture_.read(frame))
		{
			throw std::runtime_error("Failed to grab image in continuouslyCapture() !");
		}
		if(frame_ready_callback_)
			frame_ready_callback_(frame);
	}
	
	is_capturing_ = false;
	if(capture_stop_callback_)
		capture_stop_callback_();
}

void WebCamera::checkIsOpened()
{
	if(!isOpened())
		throw std::logic_error("Camera is not opened!");
}

void WebCamera::checkNotCapturing()
{
	if(isCapturing())
		throw std::logic_error("Camera is capturing! Please stop it first!");
}

void WebCamera::checkIsCapturing()
{
	if(!isCapturing())
		throw std::logic_error("Camera is not capturing! Please start it first!");
}

std::array<int, 2> WebCamera::getResolution()
{
	auto width = capture_.get(cv::CAP_PROP_FRAME_WIDTH);
	auto height = capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
	return std::array<int, 2>{static_cast<int>(width), static_cast<int>(height)};	
}

/* ---------- callbacks --------- */

void WebCamera::setCapturingStartCallback(std::function<void()>&& callback)
{
	capture_start_callback_ = callback;
}

void WebCamera::setCapturingStopCallback(std::function<void()>&& callback)
{
	capture_stop_callback_ = callback;
}

void WebCamera::setFrameReadyCallback(std::function<void(const cv::Mat&)>&& callback)
{
	frame_ready_callback_ = callback;
}

/* ---------- callbacks --------- */
