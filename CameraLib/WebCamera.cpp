#include "Framework.h"
#include "WebCamera.h"
#include <opencv2/videoio.hpp>


using namespace CameraLib;

WebCamera::WebCamera(std::string id): Camera(), resolutions_table_({})
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
	if(isOpened())
		return;

	if(!capture_.open(id_))
	{
		auto message = boost::format("Failed to open camera of id [%1%]") % id_;
		throw std::runtime_error(message.str());
	}
}

void WebCamera::close()
{
	if(isCapturing())
		stopContinuesCapture();

	if(isOpened())
		capture_.release();
}

bool WebCamera::isOpened()
{
	return capture_.isOpened();
}

void WebCamera::onceCapture(cv::OutputArray data)
{
	CHECK_IS_OPENED();
	CHECK_NOT_CAPTURING();
	
	try
	{
		auto success = capture_.read(data);
	}
	catch(const std::exception& e)
	{
		auto message = e.what();
		throw e;
	}

	//if(!success)
	//	throw std::runtime_error("Failed to read image from video capture!");
}

void WebCamera::startContinuesCapture()
{
	CHECK_IS_OPENED();
	CHECK_NOT_CAPTURING();

	if(capture_thread_)
		throw std::runtime_error("Why capture thread is still alive?");

	capture_thread_ = std::make_unique<std::thread>(std::bind(&WebCamera::continuouslyCapture, this));
}

void WebCamera::stopContinuesCapture()
{
	CHECK_IS_OPENED();

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
}

/* ---------- callbacks --------- */


void WebCamera::setExposure(double us)
{

}

void WebCamera::getExposure(double& us) const
{

}

void WebCamera::getExposureRange(double& min, double& max, double& step) const
{
}

void WebCamera::setGain(int value)
{

}

void WebCamera::getGain(int& value) const
{

}

void WebCamera::getGainRange(int& min, int& max, int& step) const
{
}

void WebCamera::registerFrameListener(std::weak_ptr<FrameListener> listener)
{
}