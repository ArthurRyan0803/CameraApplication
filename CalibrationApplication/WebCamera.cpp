#include "WebCamera.h"
#include <opencv2/videoio.hpp>


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
		stopCapture();

	if(isOpened())
		capture_.release();
}

bool WebCamera::isOpened()
{
	return capture_.isOpened();
}

void WebCamera::oneShot(cv::OutputArray data)
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

void WebCamera::showParameterDialog()
{

}

void WebCamera::startCapture()
{
	CHECK_IS_OPENED();
	CHECK_NOT_CAPTURING();

	if(capture_thread_)
		throw std::runtime_error("Why capture thread is still alive?");

	capture_thread_ = std::make_unique<std::thread>(std::bind(&WebCamera::continuouslyCapture, this));
}

void WebCamera::stopCapture()
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


std::vector<std::array<int, 2>> WebCamera::enumerateAvailableResolutions()
{
	CHECK_IS_OPENED();

	if(!resolutions_table_.empty())
		return resolutions_table_;

	for(auto& r: RESOLUTIONS_TABLE)
	{
		if(
			capture_.set(cv::CAP_PROP_FRAME_WIDTH, r[0]) && 
			capture_.set(cv::CAP_PROP_FRAME_HEIGHT, r[1])
		)
		{
			resolutions_table_.push_back(r);
		}
	}

	if(resolutions_table_.empty())
		throw std::runtime_error("Can not find any available web camera resolution!");

	return resolutions_table_;
}

std::array<int, 2> WebCamera::getCurrentResolution()
{
	CHECK_IS_OPENED();

	auto width = capture_.get(cv::CAP_PROP_FRAME_WIDTH);
	auto height = capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
	return std::array<int, 2>{static_cast<int>(width), static_cast<int>(height)};	
}


//void WebCamera::setCurrentResolution(const std::array<int, 2>& resolution)
//{
//	CHECK_IS_OPENED();
//
//	if(!(
//		capture_.set(cv::CAP_PROP_FRAME_WIDTH, resolution[0]) &&
//		capture_.set(cv::CAP_PROP_FRAME_HEIGHT, resolution[1])
//	))
//	{
//		throw std::invalid_argument("Failed to set web camera resolution!");
//	}
//}

size_t WebCamera::getPixelType()
{
	auto format = capture_.get(cv::CAP_PROP_FORMAT);
	return static_cast<size_t>(format);
}


//void WebCamera::setResolution()
//{
//	capture_.set()
//}

/* ---------- callbacks --------- */

void WebCamera::setCapturingStartCallback(std::function<void()>&& callback)
{
	capture_start_callback_ = callback;
}

void WebCamera::setCapturingStopCallback(std::function<void()>&& callback)
{
	capture_stop_callback_ = callback;
}

void WebCamera::setFrameReadyCallback(std::function<void(cv::InputArray)> callback)
{
	frame_ready_callback_ = callback;
}

/* ---------- callbacks --------- */

const std::vector<std::array<int, 2>> WebCamera::RESOLUTIONS_TABLE =
{
	{15360, 8640},
	{8192, 8192},
	{6400, 4800},
	{7680, 4800},
	{8192, 4608},
	{7680, 4320},
	{8192, 4320},
	{10240, 4320},
	{5120, 4096},
	{6400, 4096},
	{6016, 3384},
	{6480, 3240},
	{5120, 3200},
	{4096, 3072},
	{4500, 3000},
	{5120, 2880},
	{4480, 2520},
	{3200, 2400},
	{3840, 2400},
	{4096, 2304},
	{3240, 2160},
	{3840, 2160},
	{4096, 2160},
	{5120, 2160},
	{2800, 2100},
	{2560, 2048},
	{2732, 2048},
	{3200, 2048},
	{3000, 2000},
	{2560, 1920},
	{3072, 1920},
	{2736, 1824},
	{2560, 1800},
	{2880, 1800},
	{3200, 1800},
	{2304, 1728},
	{2560, 1700},
	{2880, 1620},
	{2560, 1600},
	{3840, 1600},
	{2048, 1536},
	{2256, 1504},
	{1440, 1440},
	{1800, 1440},
	{1920, 1440},
	{2160, 1440},
	{2304, 1440},
	{2560, 1440},
	{2880, 1440},
	{2960, 1440},
	{3440, 1440},
	{5120, 1440},
	{1920, 1400},
	{1856, 1392},
	{1792, 1344},
	{1600, 1280},
	{1920, 1280},
	{2048, 1280},
	{1080, 1200},
	{1600, 1200},
	{1920, 1200},
	{2160, 1200},
	{2048, 1152},
	{2436, 1125},
	{1440, 1080},
	{1920, 1080},
	{2048, 1080},
	{2280, 1080},
	{2340, 1080},
	{2400, 1080},
	{2520, 1080},
	{2538, 1080},
	{2560, 1080},
	{1400, 1050},
	{1680, 1050},
	{1024, 1024},
	{1280, 1024},
	{1440, 1024},
	{1600, 1024},
	{1776, 1000},
	{1280, 960},
	{1440, 960},
	{1152, 900},
	{1440, 900},
	{1440, 900},
	{1600, 900},
	{2880, 900},
	{1152, 864},
	{1280, 854},
	{1120, 832},
	{1024, 800},
	{1280, 800},
	{1024, 768},
	{1152, 768},
	{1280, 768},
	{1366, 768},
	{1600, 768},
	{1334, 750},
	{960, 720},
	{1152, 720},
	{1280, 720},
	{960, 640},
	{1024, 640},
	{1136, 640},
	{1138, 640},
	{832, 624},
	{800, 600},
	{1024, 600},
	{1024, 576},
	{960, 544},
	{960, 540},
	{640, 512},
	{480, 500},
	{368, 480},
	{512, 480},
	{600, 480},
	{640, 480},
	{768, 480},
	{800, 480},
	{848, 480},
	{854, 480},
	{320, 400},
	{640, 400},
	{312, 390},
	{496, 384},
	{512, 384},
	{720, 364},
	{640, 360},
	{416, 352},
	{800, 352},
	{640, 350},
	{720, 350},
	{720, 348},
	{512, 342},
	{272, 340},
	{320, 320},
	{480, 320},
	{640, 320},
	{400, 300},
	{384, 288},
	{480, 272},
	{400, 270},
	{160, 256},
	{256, 256},
	{256, 256},
	{320, 256},
	{512, 256},
	{512, 256},
	{640, 256},
	{480, 250},
	{240, 240},
	{256, 240},
	{320, 240},
	{368, 240},
	{376, 240},
	{400, 240},
	{432, 240},
	{512, 240},
	{640, 240},
	{640, 240},
	{400, 240},
	{800, 240},
	{480, 234},
	{256, 224},
	{320, 224},
	{384, 224},
	{448, 224},
	{256, 212},
	{512, 212},
	{208, 208},
	{320, 208},
	{160, 200},
	{320, 200},
	{640, 200},
	{140, 192},
	{256, 192},
	{280, 192},
	{320, 192},
	{320, 192},
	{512, 192},
	{560, 192},
	{208, 176},
	{220, 176},
	{144, 168},
	{160, 160},
	{240, 160},
	{160, 152},
	{160, 144},
	{224, 144},
	{128, 128},
	{432, 128},
	{160, 120},
	{160, 102},
	{96, 96},
	{101, 80},
	{96, 65},
	{64, 64},
	{72, 64},
	{75, 64},
	{96, 64},
	{96, 64},
	{102, 64},
	{240, 64},
	{84, 48},
	{128, 48},
	{60, 40},
	{150, 40},
	{128, 36},
	{32, 32},
	{42, 32},
	{48, 32},
	{40, 30},
	{16, 16},
	{42, 11},
};
