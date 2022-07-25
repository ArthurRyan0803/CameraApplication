#include "WebCamerasFactory.h"
#include <opencv2/videoio/videoio.hpp>
#include "WebCamera.h"


const std::string WebCamerasFactory::name("WebCamera");

std::shared_ptr<WebCamerasFactory> WebCamerasFactory::instance()
{
	static std::shared_ptr<WebCamerasFactory> instance;
	if(!instance)
	{
		instance.reset(new WebCamerasFactory);
	}
	return instance;
}


const std::string& WebCamerasFactory::getName() const
{
	return name;
}


std::vector<std::string> WebCamerasFactory::enumerateCamerasIDs()
{
	std::vector<std::string> cam_ids;
	for(int i=0;; i++)
	{
		auto camera = cv::VideoCapture();
		if(camera.open(i))
		{
			cam_ids.push_back(std::to_string(i));
		}
		else
		{
			break;
		}
	}
	return cam_ids;
}


std::shared_ptr<Camera> WebCamerasFactory::createCamera(const std::string& id) const
{
	auto camera = std::make_shared<WebCamera>(id);
	return std::static_pointer_cast<WebCamera, Camera>(camera);
}
