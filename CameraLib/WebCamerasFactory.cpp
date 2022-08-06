#include "Framework.h"
#include "WebCamerasFactory.h"


using namespace CameraLib;


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
