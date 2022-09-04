#pragma once

#include "AbstractCamerasFactory.h"
#include "WebCamera.h"

namespace CameraLib
{
	class WebCamerasFactory: public AbstractCamerasFactory
	{

	public:
		WebCamerasFactory() = default;
		~WebCamerasFactory() override = default;
		
		std::vector<std::string> enumerateCamerasIDs() override
		{
			std::vector<std::string> cam_ids;
			for (int i = 0;; i++)
			{
				auto camera = cv::VideoCapture();
				if (camera.open(i))
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

		std::shared_ptr<Camera> createCamera(const std::string& id) const override
		{
			auto camera = std::make_shared<WebCamera>(id);
			return std::static_pointer_cast<WebCamera, Camera>(camera);
		}

	};
}
