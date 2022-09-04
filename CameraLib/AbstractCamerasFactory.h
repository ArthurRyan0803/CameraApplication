#pragma once

#include "Framework.h"
#include "Camera.hpp"


namespace CameraLib
{
	class AbstractCamerasFactory: public boost::noncopyable
	{
	
	public:
		virtual std::vector<std::string> enumerateCamerasIDs() = 0;
		virtual std::shared_ptr<Camera> createCamera(const std::string& id) const = 0;
		virtual ~AbstractCamerasFactory() = default;
	};
}
