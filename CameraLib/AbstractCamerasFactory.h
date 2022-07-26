#pragma once

#include "Framework.h"
#include "Camera.h"


namespace CameraLib
{
	class CAMERALIB_DLL AbstractCamerasFactory: public boost::noncopyable
	{
	
	public:
		virtual std::vector<std::string> enumerateCamerasIDs() = 0;
		virtual std::shared_ptr<Camera> createCamera(const std::string& id) const = 0;
		virtual ~AbstractCamerasFactory() = default;
	};
}
