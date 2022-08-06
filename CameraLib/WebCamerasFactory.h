#pragma once

#include "AbstractCamerasFactory.h"
#include "WebCamera.h"

namespace CameraLib
{
	class CAMERALIB_DLL WebCamerasFactory: public AbstractCamerasFactory, boost::noncopyable
	{

	public:
		WebCamerasFactory() = default;
		~WebCamerasFactory() override = default;
		
		std::vector<std::string> enumerateCamerasIDs() override;
		std::shared_ptr<Camera> createCamera(const std::string& id) const override;

	};
}
