#pragma once

#include "AbstractCamerasFactory.h"
#include "PDNCamera.h"

#define MAX_CAMERA_CONNECTIONS 10

namespace CameraLib
{
	class CAMERALIB_DLL PDNCamerasFactory final : public AbstractCamerasFactory, public boost::noncopyable
	{
	private:
		bool sdk_init_success_;
		std::array<tSdkCameraDevInfo, 10> enum_cameras_ary_;
		int cameras_nums_;
		
	public:

		PDNCamerasFactory();
		~PDNCamerasFactory() override = default;

		std::vector<std::string> enumerateCamerasIDs() override;
		std::shared_ptr<Camera> createCamera(const std::string& id) const override;
	};
}

