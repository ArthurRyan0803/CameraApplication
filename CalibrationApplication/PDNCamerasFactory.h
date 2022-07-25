#pragma once

#include "AbstractCamerasFactory.h"
#include "PDNCamera.h"


#define MAX_CAMERA_CONNECTIONS 10

class PDNCamerasFactory final : public AbstractCamerasFactory
{
private:
	bool sdk_init_success_;
	std::array<tSdkCameraDevInfo, 10> enum_cameras_ary_;
	int cameras_nums_;

	PDNCamerasFactory();
	
public:
	static const std::string name;

	static std::shared_ptr<PDNCamerasFactory> instance();
	
	~PDNCamerasFactory() override = default;

	std::vector<std::string> enumerateCamerasIDs() override;
	std::shared_ptr<Camera> createCamera(const std::string& id) const override;
	const std::string& getName() const override;
};

