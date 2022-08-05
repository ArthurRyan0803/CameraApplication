#include "Framework.h"
#include "PDNCamerasFactory.h"
#include "Logger.hpp"

using namespace CameraLib;

#define USB_CAMERA "Usb3Camera0"

PDNCamerasFactory::PDNCamerasFactory(): AbstractCamerasFactory(), cameras_infos_({}), cameras_nums_(MAX_CAMERA_CONNECTIONS)
{
	int code = CameraSdkInit(0);
	sdk_init_success_ = code == CAMERA_STATUS_SUCCESS;
	//if(!sdk_init_success_)
	//	Logger::instance(__FILE__).error((boost::format("Cannot init mind vision camera sdk! error code: %1%") % code).str());
}

std::vector<std::string> PDNCamerasFactory::enumerateCamerasIDs()
{
	std::array<tSdkCameraDevInfo, 10> ary;
	auto code = CameraEnumerateDevice(ary.data(), &cameras_nums_);

	if(SDK_UNSUCCESS(code))
	{
		auto message = (boost::format("No v-sensor (mind vision) sensors found! error code %1%") % code).str();
		GET_LOGGER().error(message);
		return {};
	}

	std::vector<std::string> ids;
	for(size_t i=0; i<cameras_nums_; i++)
	{
		std::string name(ary[i].acProductSeries);
		if(name == USB_CAMERA)
		{
			std::string sn(ary[i].acSn);
			cameras_infos_[sn] = ary[i];
			ids.push_back(sn);
		}
	}

	return ids;
}


std::shared_ptr<Camera> PDNCamerasFactory::createCamera(const std::string& sn) const
{
	if(cameras_infos_.find(sn) == cameras_infos_.end())
		throw PDNCameraException("Invalid sn!", -1, __FILE__, __LINE__);

	auto info = cameras_infos_.at(sn);
	auto camera = std::make_shared<PDNCamera>(info);
	return std::static_pointer_cast<PDNCamera, Camera>(camera);
}
