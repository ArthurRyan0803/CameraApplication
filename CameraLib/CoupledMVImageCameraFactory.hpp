#pragma once

#include "AbstractCamerasFactory.h"
#include "PDRImageCamera.h"
#include "PDNImageCamera.h"
#include "Logger.hpp"
#include "MVImageCamera.h"
#include "MVDef.hpp"


#define MAX_CAMERA_CONNECTIONS 10


namespace CameraLib
{
	template<typename CoupledMVImageCamera_>
	class CoupledMVImageCameraFactory final : public AbstractCamerasFactory
	{
	private:

		std::map<std::string, tSdkCameraDevInfo> cameras_infos_;
		int cameras_nums_;
		Logger& logger_;

	public:

		CoupledMVImageCameraFactory() : cameras_nums_(0), logger_(GET_LOGGER())
		{
			static_assert(std::is_base_of<CoupledMVImageCamera, CoupledMVImageCamera_>());
		}

		~CoupledMVImageCameraFactory() override = default;

		std::vector<std::string> enumerateCamerasIDs() override
		{
			std::array<tSdkCameraDevInfo, 10> ary;
			cameras_nums_ = ary.size();
			auto code = CameraEnumerateDevice(ary.data(), &cameras_nums_);

			if (SDK_UNSUCCESS(code))
			{
				auto message = (boost::format("No v-sensor (mind vision) sensors found! error code %1%") % code).str();
				LOG_ERROR(logger_, message);
				return {};
			}

			std::vector<std::string> ids;
			for (size_t i = 0; i < cameras_nums_; i++)
			{
				std::string name(ary[i].acProductSeries);
				if (
						(name == "Usb3Camera0" && (std::is_same_v<CoupledMVImageCamera_, PDNImageCamera>)) ||
						(name == "GiGeCamera" && std::is_same_v<CoupledMVImageCamera_, PDRImageCamera>)
					)
				{
					std::string sn(ary[i].acSn);
					cameras_infos_[sn] = ary[i];
					ids.push_back(sn);
				}
			}

			return ids;
		}

		std::shared_ptr<Camera> createCamera(const std::string& sn) const override
		{
			return std::static_pointer_cast<Camera, CoupledMVImageCamera_>(createMVCamera(sn));
		}

		std::shared_ptr<CoupledMVImageCamera_> createMVCamera(const std::string& sn) const
		{
			if (cameras_infos_.find(sn) == cameras_infos_.end())
				THROW_MV_SDK_EXCEPTION("Invalid sn!", -1);

			auto info = cameras_infos_.at(sn);
			auto camera = std::make_shared<CoupledMVImageCamera_>(info);
			return camera;
		}
	};
}
