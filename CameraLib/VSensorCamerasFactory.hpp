#pragma once

#include "AbstractCamerasFactory.h"
#include "PDNCamera.h"
#include "PDRCamera.h"
#include "Logger.hpp"
#include "VSensorDef.h"


#define MAX_CAMERA_CONNECTIONS 10


namespace CameraLib
{
	template<class VSensorCamera>
	class VSensorCameraFactory final : public AbstractCamerasFactory
	{
	private:
		
		std::map<std::string, tSdkCameraDevInfo> cameras_infos_;
		int cameras_nums_;
		Logger& logger_;
		
	public:

		VSensorCameraFactory(): cameras_nums_(0), logger_(GET_LOGGER())
		{
			static_assert(std::is_base_of_v<Camera, VSensorCamera>);
			static_assert(std::is_same_v<VSensorCamera, PDNCamera> || std::is_same_v<VSensorCamera, PDRCamera>);
			static bool has_init_ = false;

			if (has_init_) return;

			try
			{
				VSENSOR_SDK_TRACK(CameraSetSysOption("NumBuffers", "8"));
				VSENSOR_SDK_TRACK(CameraSdkInit(0));
				has_init_ = true;
				LOG_DEBUG(logger_, "MVSDK init success!");
			}
			catch (const std::exception& e)
			{
				LOG_ERROR(logger_, e.what());
				throw e;
			}
		}

		~VSensorCameraFactory() override = default;

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
						(name == "Usb3Camera0" && std::is_same_v<VSensorCamera, PDNCamera>) ||
						(name == "GiGeCamera" && std::is_same_v<VSensorCamera, PDRCamera>)
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
			if (cameras_infos_.find(sn) == cameras_infos_.end())
				THROW_VSENSOR_EXCEPTION("Invalid sn!", -1);

			auto info = cameras_infos_.at(sn);
			auto camera = std::make_shared<VSensorCamera>(info);
			return std::static_pointer_cast<VSensorCamera, Camera>(camera);
		}
	};
}

