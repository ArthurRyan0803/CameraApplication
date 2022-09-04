#pragma once

#include <memory>

#include "AbstractCamerasFactory.h"
#include "VSensorDef.hpp"
#include "VSensor/VSensorDefine.h"
#include "VSensor/VSensor.h"
#include "VSensorPointCloudCamera.h"


namespace CameraLib
{
	class VSensorPointCloudCamerasFactory: AbstractCamerasFactory
	{
	private:
		std::map<std::string, std::shared_ptr<VSENSOR::VSensor>> apis_map_;
		std::map<std::string, int> id_to_index_map_;
		std::map<std::string, std::string> id_to_ip_map_;

	public:

		std::vector<std::string> enumerateCamerasIDs() override
		{
			auto api_context = std::make_unique<VSENSOR::VSensor>();
			int found_cameras = 10;
			std::array<VSensorCameraInfo, 10> infos {};
			VSENSOR_SDK_CHECK(api_context->GetDeviceList(infos.data(), &found_cameras));
			api_context.reset();

			std::vector<std::string> ids;
			for(size_t i=0; i<found_cameras; i++)
			{
				auto new_api_context_ = std::make_shared<VSENSOR::VSensor>();

				// let new context scan device again.
				VSENSOR_SDK_CHECK(new_api_context_->GetDeviceList(infos.data(), &found_cameras));

				std::string id = std::to_string(infos[i].uInstance);
				apis_map_[id] = new_api_context_;
				id_to_index_map_[id] = i;
				id_to_ip_map_[id] = std::string(infos[i].Address);
				ids.push_back(id);
			}

			return ids;
		}

		std::shared_ptr<Camera> createCamera(const std::string& id) const override 
		{
			return createPointCloudCamera(id);
		}
		
		std::shared_ptr<VSensorPointCloudCamera> createPointCloudCamera(const std::string& id) const 
		{
			int index = id_to_index_map_.at(id);
			auto ip = id_to_ip_map_.at(id);
			auto api = apis_map_.at(id);
			auto camera = std::make_shared<VSensorPointCloudCamera>(api, index, ip);
			return camera;
		}

		~VSensorPointCloudCamerasFactory() override = default;
	};
}
