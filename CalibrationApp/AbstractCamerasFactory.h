#pragma once

#include <string>
#include <vector>
#include <boost/noncopyable.hpp>
#include <stdexcept>
#include <map>

#include "Camera.h"
#include <memory>


class AbstractCamerasFactory: boost::noncopyable
{
private:
	static std::map<std::string, std::weak_ptr<AbstractCamerasFactory>> factories_map_;
	
public:
	virtual const std::string& getName() const = 0;
	virtual std::vector<std::string> enumerateCameraIDs() const = 0;
	virtual std::shared_ptr<Camera> createCamera(const std::string& id) const = 0;
	virtual ~AbstractCamerasFactory() = 0;
	
	static void registerFactory(const std::string& category, const std::shared_ptr<AbstractCamerasFactory>& factory);
	static std::shared_ptr<AbstractCamerasFactory> getCameraFactory(const std::string& category);
	static std::map<std::string, std::shared_ptr<AbstractCamerasFactory>> getAllFactories();
};
