#include "AbstractCamerasFactory.h"


std::map<std::string, std::weak_ptr<AbstractCamerasFactory>> AbstractCamerasFactory::factories_map_;


void AbstractCamerasFactory::registerFactory(const std::string& category, const std::shared_ptr<AbstractCamerasFactory>& factory)
{
	factories_map_[category] = std::weak_ptr(factory);
}

std::shared_ptr<AbstractCamerasFactory> AbstractCamerasFactory::getCameraFactory(const std::string& category)
{
	if(factories_map_.find(category) != factories_map_.end())
	{
		if(auto factory = factories_map_[category].lock())
		{
			return factory;
		}
		throw std::invalid_argument("The factory: [" + category + "] already disposed!");
	}

	throw std::invalid_argument("Unrecognized camera category: [" + category + "]");
}

std::map<std::string, std::shared_ptr<AbstractCamerasFactory>> AbstractCamerasFactory::getAllFactories()
{
	std::map<std::string, std::shared_ptr<AbstractCamerasFactory>> return_factories;
	for(auto& pair: factories_map_)
	{
		if(auto factory = pair.second.lock())
		{
			return_factories[pair.first] = factory;
		}
	}
	return return_factories;
}

AbstractCamerasFactory::~AbstractCamerasFactory() = default;
