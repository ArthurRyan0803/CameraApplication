#pragma once

#include <memory>

#include "AbstractCamerasFactory.h"

class WebCamerasFactory: public AbstractCamerasFactory
{
protected:
	WebCamerasFactory() = default;

public:
	static const std::string name;

	static std::shared_ptr<WebCamerasFactory> instance();
	
	~WebCamerasFactory() override = default;
	std::vector<std::string> enumerateCamerasIDs() override;
	std::shared_ptr<Camera> createCamera(const std::string& id) const override;
	const std::string& getName() const override;
};
