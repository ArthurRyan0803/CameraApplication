#include <opencv2/imgproc.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "PDNCamerasFactory.h"
#include "Logger.hpp"


typedef PDNCamera::SensorMode SensorMode;


SensorMode getMode(const std::string& mode_str)
{
	auto str = boost::to_upper_copy<std::string>(mode_str);
	if(str == "LEFT")
		return SensorMode::Left;
	if(str == "RIGHT")
		return SensorMode::Right;
	if(str == "BOTH")
		return SensorMode::Both;
	
	throw std::invalid_argument("Unrecognized sensor mode: " + mode_str);
}


std::string getModeStr(SensorMode mode)
{
	switch(mode)
	{
	case SensorMode::Left: return "left"; 
	case SensorMode::Right: return "right";
	case SensorMode::Both: return "both";
	default: throw std::invalid_argument("Unrecognized sensor mode type!");
	}
}


const std::string PDNCamerasFactory::name("PDNCamera");


PDNCamerasFactory::PDNCamerasFactory(): AbstractCamerasFactory(), enum_cameras_ary_({}), cameras_nums_(MAX_CAMERA_CONNECTIONS)
{
	int code = CameraSdkInit(0);
	sdk_init_success_ = code == CAMERA_STATUS_SUCCESS;
	if(!sdk_init_success_)
		Logger::instance(__FILE__).error((boost::format("Cannot init mind vision camera sdk! error code: %1%") % code).str());
}


std::shared_ptr<PDNCamerasFactory> PDNCamerasFactory::instance()
{
	static std::shared_ptr<PDNCamerasFactory> instance;
	if(!instance)
	{
		instance.reset(new PDNCamerasFactory);
	}
	return instance;
}


const std::string& PDNCamerasFactory::getName() const
{
	return name;
}


std::vector<std::string> PDNCamerasFactory::enumerateCamerasIDs()
{
	auto code = CameraEnumerateDevice(enum_cameras_ary_.data(), &cameras_nums_);
	if(SDK_UNSUCCESS(code))
	{
		auto message = (boost::format("No v-sensor (mind vision) sensors found! error code %1%") % code).str();
		Logger::instance(__FILE__).error(message);
		return {};
	}
	
	std::vector<std::string> ids;
	for(int i=0; i<cameras_nums_; i++)
	{
		ids.push_back((boost::format("%1%_%2%") % i % getModeStr(SensorMode::Left)).str());
		ids.push_back((boost::format("%1%_%2%") % i % getModeStr(SensorMode::Right)).str());
		ids.push_back((boost::format("%1%_%2%") % i % getModeStr(SensorMode::Both)).str());
	}

	return ids;
}


std::shared_ptr<Camera> PDNCamerasFactory::createCamera(const std::string& id) const
{
	std::vector<std::string> strs;

	boost::split(strs, id,boost::is_any_of("_"));
	if(strs.size() != 2)
		throw std::invalid_argument("Invalid camera id! correct format: id_mode");
	auto index_str = strs[0];

	if(index_str.empty())
		throw std::invalid_argument("Invalid camera id!");

	if(!std::all_of(index_str.begin(), index_str.end(), ::isdigit))
		throw std::invalid_argument((boost::format("Mv camera index [%1%] is not valid numerical string!") % 1).str());

	auto camera_index = std::stoi(index_str);
	if(camera_index >= cameras_nums_)
		throw std::invalid_argument("Camera index out of range!");
	
	auto camera = std::make_shared<PDNCamera>(enum_cameras_ary_[camera_index], getMode(strs[1]));
	return std::static_pointer_cast<PDNCamera, Camera>(camera);
}
