#pragma once
#include <string>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

class Def
{
public:
	Def() = delete;
	~Def() = delete;

	inline const static std::string appName = "CalibrationApp";

	const static std::string& getSysAppDir()
	{

#if _WIN32
		auto var_value = getenv("LOCALAPPDATA");
		static auto sys_app_data_path = std::string(var_value);
#else
#error Current platform is not supported now!
#endif
		
		assert(
			boost::filesystem::is_directory(sys_app_data_path) && boost::filesystem::exists(sys_app_data_path)
			&& "Why LOCALAPPDIR not exists?"
		);

		return sys_app_data_path;
	}

	const static std::string& getAppDir()
	{
		auto& sys_app_dir = getSysAppDir();
		auto app_dir = boost::filesystem::path(sys_app_dir) / appName;

		if(is_directory(app_dir))
			return sys_app_dir;
		else
		{
			if(exists(app_dir))
			{
				auto message = (boost::format("%s is not existing directory!") % app_dir).str();
				throw std::runtime_error(message);
			}
			else
			{
				if(create_directory(app_dir))
					return sys_app_dir;

				auto message = (boost::format("Create directory %s failed!") % app_dir).str();
				throw std::runtime_error(message);
			}
		}
	}
};

