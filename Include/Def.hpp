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

	const static std::string getSysAppDir()
	{

#if _WIN32
		static char* p_buffer = nullptr;
		size_t len = 0;
		_dupenv_s(&p_buffer, &len, "LOCALAPPDATA");
		assert(p_buffer && "_dupenv_s return null!");

		//auto sys_app_data_path = std::string(getenv("LOCALAPPDATA"));
		std::string sys_app_data_path(p_buffer);
		//free(p_buffer);
#else
#error Current platform is not supported now!
#endif

		assert(
			boost::filesystem::is_directory(sys_app_data_path) && boost::filesystem::exists(sys_app_data_path)
			&& "Why LOCALAPPDIR not exists?"
		);

		return sys_app_data_path;
	}

	const static std::string getAppDir()
	{
		auto sys_app_dir = getSysAppDir();

		auto app_dir = (boost::filesystem::path(sys_app_dir) / appName);
		auto app_dir_str = app_dir.string();

		if(is_directory(app_dir))
			return app_dir_str;

		if(exists(app_dir))
		{
			auto message = (boost::format("%s is not existing directory!") % app_dir).str();
			throw std::runtime_error(message);
		}

		if(create_directory(app_dir))
			return app_dir_str;

		auto message = (boost::format("Create directory %s failed!") % app_dir).str();
		throw std::runtime_error(message);
	}
};

