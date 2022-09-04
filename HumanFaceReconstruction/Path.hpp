#pragma once

#include <string>
#include <Utils.hpp>

class Path
{
public:
	Path() = delete;
	~Path() = delete;

	static const std::string& getStitchingTransformParamsFilePath()
	{
		static auto& dir_path = Utils::getCurrentDirPath();
		static auto file_path = (boost::filesystem::path(dir_path) / "stitching_params.json").string();

		return file_path;
	}

	static const std::string& getPointCloudProcessParamsPath()
	{
		static auto& dir_path = Utils::getCurrentDirPath();
		static auto file_path = (boost::filesystem::path(dir_path) / "point_cloud_params.json").string();

		return file_path;
	}

	static const std::string& getCameraParamsPath()
	{
		static auto& dir_path = Utils::getCurrentDirPath();
		static auto file_path = (boost::filesystem::path(dir_path) / "cameras_params.json").string();

		return file_path;
	}

	static const std::string& getVisualizationParamsPath()
	{
		static auto& dir_path = Utils::getCurrentDirPath();
		static auto file_path = (boost::filesystem::path(dir_path) / "visualization.json").string();

		return file_path;
	}

	static const std::string& getDebugDataFolder()
	{
		static auto& dir_path = Utils::getCurrentDirPath();
		static auto file_path = (boost::filesystem::path(dir_path) / "debug").string();

		return file_path;
	}
};
