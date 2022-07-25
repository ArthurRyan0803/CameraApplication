#pragma once

#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <fstream>
#include "Logger.hpp"


class SingleViewCalibrationParams
{
	Logger& logger_;

public:
	std::vector<double> intrinsic_parameters;
	std::vector<double> rotation;
	std::vector<double> translation;
	std::vector<double> distortions;

	float RMS;

	SingleViewCalibrationParams(): logger_(GET_LOGGER()), RMS(-1)
	{
		// Allocate space for params.
		intrinsic_parameters.resize(9);
		rotation.resize(9);

		translation.resize(3);
		distortions.resize(3);
	}

	void save(const std::string& path)
	{
		std::ofstream file(path);
		nlohmann::json j;
		j["intrinsic_parameters"] = intrinsic_parameters;
		j["rotation"] = rotation;
		j["translation"] = translation;
		j["distortions"] = distortions;
		file << j.dump(4) << std::endl;
	}

	//static SingleViewCalibrationParams load(const std::string& path)
	//{
	//	
	//}
};
