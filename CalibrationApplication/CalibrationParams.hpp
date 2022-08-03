#pragma once

#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <fstream>
#include "Logger.hpp"


class PlanarCalibrationParams
{
	Logger& logger_;

public:
	//std::vector<double> camera_matrix;
	//std::vector<double> rotation;
	//std::vector<double> translation;
	//std::vector<double> distortions;

	cv::Mat camera_matrix;
	cv::Mat rvec, rmat;
	cv::Mat tvec;
	std::vector<double> distortions;
	double RMS;

	PlanarCalibrationParams(): logger_(GET_LOGGER()), RMS(-1)
	{
		// Allocate space for params.
		//camera_matrix.resize(9);
		//rotation.resize(9);

		//translation.resize(3);
		//distortions.resize(3);
	}

	void save(const std::string& path)
	{
		//std::ofstream file(path);
		//nlohmann::json j;
		//j["camera_matrix"] = camera_matrix;
		//j["rotation"] = rotation;
		//j["translation"] = translation;
		//j["distortions"] = distortions;
		//j["RMS"] = RMS;
		//file << j.dump(4) << std::endl;
	}

	//static SingleViewCalibrationParams load(const std::string& path)
	//{
	//	
	//}
};


class StereoCalibrationParams
{
	Logger& logger_;

public:

	cv::Mat R, T, E, F;
	cv::Mat R1, R2, P1, P2, Q;
	cv::Mat left_map1, left_map2;
	cv::Mat right_map1, right_map2;
	double RMS;

	StereoCalibrationParams(): logger_(GET_LOGGER()), RMS(-1)
	{
	}

	void save(const std::string& path)
	{
	}
};
