#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "Logger.hpp"



class SerializableParams
{
	virtual void fs_serialize(cv::FileStorage& fs) const { throw std::runtime_error("The fs_serialize method is not implemented!"); }; 
	virtual void fs_deserialize(cv::FileStorage& fs) { throw std::runtime_error("The fs_deserialize method is not implemented!"); };
	virtual void fs_node_deserialize(const cv::FileNode& node) { throw std::runtime_error("The fs_deserialize method is not implemented!"); };

public:

	void read(const cv::FileNode& node)
	{
		fs_node_deserialize(node);
	}

	void write(cv::FileStorage& fs) const
	{
		fs_serialize(fs);
	}

	void save(const std::string& path)
	{
		cv::FileStorage fs(path, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON, "utf-8");
		fs_serialize(fs);
		fs.release();
	}

	static void load(const std::string& path, SerializableParams& params)
	{
		cv::FileStorage fs(path, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON, "utf-8");
		params.fs_deserialize(fs);
		fs.release();
	}

	friend std::ostream& operator << (std::ostream& os, const SerializableParams& params)
	{
		cv::FileStorage fs("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON, "utf-8");
		params.fs_serialize(fs);
		os << fs.releaseAndGetString();
		return os;
	}

	friend std::istream& operator >> (std::istream& is, SerializableParams& params)
	{
		std::string str(std::istreambuf_iterator<char>(is), {});
		cv::FileStorage fs(str, cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON, "utf-8");
		params.fs_deserialize(fs);
		return is;
	}
};

static
void read(const cv::FileNode& node, SerializableParams& value, const SerializableParams& default_value = SerializableParams())
{
	if(node.empty())
		value = default_value;
	else
		value.read(node);
}


static void write(cv::FileStorage& fs, const std::string& name, const SerializableParams& value)
{
	fs << "{";
	value.write(fs);
	fs << "}";
}


class PlanarCalibrationParams: public SerializableParams
{
	void fs_serialize(cv::FileStorage& fs) const override
	{
		fs << "camera_matrix" << camera_matrix;
		fs << "rvec" << rvec;
		fs << "rmat" << rmat;
		fs << "tvec" << tvec;
		fs << "distortions" << distortions;
		fs << "RMS" << RMS;
	}

	void fs_deserialize(cv::FileStorage& fs) override
	{
		fs["camera_matrix"] >> camera_matrix;
		fs["rvec"] >> rvec;
		fs["rmat"] >> rmat;
		fs["tvec"] >> tvec;
		fs["distortions"] >> distortions;
		fs["RMS"] >> RMS;
	}

	void fs_node_deserialize(const cv::FileNode& node) override
	{
		node["camera_matrix"] >> camera_matrix;
		node["rvec"] >> rvec;
		node["rmat"] >> rmat;
		node["tvec"] >> tvec;
		node["distortions"] >> distortions;
		node["RMS"] >> RMS;
	}

public:
	cv::Mat camera_matrix;
	cv::Mat rvec, rmat;
	cv::Mat tvec;
	std::vector<double> distortions;
	double RMS;

	PlanarCalibrationParams(): SerializableParams(), RMS(-1)
	{
	}
};


class StereoCalibrationParams: public SerializableParams
{
	void fs_serialize(cv::FileStorage& fs) const override
	{
		fs << "R" << R;
		fs << "T" << T;
		fs << "E" << E;
		fs << "F" << F;

		fs << "R1" << R1;
		fs << "R2" << R2;
		fs << "P1" << P1;
		fs << "P2" << P2;
		fs << "Q" << Q;
		
		fs << "left_map_1" << left_map1;
		fs << "left_map_2" << left_map2;

		fs << "right_map_1" << right_map1;
		fs << "right_map_2" << right_map2;

		fs << "RMS" << RMS;
	}

	void fs_deserialize(cv::FileStorage& fs) override
	{
		fs["R"] >> R;
		fs["T"] >> T;
		fs["E"] >> E;
		fs["F"] >> F;

		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		
		fs["left_map_1"] >> left_map1;
		fs["left_map_2"] >> left_map2;

		fs["right_map_1"] >> right_map1;
		fs["right_map_2"] >> right_map2;

		fs["RMS"] >> RMS;
	}

	void fs_node_deserialize(const cv::FileNode& node) override
	{
		node["R"] >> R;
		node["T"] >> T;
		node["E"] >> E;
		node["F"] >> F;

		node["R1"] >> R1;
		node["R2"] >> R2;
		node["P1"] >> P1;
		node["P2"] >> P2;
		node["Q"] >> Q;
		
		node["left_map_1"] >> left_map1;
		node["left_map_2"] >> left_map2;

		node["right_map_1"] >> right_map1;
		node["right_map_2"] >> right_map2;

		node["RMS"] >> RMS;
	}

public:

	cv::Mat R, T, E, F;
	cv::Mat R1, R2, P1, P2, Q;
	cv::Mat left_map1, left_map2;
	cv::Mat right_map1, right_map2;
	double RMS;

	StereoCalibrationParams(): SerializableParams(), RMS(-1)
	{
	}
};


class DualViewCalibrationParams: public SerializableParams
{
	void fs_serialize(cv::FileStorage& fs) const override
	{
		fs << "left" << left;
		fs << "right" << right;
		fs << "stereo" << stereo;
	}

	void fs_deserialize(cv::FileStorage& fs) override
	{
		fs["left"] >> left;
		fs["right"] >> right;
		fs["stereo"] >> stereo;
	}

public:

	PlanarCalibrationParams left, right;
	StereoCalibrationParams stereo;

	DualViewCalibrationParams(): SerializableParams()
	{
	}
};
