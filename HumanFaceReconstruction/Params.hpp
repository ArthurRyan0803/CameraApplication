#pragma once

#include <opencv2/opencv.hpp>
#include <CalibrationParams.hpp>
#include <Eigen/eigen>
#include <Utils.hpp>
#include <QDebug>


class StitchingTransformParams: public SerializableParams
{
public:
	cv::Mat CalibR, CalibT;
	PlanarCalibrationParams left, right;

	virtual ~StitchingTransformParams() = default;

	cv::Mat cvTransformation()
	{
		cv::Mat calib_transform = cv::Mat::eye(4, 4, CV_64FC1);
		CalibR.copyTo(calib_transform.rowRange(0, 3).colRange(0, 3));
		CalibT.copyTo(calib_transform.rowRange(0, 3).colRange(3, 4));

		return calib_transform;
	}

	Eigen::Matrix4f eigenTransformation()
	{
		auto cv_transform = cvTransformation();
		return Utils::matrixCast<double, float, 4>(cv_transform);
	}

	void copyFromStereoCalibParams(const DualViewCalibrationParams& params)
	{
		CalibR = params.stereo.R.clone();
		CalibT = params.stereo.T.clone();

		left = params.left;
		right = params.right;
	}


private:
	void fsSerialize(cv::FileStorage& fs) const override
	{
		fs << "CalibR" << CalibR;
		fs << "CalibT" << CalibT;

		fs << "left" << left;
		fs << "right" << right;
	}

	void fsDeserialize(cv::FileStorage& fs) override
	{
		fs["CalibR"] >> CalibR;
		fs["CalibT"] >> CalibT;

		fs["left"] >> left;
		fs["right"] >> right;
	}
};


class PointCloudProcessParams: public SerializableParams
{
public:
	float voxel_size {1.0};
    unsigned char filter_neighbors {50};
    double filter_stddev_thd {0.7f};
    unsigned char icp_iters {10};
	bool process_stitched_cloud { true };

	void fsSerialize(cv::FileStorage& fs) const override
	{
		fs << "voxel_size" << voxel_size;
		fs << "filter_neighbors" << filter_neighbors;
		fs << "filter_stddev_thd" << filter_stddev_thd;
		fs << "icp_iters" << icp_iters;
		fs << "process_stitched_cloud" << process_stitched_cloud;
	}

	void fsDeserialize(cv::FileStorage& fs) override
	{
		fs["voxel_size"] >> voxel_size;
		fs["filter_neighbors"] >> filter_neighbors;
		fs["filter_stddev_thd"] >> filter_stddev_thd;
		fs["icp_iters"] >> icp_iters;
		fs["process_stitched_cloud"] >> process_stitched_cloud;
	}
};


class CameraParams: public SerializableParams
{
public:
	double gray_exposure {10};
	double gray_gain {3};
	double color_exposure {40};
	double color_gain {10};

	void fsSerialize(cv::FileStorage& fs) const override
	{
		fs << "gray_exposure" << gray_exposure;
		fs << "gray_gain" << gray_gain;
		fs << "color_exposure" << color_exposure;
		fs << "color_gain" << color_gain;
	}

	void fsDeserialize(cv::FileStorage& fs) override
	{
		fs["gray_exposure"] >> gray_exposure;
		fs["gray_gain"] >> gray_gain;
		fs["color_exposure"] >> color_exposure;
		fs["color_gain"] >> color_gain;
	}

	void fsNodeDeserialize(const cv::FileNode& node) override
	{
		node["gray_exposure"] >> gray_exposure;
		node["gray_gain"] >> gray_gain;
		node["color_exposure"] >> color_exposure;
		node["color_gain"] >> color_gain;
	}
};


class DualCamerasParams: public SerializableParams
{
public:
	CameraParams left_camera;
	CameraParams right_camera;

	void fsSerialize(cv::FileStorage& fs) const override
	{
		fs << "left_camera" << left_camera;
		fs << "right_camera" << right_camera;
	}

	void fsDeserialize(cv::FileStorage& fs) override
	{
		fs["left_camera"] >> left_camera;
		fs["right_camera"] >> right_camera;
	}
};

class VisualizationParams: public SerializableParams
{
public:
	float point_size {1.0f};
	float left_right_cloud_visiable { false };

	void fsSerialize(cv::FileStorage& fs) const override
	{
		fs << "point_size" << point_size;
		fs << "left_right_cloud_visiable" << left_right_cloud_visiable;
	}

	void fsDeserialize(cv::FileStorage& fs) override
	{
		fs["point_size"] >> point_size;
		fs["left_right_cloud_visiable"] >> left_right_cloud_visiable;
	}
};
