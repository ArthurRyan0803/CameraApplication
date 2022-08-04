#include <iostream>
#include <PDNCamerasFactory.h>
#include <PDNCamera.h>
#include <opencv2/opencv.hpp>
#include <Utils.hpp>
#include <vtk-9.0/vtkObject.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "CalibrationParams.hpp"
#include "CalibrationPatternMethod.h"
#include "CalibrationMethods.h"


namespace fs = boost::filesystem;

const static fs::path data_folder(R"(C:\Users\ryan\Desktop\snap)");
const static fs::path params_path = data_folder / "params.json";
const static fs::path calib_folder = data_folder / "calib";


CalibrationBoardSettings boardSetting()
{
	CalibrationBoardSettings board_settings;
	board_settings.horizontal_count = 11;
	board_settings.vertical_count = 9;
	board_settings.interval = 15;
	return board_settings;
}



void visualizeCalibPlanar(
	pcl::visualization::PCLVisualizer& pcl_visualizer,
	const CalibrationBoardSettings& settings, double r, double g, double b, const std::string& id
)
{
	size_t index = 0;
	for(size_t r = 0; r < settings.vertical_count; r++)
	{
		for(size_t c = 0; c < settings.horizontal_count; c++)
		{
			pcl::PointXYZ start(c * settings.interval, r * settings.interval, 0);

			if(r != settings.vertical_count - 1)
			{
				pcl::PointXYZ end_1(c * settings.interval, (r + 1) * settings.interval, 0);
				pcl_visualizer.addLine(start, end_1, "line" + std::to_string(index++));
			}

			if(c != settings.horizontal_count - 1)
			{
				pcl::PointXYZ end_2((c + 1) * settings.interval, r * settings.interval, 0);
				pcl_visualizer.addLine(start, end_2, "line" + std::to_string(index++));
			}
		}
	}
	
	pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id);
	pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id);
}


void visualizeCamera(
	pcl::visualization::PCLVisualizer& pcl_visualizer,
	const PlanarCalibrationParams& camera_params, const cv::Mat& R, const std::string& id,
	double r, double g, double b
)
{
	static int w=20, h=20, d=60;

	cv::Mat r_mat;
	cv::transpose(camera_params.rmat, r_mat);
	cv::Mat t_mat = -r_mat*camera_params.tvec;

	cv::Mat rectified_r_mat = R * r_mat;

	auto t_eigen_vec = Utils::VectorCast<double, float, 3>(t_mat);
	auto r_eigen_mat = Utils::MatrixCast<double, float, 3>(r_mat);
	auto rectified_r_eigen_mat = Utils::MatrixCast<double, float, 3>(rectified_r_mat);

	pcl_visualizer.addCube(t_eigen_vec, Eigen::Quaternionf(r_eigen_mat), w, h, d, id + "_original");
	pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id + "_original");
	
	pcl_visualizer.addCube(t_eigen_vec, Eigen::Quaternionf(rectified_r_eigen_mat), w, h, d, id + "_rectified");
	pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id + "_rectified");
	pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id + "_rectified");
}


void pcl_visualize(DualViewCalibrationParams& params)
{
	pcl::visualization::PCLVisualizer pcl_visualizer;
	pcl_visualizer.initCameraParameters();
	visualizeCalibPlanar(pcl_visualizer, boardSetting(), 0.7, 0.7, 0.7, "calib_planar");
	visualizeCamera(pcl_visualizer, params.left, params.stereo.R1, "left_camera", 0, 1, 0);
	visualizeCamera(pcl_visualizer, params.right, params.stereo.R2, "right_camera", 0, 0, 1);
	pcl_visualizer.addCoordinateSystem(100);
	pcl_visualizer.resetCamera();

	while (!pcl_visualizer.wasStopped())
	{
		pcl_visualizer.spinOnce(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}


void split_image(const cv::Mat& image, cv::Mat& left_image, cv::Mat& right_image)
{
	image.rowRange(0, image.rows).colRange(0, image.cols / 2).copyTo(right_image);
	image.rowRange(0, image.rows).colRange(image.cols / 2, image.cols).copyTo(left_image);
}


void calibrate()
{
	std::vector<std::shared_ptr<cv::Mat>> left_images;
	std::vector<std::shared_ptr<cv::Mat>> right_images;

	for (size_t i = 1;; i++)
	{
		auto image_path = data_folder / (std::to_string(i) + ".BMP");

		if (!fs::exists(image_path))
			break;

		std::shared_ptr<cv::Mat> left_image = std::make_shared<cv::Mat>(), right_image = std::make_shared<cv::Mat>();

		auto image = cv::imread(image_path.string());

		split_image(image, *left_image, *right_image);

		left_images.push_back(left_image);
		right_images.push_back(right_image);
	}

	std::vector<std::vector<cv::Point2f>> left_points, right_points;
	std::vector<bool> left_flags, right_flags;
	DualViewCalibrationParams params;
	
	bool success = stereoCalibration(
		left_images, right_images, 0, boardSetting(), CirclesArray,
		params, left_points, left_flags, right_points, right_flags
	);

	assert(success);

	params.save((data_folder / "params.json").string());
}



int main()
{
	vtkObject::GlobalWarningDisplayOff();

	//calibrate();
	
	DualViewCalibrationParams params;
	params.load(params_path.string());

	pcl_visualize(params);

	//cv::Mat image = cv::imread((data_folder / "1.BMP").string());

	//cv::Mat left_image, right_image, left_remap, right_remap;

	//split_image(image, left_image, right_image);

	//cv::Mat left_map1, left_map2, right_map1, right_map2;
	//cv::initUndistortRectifyMap(params.left.camera_matrix, params.left.distortions, params.left.rmat, 
	//	params.stereo.P1, { image.cols / 2, image.rows }, CV_16SC2, left_map1, left_map2
	//);

	//cv::initUndistortRectifyMap(params.right.camera_matrix, params.right.distortions, params.right.rmat, 
	//	params.stereo.P2, { image.cols / 2, image.rows }, CV_16SC2, right_map1, right_map2
	//);

	//cv::remap(left_image, left_remap, left_map1, left_map2, cv::INTER_LANCZOS4);
	//cv::remap(right_image, right_remap, right_map1, right_map2, cv::INTER_LANCZOS4);

	//cv::imwrite((calib_folder / R"(/left_remap.png)").string(), left_remap);
	//cv::imwrite((calib_folder / R"(/right_remap.png)").string(), right_remap);
	//
	//std::cout << "Finding key points..." << std::endl;
	//std::vector<cv::Point2f> left_points, right_points;
	//
	//auto sgbm = cv::StereoSGBM::create();
	//cv::Mat disparity;
	//sgbm->compute(left_image, right_image, disparity);
	//cv::imwrite((calib_folder / R"(/stereo.png)").string(), disparity);

	//cv::Mat depth_image;
	//cv::reprojectImageTo3D(disparity, depth_image, params.stereo.Q);
	//cv::imwrite((calib_folder / R"(/depth.png)").string(), depth_image);

}