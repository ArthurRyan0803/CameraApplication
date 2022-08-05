#include <iostream>
#include <PDNCamerasFactory.h>
#include <PDNCamera.h>
#include <opencv2/opencv.hpp>
#include <Utils.hpp>
#include <vtk-9.0/vtkObject.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <unordered_set>

#include "CalibrationParams.hpp"
#include "CalibrationPatternMethod.h"
#include "CalibrationMethods.h"


namespace fs = boost::filesystem;

const static fs::path data_folder(R"(C:\Users\ryan\Desktop\stereo_calib)");
const static fs::path images_folder = data_folder / "OriginalImages";
const static fs::path params_path = data_folder / "params.json";


#define VALID_POINT(PT) ((PT).x != 0 || (PT).y != 0)


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


void visualizePoints(
	pcl::visualization::PCLVisualizer& pcl_visualizer,
	std::vector<cv::Point3f> points, double r, double g, double b, const std::string& id
)
{
	size_t index = 0;
	for(auto& pt: points)
	{
		std::string id = "point" + std::to_string(index++);
		pcl_visualizer.addSphere(pcl::PointXYZ(pt.x, pt.y, pt.z), 1, id);
		pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, id);
	}
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
	image.rowRange(0, image.rows).colRange(0, image.cols / 2).copyTo(left_image);
	image.rowRange(0, image.rows).colRange(image.cols / 2, image.cols).copyTo(right_image);
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
	
	std::string message = stereoCalibration(
		left_images, right_images, 0, boardSetting(), CirclesArray,
		params, left_points, left_flags, right_points, right_flags
	);

	assert(message.empty());

	params.save((data_folder / "params.json").string());
}


int find_nearest_point(const std::vector<cv::Point2f>& src_points, float x, float y, double& dist_2)
{
	double min_dis = -1;
	int min_idx = 0;
	for(size_t i=0; i<src_points.size(); i++)
	{
		auto& pt = src_points[i];
		auto dis = cv::pow(pt.x - x, 2) + cv::pow(pt.y - y, 2);
		if(dis < min_dis || min_dis < 0)
		{
			min_dis = dis;
			min_idx = i;
		}
	}
	dist_2 = min_dis;
	return min_idx;
}


// https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html?highlight=remap#remap
void find_remapped_points(
	const cv::Mat& image_remapped, const cv::Mat& map1,
	const std::vector<cv::Point2f>& src_points, std::vector<cv::Point2f>& dst_points
)
{
	dst_points.clear();
	dst_points.resize(src_points.size());
	std::vector<double> distances(src_points.size());
	for(auto& d: distances)
		d = -1;

	for(int y=0; y<image_remapped.rows; y++)
	{
		for(int x=0; x<image_remapped.cols; x++)
		{
			if(image_remapped.at<uint8_t>(y, x) <= 50)
				continue;

			auto src_pt = map1.at<cv::Vec2s>(y, x);
			auto src_x = src_pt[0], src_y = src_pt[1];

			if(!(src_x >= 0 && src_x < image_remapped.cols) || !(src_y >= 0 && src_y < image_remapped.rows))
				continue;

			double dist_2;
			int idx = find_nearest_point(src_points, src_x, src_y, dist_2);
			if(dist_2 < 1 || (dist_2 < distances[idx] && distances[idx] > 0))
			{
				distances[idx] = dist_2;
				dst_points[idx] = cv::Point2f(x, y);
			}
		}
	}	
}


std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


int main()
{
	vtkObject::GlobalWarningDisplayOff();

	//calibrate();
	
	DualViewCalibrationParams params;
	std::cout << "loading params..." << std::endl;
	params.load(params_path.string());

	//pcl_visualize(params);

	cv::Mat image = cv::imread((images_folder / "1.png").string(), cv::IMREAD_GRAYSCALE);
	cv::imwrite((data_folder / R"(/original.png)").string(), image);

	cv::Mat left_image, right_image, left_remapped, right_remapped;

	split_image(image, left_image, right_image);

	cv::remap(
		left_image, left_remapped, 
		params.stereo.left_map1, params.stereo.left_map2, cv::INTER_LANCZOS4
	);
	cv::remap(
		right_image, right_remapped, 
		params.stereo.right_map1, params.stereo.right_map2, cv::INTER_LANCZOS4
	);

	cv::imwrite((data_folder / R"(/left_remap.png)").string(), left_remapped);
	cv::imwrite((data_folder / R"(/right_remap.png)").string(), right_remapped);

	std::cout << "Finding key points..." << std::endl;
	std::vector<cv::Point2f> left_src_points, right_src_points;
	assert(findKeyPoints(left_image, left_src_points, CirclesArray, {11, 9}));
	assert(findKeyPoints(right_image, right_src_points, CirclesArray, {11, 9}));
	assert(left_src_points.size() == right_src_points.size());
	size_t points_count = left_src_points.size();

	std::vector<cv::Point2f> left_remapped_points, right_remapped_points;

	std::cout << "Calculating remapped points..." << std::endl;
	find_remapped_points(left_remapped, params.stereo.left_map1, left_src_points, left_remapped_points);
	find_remapped_points(right_remapped, params.stereo.right_map1, right_src_points, right_remapped_points);
	
	std::vector<float> disparities(points_count);
	std::vector<size_t> valid_pt_indices;
	for(size_t i=0; i<left_remapped_points.size(); i++)
	{
		auto& left_pt = left_remapped_points[i];
		auto& right_pt = right_remapped_points[i];
		if(!VALID_POINT(left_pt) || !VALID_POINT(right_pt))
			left_pt.x = left_pt.y = right_pt.x = right_pt.y = 0;

		valid_pt_indices.push_back(i);
		disparities[i] = left_pt.x - right_pt.x;
	}

	for(auto& pt: left_remapped_points)
		cv::circle(left_remapped, pt, 2, {1.0, 0.0, 0.0, 0.0}, 2, cv::LINE_AA);
	for(auto& pt: right_remapped_points)
		cv::circle(right_remapped, pt, 2, {1.0, 0.0, 0.0, 0.0}, 2, cv::LINE_AA);

	cv::imwrite((data_folder / R"(/left_remapped_points.png)").string(), left_remapped);
	cv::imwrite((data_folder / R"(/right_remapped_points.png)").string(), right_remapped);

	cv::Mat coors;
	cv::reprojectImageTo3D(disparities, coors, params.stereo.Q, false);

	std::vector<cv::Point3f> phy_points;

	cv::Mat r_mat;
	cv::transpose(params.left.rmat, r_mat);
	r_mat = params.stereo.R1 * r_mat;

	cv::Mat t_vec = -r_mat*params.left.tvec;

	for(auto i: valid_pt_indices)
	{
		auto pt = cv::Mat(r_mat * coors.at<cv::Vec3f>(0, i) + t_vec);
		std::cout << pt << std::endl;
	}
}
