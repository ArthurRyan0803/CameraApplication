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
const static fs::path remapped_folder = data_folder / "RemappedImages";
const static fs::path params_path = data_folder / "params.json";


#define VALID_POINT2(PT) ((PT).x > 1e-5 || (PT).y > 1e-5)
#define VALID_POINT3(PT) ((PT).x > 1e-5|| (PT).y > 1e-5 || (PT).z > 1e-5)
#define RANDOM_UNIFORM() (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))


CalibrationBoardSettings boardSetting()
{
	CalibrationBoardSettings board_settings;
	board_settings.horizontal_count = 11;
	board_settings.vertical_count = 9;
	board_settings.interval = 15;
	return board_settings;
}


void visualizePlanarPoints(
	pcl::visualization::PCLVisualizer& pcl_visualizer,
	const CalibrationBoardSettings& settings , const std::vector<cv::Point3f>& points,
	double r_, double g_, double b_, const std::string& id
)
{
	if (points.size() == 0)
		return;

	size_t rows = settings.vertical_count;
	size_t cols = settings.horizontal_count;

	for (size_t r = 0; r < rows; r++)
	{
		for (size_t c = 0; c < cols; c++)
		{
			int index = r * cols + c;

			auto pt = points[index];
			if(!VALID_POINT3(pt))
				continue;

			pcl::PointXYZ start(pt.x, pt.y, pt.z);

			if (r != rows - 1)
			{
				auto pt1 = points[index + cols];
				if (VALID_POINT3(pt1))
				{
					auto id_ = id + std::to_string(index) + "_";
					pcl::PointXYZ end(pt1.x, pt1.y, pt1.z);
					pcl_visualizer.addLine(start, end, id_);
					pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r_, g_, b_, id_);
				}
			}

			if (c != cols - 1)
			{
				auto pt2 = points[index + 1];
				if (VALID_POINT3(pt2))
				{
					auto id_ = id + std::to_string(index) + "__";
					pcl::PointXYZ end(pt2.x, pt2.y, pt2.z);
					pcl_visualizer.addLine(start, end, id_);
					pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r_, g_, b_, id_);
				}
			}
		}
	}
}


void visualizeCamera(
	pcl::visualization::PCLVisualizer& pcl_visualizer,
	const PlanarCalibrationParams& camera_params, const cv::Mat& R_, const std::string& id,
	double r, double g, double b
)
{
	static int w=20, h=20, d=60;

	//cv::Mat R = camera_params.rmat.t();

	cv::Mat R_new = (R_ * camera_params.rmat).t();
	cv::Mat t_new = -camera_params.rmat.t() * camera_params.tvec;
	
	auto r_eigen_mat = Utils::MatrixCast<double, float, 3>(camera_params.rmat.t());
	auto t_eigen_vec = Utils::VectorCast<double, float, 3>(- camera_params.rmat.t() * camera_params.tvec);

	auto rectified_r_eigen_mat = Utils::MatrixCast<double, float, 3>(R_new);
	auto rectified_t_eigen_mat = Utils::VectorCast<double, float, 3>(t_new);

	pcl_visualizer.addCube(t_eigen_vec, Eigen::Quaternionf(r_eigen_mat), w, h, d, id + "_original");
	pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id + "_original");
	
	pcl_visualizer.addCube(rectified_t_eigen_mat, Eigen::Quaternionf(rectified_r_eigen_mat), w, h, d, id + "_rectified");
	pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id + "_rectified");
	pcl_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, id + "_rectified");
}


void pcl_visualize(DualViewCalibrationParams& params, const std::vector<std::vector<cv::Point3f>>& planar_points)
{
	pcl::visualization::PCLVisualizer pcl_visualizer;
	pcl_visualizer.initCameraParameters();

	auto settings = boardSetting();

	//visualizeCalibPlanar(pcl_visualizer, boardSetting(), 0.7, 0.7, 0.7, "calib_planar");
	visualizeCamera(pcl_visualizer, params.left, params.stereo.R1, "left_camera", 0, 1, 0);
	visualizeCamera(pcl_visualizer, params.right, params.stereo.R2, "right_camera", 0, 0, 1);

	int planar_index = 0;
	for(auto& points: planar_points)
	{
		float r = RANDOM_UNIFORM(), g = RANDOM_UNIFORM(), b = RANDOM_UNIFORM();
		visualizePlanarPoints(pcl_visualizer, settings, points, r, g, b, "planar_points__" +  std::to_string(planar_index++));
	}

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


void read_images(std::vector<std::shared_ptr<cv::Mat>>& left, std::vector<std::shared_ptr<cv::Mat>>& right)
{
	for (size_t i = 0;; i++)
	{
		auto image_path = images_folder / (std::to_string(i) + ".png");

		if (!fs::exists(image_path))
			break;

		std::shared_ptr<cv::Mat> left_image = std::make_shared<cv::Mat>(), right_image = std::make_shared<cv::Mat>();

		auto image = cv::imread(image_path.string());

		split_image(image, *left_image, *right_image);

		left.push_back(left_image);
		right.push_back(right_image);
	}
}


void calibrate()
{
	std::vector<std::shared_ptr<cv::Mat>> left_images;
	std::vector<std::shared_ptr<cv::Mat>> right_images;
	read_images(left_images, right_images);


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
			if(image_remapped.at<uint8_t>(y, x) <= 10)
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


void paint_points(cv::Mat& image, std::vector<cv::Point2f> points)
{
	for (auto& pt : points)
	{
		cv::circle(image, pt, 2, { 1, 0, 0, 0 }, 5);
	}
}


int main()
{
	vtkObject::GlobalWarningDisplayOff();

	//calibrate();
	
	DualViewCalibrationParams params;
	std::cout << "loading params..." << std::endl;
	params.load(params_path.string());

	std::vector<std::shared_ptr<cv::Mat>> left_images;
	std::vector<std::shared_ptr<cv::Mat>> right_images;
	read_images(left_images, right_images);

	std::vector<int> indices {0, 7};
	std::vector<std::vector<cv::Point3f>> planars_points(left_images.size());
	std::vector<std::shared_ptr<std::thread>> threads;

	for(auto i: indices)
	{
		std::shared_ptr<std::thread> thread = std::make_shared<std::thread>(
			[&left_images, &right_images, i, &params, &planars_points]
			{
				std::cout << i << std::endl;

				cv::Mat left_image = *left_images[i], right_image = *right_images[i];
				cv::Mat left_remapped, right_remapped;

				cv::remap(
					left_image, left_remapped,
					params.stereo.left_map1, params.stereo.left_map2, cv::INTER_LANCZOS4
				);
				cv::remap(
					right_image, right_remapped,
					params.stereo.right_map1, params.stereo.right_map2, cv::INTER_LANCZOS4
				);

				std::cout << "Finding key points..." << std::endl;
				std::vector<cv::Point2f> left_src_points, right_src_points;
				assert(findKeyPoints(left_image, left_src_points, CirclesArray, { 11, 9 }));
				assert(findKeyPoints(right_image, right_src_points, CirclesArray, { 11, 9 }));
				assert(left_src_points.size() == right_src_points.size());
				size_t points_count = left_src_points.size();

				std::vector<cv::Point2f> left_remapped_points, right_remapped_points;

				std::cout << "Calculating remapped points..." << std::endl;
				find_remapped_points(left_remapped, params.stereo.left_map1, left_src_points, left_remapped_points);
				find_remapped_points(right_remapped, params.stereo.right_map1, right_src_points, right_remapped_points);

				paint_points(left_remapped, left_remapped_points);
				paint_points(right_remapped, right_remapped_points);

				cv::imwrite((remapped_folder / (std::to_string(i) + "_left_remap.png")).string(), left_remapped);
				cv::imwrite((remapped_folder / (std::to_string(i) + "_right_remap.png")).string(), right_remapped);
				
				std::vector<cv::Vec4d> disparities(points_count);
				std::vector<size_t> valid_pt_indices;
				for (size_t i = 0; i < left_remapped_points.size(); i++)
				{
					auto& left_pt = left_remapped_points[i];
					auto& right_pt = right_remapped_points[i];
					if (VALID_POINT2(left_pt) && VALID_POINT2(right_pt))
					{
						valid_pt_indices.push_back(i);
						disparities[i] = { left_pt.x, left_pt.y, left_pt.x - right_pt.x , 1 };
					}
				}

				std::vector<cv::Vec3d> coors;
				for (auto& t : disparities)
				{
					auto pt = cv::Mat(params.stereo.Q * t);
					auto w = pt.at<double>(3, 0);
					cv::Vec3d real_pt = { pt.at<double>(0, 0) / w, pt.at<double>(1, 0) / w, pt.at<double>(2, 0) / w };
					coors.push_back(real_pt);
				}

				std::vector<cv::Point3f> phy_points(points_count);

				auto R_new = (params.stereo.R * params.left.rmat).t();
				cv::Mat t_new = -params.left.rmat.t() * params.left.tvec;

				for (auto i : valid_pt_indices)
				{
					auto new_coor = cv::Mat(R_new * coors[i] + t_new);
					phy_points.at(i) = { (float)new_coor.at<double>(0, 0), (float)new_coor.at<double>(1, 0), (float)new_coor.at<double>(2, 0) };
				}

				planars_points[i] = phy_points;
			}
		);
		threads.push_back(thread);
	}

	for (auto& thd : threads)
		thd->join();

	pcl_visualize(params, planars_points);
}
