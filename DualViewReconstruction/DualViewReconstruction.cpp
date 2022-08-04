#include <iostream>
#include <PDNCamerasFactory.h>
#include <PDNCamera.h>
#include <opencv2/opencv.hpp>
#include <CalibrationParams.hpp>
#include <Utils.hpp>
#include <vtk-9.0/vtkObject.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <CalibrationPatternMethod.h>


const static std::string param_file_path = R"(C:\Users\ryan\Desktop\stereo_calib\params.json)";


void visualizeCalibPlanar(
	pcl::visualization::PCLVisualizer& pcl_visualizer,
	int horizontal_count, int vertical_count, int vertical, double r, double g, double b, const std::string& id
)
{
	auto w = horizontal_count * vertical,
			h = vertical_count * vertical,
			z = vertical;

	pcl_visualizer.addCube(
		Eigen::Vector3f(w / 2,  h / 2, -z/2), Eigen::Quaternionf::Identity(), 
		w, h, z, id
	);
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



//pcl::visualization::PCLVisualizer pcl_visualizer;
//pcl_visualizer.initCameraParameters();
//visualizeCalibPlanar(pcl_visualizer, 11, 9, 15, 0.7, 0.7, 0.7, "calib_planar");
//visualizeCamera(pcl_visualizer, params.left, params.stereo.R1, "left_camera", 0, 1, 0);
//visualizeCamera(pcl_visualizer, params.right, params.stereo.R2, "right_camera", 0, 0, 1);
//pcl_visualizer.addCoordinateSystem(100);
//pcl_visualizer.resetCamera();
//
//while(!pcl_visualizer.wasStopped())
//{
//	pcl_visualizer.spinOnce(100);
//	std::this_thread::sleep_for(std::chrono::milliseconds(100));
//}


int main()
{
	vtkObject::GlobalWarningDisplayOff();

	std::cout << "Loading calibration params..." << std::endl; 
	DualViewCalibrationParams params;
	std::ifstream is(param_file_path);
	is >> params;
	
	std::cout << "Taking image..." << std::endl;
	CameraLib::PDNCamerasFactory camera_factory;
	auto ids = camera_factory.enumerateCamerasIDs();
	auto camera = camera_factory.createCamera(ids[0]);

	camera->open();
	cv::Mat image;
	camera->oneShot(image);
	camera->close();
	
	int cols = image.cols / 2;
	int rows = image.rows;
	cv::Mat left_image, right_image;
	cv::Mat left_undistort_image, right_undistort_image;

	cv::Mat copy;
	image.copyTo(copy);

	image.rowRange(0, rows).colRange(0, cols).copyTo(left_image);
	image.rowRange(0, rows).colRange(cols, cols * 2).copyTo(right_image);

	cv::imwrite(R"(C:\Users\ryan\Desktop\test\entire.png)", image);
	cv::imwrite(R"(C:\Users\ryan\Desktop\test\left.png)", left_image);
	cv::imwrite(R"(C:\Users\ryan\Desktop\test\right.png)", right_image);

	cv::undistort(left_image, left_undistort_image, params.left.camera_matrix, params.left.distortions);
	cv::undistort(right_image, right_undistort_image, params.right.camera_matrix, params.right.distortions);
	
	cv::imwrite(R"(C:\Users\ryan\Desktop\test\left_undistort.png)", left_undistort_image);
	cv::imwrite(R"(C:\Users\ryan\Desktop\test\right_undistort.png)", right_undistort_image);
	
	std::cout << "Finding key points..." << std::endl;
	std::vector<cv::Point2f> left_points, right_points;
	bool success = findKeyPoints(left_undistort_image, left_points, CirclesArray, {11, 9});
	if(!success)
		std::cout << "Failed to find left points!" << std::endl;

	success = findKeyPoints(right_undistort_image, right_points, CirclesArray, {11, 9});
	if(!success)
		std::cout << "Failed to find right points!" << std::endl;
	
	auto type = CV_MAT_TYPE(params.left.camera_matrix.flags);

	cv::Mat RT_l(3, 4, type), RT_r(3, 4, type);
	RT_l.rowRange(0, 3).colRange(0, 3) = params.left.rmat;
	RT_l.rowRange(0, 3).colRange(3, 4) = params.left.tvec;
	cv::Mat P_l = params.left.camera_matrix * RT_l;

	RT_r.rowRange(0, 3).colRange(0, 3) = params.right.rmat;
	RT_r.rowRange(0, 3).colRange(3, 4) = params.right.tvec;
	cv::Mat P_r = params.right.camera_matrix * RT_r;

	std::vector<cv::Point3f> phy_points { cv::Point3f(0, 0, 0) };
	std::vector<cv::Point2f> left_pix_points, right_pix_points;
	cv::projectPoints(
		phy_points, params.left.rvec, params.left.tvec, 
		params.left.camera_matrix, params.left.distortions, left_pix_points
	);
	cv::projectPoints(
		phy_points, params.right.rvec, params.right.tvec, 
		params.right.camera_matrix, params.right.distortions, right_pix_points
	);

	for(auto& pt: left_pix_points)
		std::cout << pt << std::endl;

	for(auto& pt: right_pix_points)
		std::cout << pt << std::endl;

	getchar();

	for(size_t i=0; i<left_points.size(); i++)
	{
		auto left_point = left_points[i];
		auto right_point = right_points[i];

		auto type = CV_MAT_TYPE(params.left.camera_matrix.flags);

		cv::Mat RT_l(3, 4, type), RT_r(3, 4, type);
		RT_l.rowRange(0, 3).colRange(0, 3) = params.left.rmat;
		RT_l.rowRange(0, 3).colRange(3, 4) = params.left.tvec;
		cv::Mat P_l = params.left.camera_matrix * RT_l;

		RT_r.rowRange(0, 3).colRange(0, 3) = params.right.rmat;
		RT_r.rowRange(0, 3).colRange(3, 4) = params.right.tvec;
		cv::Mat P_r = params.right.camera_matrix * RT_r;
		
		cv::Mat params_(4, 3, type);
		params_.row(0) = left_point.x * P_l.row(2) - P_l.row(0);
		params_.row(1) = left_point.y * P_l.row(2) - P_l.row(1);
		params_.row(2) = right_point.x * P_r.row(2) - P_r.row(0);
		params_.row(3) = right_point.y * P_r.row(2) - P_r.row(1);

		cv::Mat X;
		cv::solve(params_, cv::Mat::zeros(4, 1, type), X, cv::DECOMP_SVD);

		std::cout << X.t() << std::endl;
	}
}