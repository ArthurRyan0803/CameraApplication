#include <iostream>
#include <thread>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointXYZRGBNormal PointNormal;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::PointCloud<PointNormal> CloudWithNormal;

#define SEARCH_RADIUS 8
#define VOXEl_SIZE 1.0f

#define _OPENMP

int main()
{
	auto path_in = R"(C:\Users\ryan\Desktop\facial_clouds\face_clean.ply)";
	auto path_out = R"(C:\Users\ryan\Desktop\facial_clouds\face_mls.ply)";

	Cloud::Ptr cloud_in = std::make_shared<Cloud>();
	CloudWithNormal cloud_out;

	pcl::io::loadPLYFile(path_in, *cloud_in);
	auto tree = std::make_shared<pcl::search::KdTree<Point>>();


	pcl::VoxelGrid<Point> voxel_downsample;

	voxel_downsample.setInputCloud(cloud_in);
	voxel_downsample.setLeafSize(VOXEl_SIZE, VOXEl_SIZE, VOXEl_SIZE);
	voxel_downsample.filter(*cloud_in);
	
	pcl::MovingLeastSquares<Point, PointNormal> mls;

	std::cout << "MLS..." << std::endl;

	mls.setInputCloud(cloud_in);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(SEARCH_RADIUS);
	mls.setComputeNormals(true);
	mls.setPolynomialOrder(2);
	mls.setSqrGaussParam(0.0001);
	mls.setNumberOfThreads(std::thread::hardware_concurrency());

	mls.process(cloud_out);

	pcl::io::savePLYFileBinary(path_out, cloud_out);

}
