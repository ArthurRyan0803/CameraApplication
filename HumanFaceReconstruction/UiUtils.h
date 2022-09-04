#pragma once

#include <QImage>
#include <stdexcept>
#include <opencv2/core/mat.hpp>
#include <boost/format.hpp>
#include <Eigen/Eigen>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>

#ifdef _WIN32
#include <Windows.h>
#endif

#include "VSensor/VSensorDefine.h"




void createQImage(const cv::Mat& src_image, std::unique_ptr<QImage>& dst_image);
void updateImageData(const cv::Mat& src_image, QImage& dst_image);
void getImagePaintRegion(
	const std::array<int, 2>& image_size, const std::array<int, 2>& canvas_size,
	std::array<int, 4>& paint_region
);
void visualizePointCloud(
	const std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clouds,
	pcl::visualization::PCLVisualizer& visualizer,
	float point_size = 1.0f
);
void updatePointSize(
	const std::string& id, float point_size,
	pcl::visualization::PCLVisualizer& visualizer
);
void getPointCloudFromPDRResult(const VSensorResult& result, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
void getColorImageFromPDRResult(const VSensorResult& result, cv::Mat& image, bool copy=false);
