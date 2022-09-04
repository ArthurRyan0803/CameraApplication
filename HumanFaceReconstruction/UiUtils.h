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


class UiUtils
{

public:
	UiUtils() = delete;
	~UiUtils() = delete;

	static void createQImage(const cv::Mat& src_image, std::unique_ptr<QImage>& dst_image)
	{
		auto channels = src_image.channels();

		auto q_image_format = QImage::Format_Invalid;
		if(channels == 3)
			q_image_format = QImage::Format_BGR888;
		else if (channels == 1)
			q_image_format = QImage::Format_Grayscale8;
		else
			throw std::runtime_error(
				(boost::format("The frame channels [%1%] and element_size [%2%] are not supported simultaneously!") % channels)
				.str()
			);

		dst_image.reset(new QImage(src_image.cols, src_image.rows, q_image_format));
	}

	static void updateImageData(const cv::Mat& src_image, QImage& dst_image)
	{
		assert(
			(src_image.channels() == 3 && dst_image.format() == QImage::Format_BGR888) ||
			(src_image.channels() == 1 && dst_image.format() == QImage::Format_Grayscale8)
		);

		auto size = src_image.rows * src_image.step;
		memcpy(dst_image.bits(), src_image.data, size);
	}

	static void getImagePaintRegion(
		const std::array<int, 2>& image_size, const std::array<int, 2>& canvas_size,
		std::array<int, 4>& paint_region
	)
	{
		auto width_ratio = static_cast<float>(image_size[0]) / static_cast<float>(canvas_size[0]);
		auto height_ratio = static_cast<float>(image_size[1]) / static_cast<float>(canvas_size[1]);
		if(width_ratio > height_ratio)
		{
			int paint_height = static_cast<int>(image_size[1] / width_ratio);
			int y = (canvas_size[1] - paint_height) / 2;
			paint_region = std::array<int, 4>{0, y, canvas_size[0], paint_height};
		}
		else
		{
			int paint_width = static_cast<int>(image_size[0] / height_ratio);
			int x = (canvas_size[0] - paint_width) / 2;
			paint_region = std::array<int, 4>{x, 0, paint_width, canvas_size[1]};
		}
	}

	static void visualizePointCloud(
		const std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clouds,
		pcl::visualization::PCLVisualizer& visualizer,
		float point_size
	)
	{
		for(auto& pair: clouds)
		{
			if(visualizer.contains(pair.first))
				visualizer.updatePointCloud(pair.second, pair.first);
			else
				visualizer.addPointCloud(pair.second, pair.first);
			visualizer.setPointCloudRenderingProperties(
				pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, pair.first
			);
		}

		visualizer.resetCamera();
		visualizer.getRenderWindow()->Render();
	}


	static void updatePointSize(
		const std::string& id, float point_size,
		pcl::visualization::PCLVisualizer& visualizer
	)
	{
		assert(point_size > 0 && "point_size muster greater than 0!");
		assert(visualizer.contains(id) && (id + " is not in the pcl visualizer!").c_str());
	
		visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
	}


	static void getPointCloudFromPDRResult(const VSensorResult& result, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
	{
		if(!cloud)
			cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

		size_t count = IMAGE_WIDTH * IMAGE_HEIGHT;
		cloud->clear();
		for(size_t i=0; i < count; i++)
		{
			float x = result.pointx[i], y = result.pointy[i], z = result.pointz[i];
			if(!(x < 1e-5f && y < 1e-5f && z < 1e-5f))
			{
				cloud->emplace_back(pcl::PointXYZRGB(
					x, y, z, 
					result.pointr[i], result.pointg[i], result.pointb[i]
				));
			}
		}
	}


	static void getColorImageFromPDRResult(const VSensorResult& result, cv::Mat& image, bool clone)
	{
		image = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, (void*)&result.RGBMap[0]);
		cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

		if(clone)
			image = image.clone();
	}
};
