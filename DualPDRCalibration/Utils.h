#pragma once

#include <memory>
#include <QImage>
#include <stdexcept>
#include <opencv2/core/mat.hpp>
#include <boost/format.hpp>


static void createQImage(const cv::Mat& src_image, std::unique_ptr<QImage>& dst_image)
{
	auto element_size = src_image.elemSize();
	auto channels = src_image.channels();

	auto q_image_format = QImage::Format_Invalid;
	if(channels == 3 && element_size == 3)
		q_image_format = QImage::Format_BGR888;
	else if (channels == 1 && element_size == 1)
		q_image_format = QImage::Format_Grayscale8;
	else
		throw std::runtime_error(
			(boost::format("The frame channels [%1%] and element_size [%2%] are not supported simultaneously!") % channels % element_size)
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


static std::array<int, 2> convertPaintPositions(
	std::array<int, 2> image_size, std::array<int, 4> paint_region, std::array<float, 2> point
)
{
	int x = paint_region[0] + paint_region[2] * (point[0] / static_cast<float>(image_size[0]));
	int y = paint_region[1] + paint_region[3] * (point[1] / static_cast<float>(image_size[1]));

	return std::array<int, 2> {x, y};
}