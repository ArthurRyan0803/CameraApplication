#pragma once

#include <boost/format.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Eigen>

class Utils: boost::noncopyable
{
public:
	Utils() = delete;
	~Utils() = delete;
	
	/*
	 * @param: image_size: [width, height]
	 * @param: canvas_size: [width, height]
	 * @param: paint_region: [x, y, width, height]
	 */
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
			(src_image.elemSize() == 3 && dst_image.format() == QImage::Format_BGR888) ||
			(src_image.elemSize() == 1 && dst_image.format() == QImage::Format_Grayscale8)
		);

		auto size = src_image.rows * src_image.step;
		memcpy(dst_image.bits(), src_image.data, size);
	}

	template<typename Type>
	static void flat_mat_to_vector(const cv::Mat_<Type>& mat, std::vector<Type>& vector)
	{
		cv::Mat flat = mat.reshape(1, mat.total() * mat.channels());
		vector = mat.isContinuous()? flat : flat.clone();
	}

	static void createDirectory(const std::string& path)
	{
		namespace  fs = boost::filesystem;
		if(fs::exists(path))
			return;
		fs::create_directory(path);
	}

	static void createDirectory(const boost::filesystem::path& path)
	{
		namespace fs = boost::filesystem;
		if(fs::exists(path))
			return;
		fs::create_directory(path);
	}

	static void stitch_image(const cv::Mat& left_image, const cv::Mat& right_image, cv::Mat& result)
	{
		assert(left_image.rows == right_image.rows);
		assert(left_image.type() == right_image.type());

		result.release();
		result.create(left_image.rows, left_image.cols + right_image.cols, CV_MAT_TYPE(left_image.type()));
		cv::Mat left_part(result, cv::Rect(0, 0, left_image.cols, left_image.rows));
		cv::Mat right_part(result, cv::Rect(left_image.cols, 0, right_image.cols, right_image.rows));

		left_image.copyTo(left_part);
		right_image.copyTo(right_part);
	}

	template<typename TSrc, typename TDst, int Size>
	static Eigen::Matrix<TDst, Size, Size> MatrixCast(const cv::Mat& src)
	{
		assert(src.rows == Size);
		assert(src.cols == Size);

		Eigen::Matrix<TDst, Size, Size> dst;

		for (size_t r=0; r<src.rows; r++)
		{
			for (size_t c=0; c<src.cols; c++)
			{
				dst.coeffRef(r, c) = static_cast<TDst>(src.at<TSrc>(r, c));
			}
		}

		return dst;
	}

	template<typename TSrc, typename TDst, int RowSize>
	static Eigen::Matrix<TDst, RowSize, 1> VectorCast(const cv::Mat& src)
	{
		assert(src.rows == RowSize);
		assert(src.cols == 1);

		Eigen::Matrix<TDst, RowSize, 1> dst;

		for (size_t r=0; r<src.rows; r++)
		{
			dst.coeffRef(r, 0) = static_cast<TDst>(src.at<TSrc>(r, 0));
		}

		return dst;
	}
};
