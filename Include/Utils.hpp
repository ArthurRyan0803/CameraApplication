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


	static std::string type2str(cv::InputArray ary) 
	{
		auto type = ary.type();
		std::string r;

		uchar depth = type & CV_MAT_DEPTH_MASK;
		uchar chans = 1 + (type >> CV_CN_SHIFT);

		switch (depth) 
		{
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
		r += (chans + '0');

		return r;
	}
};
