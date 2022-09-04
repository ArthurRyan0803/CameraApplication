#pragma once

#include <opencv2/opencv.hpp>
#include <boost/noncopyable.hpp>


class StructuredLightUtils: boost::noncopyable
{
#define M_PI 3.14159265358979323846

public:
	StructuredLightUtils() = delete;
	~StructuredLightUtils() = delete;

	static cv::Mat getMask(const cv::Mat& dark_image, const cv::Mat& bright_image, uint8_t tolerance)
	{
		assert(dark_image.size() == bright_image.size());
		auto rows = dark_image.rows, cols = dark_image.cols;

		cv::Mat mask(rows, cols, CV_8UC1);

		for(int y=0; y<rows; y++)
		{
			for(int x=0; x<cols; x++)
			{
				auto dark = dark_image.at<uint8_t>(y, x), bright = bright_image.at<uint8_t>(y, x);
				auto& value = mask.at<uint8_t>(y, x);

				if (dark >= bright || (bright - dark) <= tolerance)
					value = 0;
				else
					value = 255;
			}
		}

		return mask;
	}

	static cv::Mat getThdMap(const cv::Mat& dark_image, const cv::Mat& bright_image)
	{
		assert(dark_image.size() == bright_image.size());
		auto rows = dark_image.rows, cols = dark_image.cols;

		cv::Mat thd_map(rows, cols, CV_8UC1);

		for (int y = 0; y < rows; y++)
		{
			for (int x = 0; x < cols; x++)
			{
				auto dark = dark_image.at<uint8_t>(y, x), bright = bright_image.at<uint8_t>(y, x);
				auto& value = thd_map.at<uint8_t>(y, x);
				value = (bright - dark) / 2;
			}
		}

		return thd_map;
	}

	template<typename T>
	static T grayCodeToBinaryCode(T gray_code)
	{
		T y = gray_code;
		while (gray_code >>= 1)
			y ^= gray_code;
		return y;
	}

	static cv::Mat calBinaryCodeFromCodingImages(
		const std::vector<cv::Mat>& gray_code_images, const cv::Mat& dark_image, const cv::Mat& bright_image, cv::OutputArray thd_mat_ = cv::noArray()
	)
	{

		cv::Mat thd_mat = getThdMap(dark_image, bright_image);
		cv::Mat gray_code_mat(thd_mat.rows, thd_mat.cols, CV_8UC1, cv::Scalar(0));
		
		for(int r=0; r < thd_mat.rows; r++)
			for(int c=0; c < thd_mat.cols; c++)
			{
				auto& code = gray_code_mat.at<uint8_t>(r, c);
				auto thd = thd_mat.at<uint8_t>(r, c);
				for (const auto& image : gray_code_images)
					code = (code << 1) | (image.at<uint8_t>(r, c) > thd ? 0x01 : 0);
				code = grayCodeToBinaryCode(code);
			}
		
		if(thd_mat_.kind() == cv::_InputArray::MAT)
			thd_mat_.assign(thd_mat);

		return gray_code_mat;
	}

	static cv::Mat cal4StepPhaseMat(const std::array<cv::Mat, 4>& fringe_images, const cv::Mat& dark_image, const cv::Mat& bright_image)
	{
		cv::Mat phase_mat(fringe_images[0].rows, fringe_images[0].cols, CV_64FC1);

		for (int y = 0; y < fringe_images[0].rows; y++)
		{
			for (int x = 0; x < fringe_images[0].cols; x++)
			{
				auto i1 = fringe_images[0].at<uint8_t>(y, x);
				auto i2 = fringe_images[1].at<uint8_t>(y, x);
				auto i3 = fringe_images[2].at<uint8_t>(y, x);
				auto i4 = fringe_images[3].at<uint8_t>(y, x);

				auto min = dark_image.at<uint8_t>(y, x);
				auto max = bright_image.at<uint8_t>(y, x);

				double phase;
				double diff = max - min;

				if (diff <= 1)
					phase = 0;
				else
				{
					double i1f = (i1 - min) / diff;
					double i2f = (i2 - min) / diff;
					double i3f = (i3 - min) / diff;
					double i4f = (i4 - min) / diff;

					phase = std::atan2((i4f - i2f), (i1f - i3f));
				}

				phase_mat.at<double>(y, x) = phase;
			}
		}

		return phase_mat;
	}

	static cv::Mat phaseMatToImage(const cv::Mat& phase_mat)
	{
		cv::Mat gray_image(phase_mat.rows, phase_mat.cols, CV_8UC1);

		for (int y = 0; y < phase_mat.rows; y++)
		{
			for (int x = 0; x < phase_mat.cols; x++)
			{
				auto phase = phase_mat.at<double>(y, x);
				gray_image.at<uint8_t>(y, x) = static_cast<uint8_t>((phase + M_PI) / (2 * M_PI) * 255);
			}
		}

		return gray_image;
	}

	static cv::Mat unwrapPhaseMat(const cv::Mat& phase_mat, const cv::Mat& code_map)
	{
		assert(phase_mat.size() == code_map.size());

		cv::Mat unwrapped_mat(phase_mat.size(), CV_64FC1, cv::Scalar(0));

		for(int y = 0; y < phase_mat.rows; y++)
		{
			for(int x = 0; x < phase_mat.cols; x++)
			{
				unwrapped_mat.at<double>(y, x) = phase_mat.at<double>(y, x) + code_map.at<uint8_t>(y, x) * (2 * M_PI);
			}
		}

		return unwrapped_mat;
	}

	static cv::Mat unwrappedPhaseMatToImage(const cv::Mat& phase_mat, uint8_t periods)
	{
		float min = -M_PI, max = periods * 2 * M_PI + min;

		cv::Mat normalized_mat = (phase_mat - min) / (max - min);
		cv::Mat phase_image = (normalized_mat * 255);
		phase_image.convertTo(phase_image, CV_8UC1);

		return phase_image;
	}
};
