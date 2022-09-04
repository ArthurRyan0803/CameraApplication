#include <iostream>
#include <opencv2/opencv.hpp>

#include "PIDController.h"
#include "MVImageCamera.h"
#include "MVCamerasFactory.hpp"


using namespace CameraLib;


cv::Mat histogram(cv::Mat image)
{
	cv::Mat gray;

	if(image.channels() == 3)
		cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
	
	int channels = 0;
	int histSize = 256;
	float midRanges[] = { 0, 256 };
	const float *ranges[] = { midRanges };
	cv::MatND hist;

	cv::calcHist(&gray, 1, &channels, cv::Mat(), hist, 1, &histSize, ranges, true, false);

	cv::Mat hist_normalized;
	cv::normalize(hist, hist_normalized);
	return hist_normalized;
}


double histogramMeanSample(cv::Mat histogram)
{
	double sum = cv::sum(histogram)[0];

	double weighted_sum = 0;
	for(size_t i=0; i<histogram.rows; i++)
		weighted_sum += i * histogram.at<float>(i, 0);

	return weighted_sum / sum;
}


double cameraExpMeasure(const std::shared_ptr<MVImageCamera>& camera, double exp_inc)
{
	cv::Mat image;
	double exp;
	camera->getExposure(exp);
	camera->setExposure(exp + exp_inc);
	camera->onceCapture(image);

	auto hist = histogram(image);
	auto measure = histogramMeanSample(hist);
	return measure;
}


int main(int args, char* argv[])
{
	constexpr float kp = 200, ki = 100, kd=0;
	
	try
	{
		double initial_exposure = 60 * 1000;
		int initial_gain = 6;

		auto factory = MVCameraFactory<MVImageCamera>();
		auto ids = factory.enumerateCamerasIDs();
		assert(!ids.empty() && "Cannot find any camera!");

		auto camera = factory.createMVCamera(ids[0]);

		std::cout << "Find a camera: id - " << ids[0] << std::endl;

		camera->open();
		camera->setGain(initial_gain);
		
		cv::Mat image;
		camera->onceCapture(image);
		camera->whiteBalance();

		PIDController controller(kp, ki, kd, -120 * 1000, 120 * 1000);
		double pid_out = 1;

		while(true)
		{
			std::cout << "Please input a set point of exposure(ms)!" << std::endl;
			double set_point;
			std::cin >> set_point;
			std::cout << "set point: " << set_point << std::endl;

			size_t images_count = 0;

			
			while(true)
			{
				camera->onceCapture(image);

				double measure = cameraExpMeasure(camera, pid_out);

				if(std::abs(measure - set_point) < 0.1)
				{
					std::cout << "measure " << measure << " has achieved the set point " << set_point << std::endl;
					std::cout << "Iterations: " << images_count << std::endl;
					break;
				}

				pid_out = controller.update(set_point, measure);
				std::cout << "measure: " << measure << "; pid_out: " << pid_out <<  std::endl;
				images_count++;
			}
		}
	}
	catch(const std::exception& e)
	{
		std::cout << e.what() << std::endl;
	}
}
