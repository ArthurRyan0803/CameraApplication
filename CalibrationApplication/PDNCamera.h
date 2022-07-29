#pragma once

#include <Windows.h>
#include <string>
#include <vector>
#include <boost/format.hpp>

#include "Camera.h"
#include "CameraApi.h"



class PDNCameraException: public std::exception
{
	std::string func_name;
	int code;
	std::string message;

public:
	PDNCameraException(std::string message, int code, std::string&& file, int line): func_name(std::move(func_name)), code(code)
	{
		message = 
		(
			boost::format("MVCamSDK error, %1%, error code: %2%, file: %3%, line: %4%") % message % code % file % line 
		).str();
		
	}

	char const* what() const override
	{
		return message.c_str();
	}
};


class PDNCamera: public Camera
{

public:
	enum SensorMode {Left, Right, Both};

private:
	const SensorMode sensor_mode_;
	tSdkCameraDevInfo camera_info_;
	CameraHandle cam_handle_;
	volatile bool is_opened_;
	BYTE* isp_buffer_;
	std::function<void(cv::InputArray)> frame_ready_callback_;
	tSdkCameraCapbility capability_{};
	std::array<int, 10> temp2_ {};
	
	//void mvCameraCallback(CameraHandle cam_handle, BYTE *frame_buffer, tSdkFrameHead* frame_head,PVOID context);
	
	static void frameCallback(CameraHandle camera_handle, BYTE *frame_buffer, tSdkFrameHead* frame_head, PVOID context);

public:

	PDNCamera(const tSdkCameraDevInfo& info, SensorMode mode);
	~PDNCamera() override;

	void open()  override;
	void close() override;
	bool isOpened() override;

	void startCapture() override;
	void stopCapture() override;
	bool isCapturing() override;
	void oneShot(cv::OutputArray image) override;

	void showParameterDialog() override;

	std::vector<std::array<int, 2>> enumerateAvailableResolutions() override;
	/*
	 * @return: {width, height}
	 */
	std::array<int, 2> getCurrentResolution() override;
	//void setCurrentResolution(const std::array<int, 2>& resolution) override;
	
	size_t getPixelType() override;

	void setFrameReadyCallback(std::function<void(cv::InputArray)> callback) override;
};
