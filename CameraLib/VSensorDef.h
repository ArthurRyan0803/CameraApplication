#pragma once

#include <string>
#include <boost/format.hpp>

namespace CameraLib
{
#define THROW_VSENSOR_SDK_EXCEPTION(MESSAGE, CODE) throw VSensorSDKException(MESSAGE, CODE, __FILE__, __LINE__)
// Check return value of vsensor camera sdk
#define VSENSOR_SDK_CHECK(OP) if(auto CODE = OP; CODE != 0) throw VSensorSDKException(#OP, CODE, __FILE__, __LINE__)

#define ONE_SHOT_WAIT_MS 1000
#define ISP_BUFFER_ALIGNMENT 16
#define IMAGE_VERTICAL_FLIP_FLAG 1
#define IMAGE_HORIZONTAL_FLIP_FLAG 2
#define IMAGE_ALL_FLIP_FLAG 3
#define VSENSOR_FRAME_WIDTH 1280
#define VSENSOR_FRAME_HEIGHT 1024

	class VSensorSDKException : public std::exception
	{
		int code_;
		std::string message_;

	public:
		VSensorSDKException(std::string message, int code, const std::string& file, int line): code_(code)
		{
			message_ = (boost::format("VSensor SDK error, %1%, error code: %2%, file: %3%, line: %4%") % message % code % file % line).str();
		}

		char const* what() const override
		{
			return message_.c_str();
		}
	};
}
