#include <Logger.hpp>
#include "VSensorDef.h"

namespace CameraLib
{
#define THROW_MV_SDK_EXCEPTION(MESSAGE, CODE) throw MVSDKException(MESSAGE, CODE, __FILE__, __LINE__)
// Check return value of mind vision camera sdk
#define MV_SDK_CHECK(OP) if(auto CODE = OP; SDK_UNSUCCESS(CODE)) throw MVSDKException(#OP, CODE, __FILE__, __LINE__)

#define MV_FRAME_WIDTH 1280
#define MV_FRAME_HEIGHT 1024

	class MVSDKException : public std::exception
	{
		int code_;
		std::string message_;

	public:
		MVSDKException(std::string message, int code, const std::string& file, int line): code_(code)
		{
			message_ = (boost::format("MVCamSDK error, %1%, error code: %2%, file: %3%, line: %4%") % message % code % file % line).str();
		}

		char const* what() const override
		{
			return message_.c_str();
		}
	};
}
