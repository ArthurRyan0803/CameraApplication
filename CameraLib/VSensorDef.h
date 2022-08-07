#pragma once

#include <string>
#include <boost/format.hpp>


#ifdef _MSC_VER
	#ifdef _WIN64
		#pragma comment(lib, "../SDK/MVCAMSDK_X64.lib")
	#else
		#pragma comment(lib, "../SDK/MVCAMSDK.lib")
	#endif
#else
	#error "Unrecognized compiler!"
#endif


#define THROW_VSENSOR_EXCEPTION(MESSAGE, CODE) throw VSensorException(MESSAGE, CODE, __FILE__, __LINE__)
#define VSENSOR_SDK_TRACK(OP) if(auto CODE = OP; SDK_UNSUCCESS(CODE)) throw VSensorException(#OP, CODE, __FILE__, __LINE__)


class VSensorException : public std::exception
{
	int code_;
	std::string message_;

public:
	VSensorException(std::string message, int code, const std::string& file, int line): code_(code)
	{
		message_ = (boost::format("MVCamSDK error, %1%, error code: %2%, file: %3%, line: %4%") % message % code % file % line).str();
	}

	char const* what() const override
	{
		return message_.c_str();
	}
};
