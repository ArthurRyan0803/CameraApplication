#ifndef __CAMERA_LIB_FRAMEWORK_H__
#define __CAMERA_LIB_FRAMEWORK_H__

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers

// std
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <functional>
#include <thread>
#include <array>

// os
#ifdef _MSC_VER
#include <Windows.h>
#else
#error "Unsupported paltform!"
#endif


// 3rd libs
#include <boost/noncopyable.hpp>
#include <boost/core/noncopyable.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>


#if defined(WIN32)
    #ifdef CAMERALIB_EXPORTS
        #define CAMERALIB_SDK __declspec(dllexport)
    #elif (defined N_CAMERA_LIB_DLLEXPORT)
        #define CAMERALIB_SDK
    #else
        #define CAMERALIB_SDK __declspec(dllimport)
    #endif

#else
    #error Unsupported platform
#endif


#ifdef _MSC_VER
	#ifdef _WIN64
		#pragma comment(lib, "../SDK/MindVision/Release_x64/MVCAMSDK_X64.lib")
		#pragma comment(lib, "../SDK/VSensor/Release_x64/VSensorSDK.lib")
	#else
		#pragma comment(lib, "../SDK/MindVision/Release_x86/MVCAMSDK.lib")
		#pragma comment(lib, "../SDK/VSensor/Release_x86/VSensorSDK.lib")
	#endif
#else
	#error "Unrecognized compiler!"
#endif


#endif // __CAMERA_LIB_FRAMEWORK_H__
