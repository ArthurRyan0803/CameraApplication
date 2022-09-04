#ifndef __CALIBRATION_LIB_FRAMEWORK_H__
#define __CALIBRATION_LIB_FRAMEWORK_H__

#include <minwindef.h>

#if defined(WIN32)
    #ifdef CALIBRATIONLIB_EXPORTS
        #define CALIBRATIONLIB_SDK __declspec(dllexport)
    #elif (defined N_CALIBRATIONLIB_EXPORTS)
        #define CALIBRATIONLIB_SDK
    #else
        #define CALIBRATIONLIB_SDK __declspec(dllimport)
    #endif

#else
    #error Unsupported platform
#endif


#endif // __CALIBRATION_LIB_FRAMEWORK_H__
#pragma once
