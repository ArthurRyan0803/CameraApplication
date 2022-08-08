#ifndef _VSENSOR_H_
#define _VSENSOR_H_
#define DLL_EXPORT
#ifdef DLL_EXPORT
#define VSENSOR_API  __declspec(dllexport)
#else
#define VSENSOR_API  __declspec(dllimport)
#endif
#include <atlstr.h>
#include <string>
#include <memory>
#include "VSensorDefine.h"

namespace VSENSOR
{
	extern "C" class VSENSOR_API VSensor
	{
	public:
		VSensor();
		~VSensor();
		/*******************************************************************
		*名称：              GetDeviceList
		*功能：              获取参数列表
		*入口参数：
		*pCameraList         获取的相机列表信息
		*PINums              找到相机的个数
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int GetDeviceList(VSensorCameraInfo* pCameraList, int* piNums);
		/*******************************************************************
		*名称：              DeviceConnect
		*功能：              设备连接，读取标定文件
		*入口参数：
		iDeviceIndex         需要连接的相机索引
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int DeviceConnect(const int& iDeviceIndex);
		/*******************************************************************
		*名称：              DeviceParameterInit
		*功能：              设备参数初始化，仅上电时需要进行此操作，初始化时间40s
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int DeviceParameterInit();
		/*******************************************************************
		*名称：              GetCamInternelParameter
		*功能：              获取相机内参
		*入口参数：
		*pCameraInternelPara 获取的相机内参
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int GetCamInternelParameter(VSensorCameraInternelPara* pCameraInternelPara);
		/*******************************************************************
		*名称：              SetZaxisRange
		*功能：              设置当前Z轴范围
		*入口参数：
		Min					 最小值
		Max					 最大值
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetZaxisRange(const int& Min, const int& Max);
		/*******************************************************************
		*名称：              SetProjectLight
		*功能：              设置投影亮度(此接口仅支持DLP2000系列3D相机)
		*入口参数：
		brightness           投影亮度，亮度范围1 - 100
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetProjectLight(const int& light);
		/*******************************************************************
		*名称：              SetDownsampling
		*功能：              设置降采样
		*入口参数：
		isOpen				 开关，未调用该函数默认关闭
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetDownsampling(const bool& isOpen);
		/*******************************************************************
		*名称：              GetExposureTime
		*功能：              获取当前曝光时间,单位为ms
		*入口参数：
		ExposureTime         获取的曝光时间
		flag                 0:设置灰度相机  1:设置彩色相机 2:设置所有相机，默认设置所有相机
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int GetExposureTime(double& ExposureTime, int flag = 2);
		/*******************************************************************
		*名称：              SetExposureTime
		*功能：              设置曝光时间，单位为ms，曝光范围1 - 200ms,注：P100系列 曝光范围0.1 - 2ms
		*入口参数：
		ExposureTime         设置的曝光时间
		flag                 0:设置灰度相机  1:设置彩色相机 2:设置所有相机，默认设置所有相机
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetExposureTime(const double& ExposureTime, int flag = 2);
		/*******************************************************************
		*名称：              SetAutoExposureTime
		*功能：              设置自动曝光，曝光范围1 - 150ms
		*入口参数：
		isOpen               自动曝光开关，未调用该函数默认关闭
		TargetLight          设置目标亮度，范围10 - 250
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetAutoExposureTime(const bool& isOpen, const int& TargetLight);
		/*******************************************************************
		*名称：              SetHDR
		*功能：              设置高动态采集模式
		*入口参数：
		ExposureTime1		 设置曝光时间1，单位为ms，曝光范围1 - 100ms,注：P100系列 曝光范围0.1 - 2ms
		ExposureTime2        设置曝光时间2，单位为ms，曝光范围1 - 100ms,注：P100系列 曝光范围0.1 - 2ms
		isOpen			     ROI开关，未调用该函数默认关闭
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetHDR(const double& ExposureTime1, const double& ExposureTime2, const bool& isOpen);
		/*******************************************************************
		*名称：              SetCameraOnceWB
		*功能：              设置彩色相机一键白平衡
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetCameraOnceWB();
		/*******************************************************************
		*名称：              SetAnalogGain
		*功能：              设置彩色相机的图像模拟增益值
		*入口参数：
		iAnalogGainR         红色通道增益值，范围0 - 255，默认100
		iAnalogGainG         绿色通道增益值，范围0 - 255，默认100
		iAnalogGainB         蓝色通道增益值，范围0 - 255，默认100
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetAnalogGain(const int& iAnalogGainR, const int& iAnalogGainG, const int& iAnalogGainB);
		/*******************************************************************
		*名称：              GetAnalogGain
		*功能：              获取彩色相机的图像模拟增益值
		*入口参数：
		iAnalogGainR         红色通道增益值
		iAnalogGainG         绿色通道增益值
		iAnalogGainB         蓝色通道增益值
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int GetAnalogGain(int& iAnalogGainR, int& iAnalogGainG, int& iAnalogGainB);
		/*******************************************************************
		*名称：              SetAnalogGain
		*功能：              设置相机的图像模拟增益值
		*入口参数：
		iAnalogGain          增益值，范围3 - 10，默认6
		flag                 0:设置灰度相机  1:设置彩色相机 2:设置所有相机，默认设置所有相机
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetAnalogGain(const int& iAnalogGain, int flag = 2);
		/*****************************************************************
		*名称：              GetLuminance1
		*功能：              获取环境亮度
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int GetLuminance1(int& Luminance);
		/*****************************************************************
		*名称：              GetLuminance2
		*功能：              获取环境亮度
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int GetLuminance2(int& Luminance);
		/*******************************************************************
		*名称：              SetOutlierDetect
		*功能：              设置离散点
		*入口参数：
		window               离散窗口，范围1 - 5
		para				 离散参数，范围1.0 - 5.0
		isOpen               离散开关，未调用该函数默认关闭
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetOutlierDetect(const int& window, const float& para, const bool& isOpen);
		/*******************************************************************
		*名称：              SetFilter1
		*功能：              设置双边滤波
		*入口参数：
		window               滤波窗口，范围1 - 15
		para				 滤波参数，范围1.0 - 10.0
		isOpen               滤波开关，未调用该函数默认关闭
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetFilter1(const int& window, const float& para, const bool& isOpen);
		/*******************************************************************
		*名称：              SetFilter2
		*功能：              设置迭代滤波
		*入口参数：
		para				 滤波参数，范围1.0 - 10.0
		frequency			 滤波次数，范围1 - 5
		isOpen               滤波开关，未调用该函数默认关闭
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetFilter2(const float& para, const int& frequency, const bool& isOpen);
		/*******************************************************************
		*名称：              SetFilter3
		*功能：              设置高斯滤波
		*入口参数：
		para				 滤波参数，范围1.0 - 10.0
		isOpen               滤波开关，未调用该函数默认关闭
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetFilter3(const float& para, const bool& isOpen);
		/*******************************************************************
		*名称：              SetROI
		*功能：              设置ROI感兴趣区域
		*入口参数：
		offsetX		         起始坐标X，范围0 - 880 
		offSetY              起始坐标Y，范围0 - 624
		width			     区域宽，范围400 - 1280, 仅支持16倍数
		height				 区域高，范围400 - 1024，仅支持4倍数
		isOpen			     ROI开关，未调用该函数默认关闭
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetROI(const int& offsetX, const int& offSetY, const int& width, const int& height, const bool& isOpen);
		/*******************************************************************
		*名称：              SetCaptureMode
		*功能：              设置相机采集模式
		*入口参数：
		flag                 0:预览模式  1:采集点云模式，默认为预览模式
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SetCaptureMode(const int& flag);
		/*******************************************************************
		*名称：              SingleRestruction
		*功能：              单次重建3D点云
		*入口参数：
		presult              获取的点云数据结果
		Capflag              0:全量输出  1:输出点云  2:输出点云和深度图  3:输出点云和灰度图
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SingleRestruction(VSensorResult* presult, const int& Capflag);
		/*******************************************************************
		*名称：              ContinuousRestructionStart
		Capflag              0:全量输出  1:输出点云  2:输出点云和深度图  3:输出点云和灰度图
		*功能：              开始3D重建连续模式
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int ContinuousRestructionStart(const int& Capflag);
		/*******************************************************************
		*名称：              ContinuousRestructionStop
		*功能：              结束3D重建连续模式
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int ContinuousRestructionStop();
		/*******************************************************************
		*名称：              CaptureFrame
		*功能：              获取一帧点云数据
		*入口参数：
		presult              获取的点云数据结果
		ctime                最大延时时间，单位ms
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int CaptureFrame(VSensorResult* presult, const int& ctime);
		/*******************************************************************
		*名称：              SaveGrayMap
		*功能：              获取灰度图
		*入口参数：
		*presult             生成的点云结果
		SaveName             保存灰度图文件路径，支持格式png、jpg、bmp
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SaveGrayMap(const std::string& filePath, VSensorResult* presult);
		/*******************************************************************
		*名称：              SaveDepthMap
		*功能：              获取深度图
		*入口参数：
		*presult             生成点云结果
		SaveName             保存深度图文件路径，仅支持png
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SaveDepthMap(const std::string&  filePath, VSensorResult* presult);
		/*******************************************************************
		*名称：              Save3DCloud
		*功能：              保存点云
		*入口参数：
		*presult             生成点云结果
		SaveName             保存点云文件路径,支持格式pcd、ply、txt
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int Save3DCloud(const std::string& filePath, VSensorResult* presult);
		/*******************************************************************
		*名称：              SaveRGBMap
		*功能：              获取彩色图(此接口仅支持RGB系列3D相机)
		*入口参数：
		*presult             生成点云结果
		SaveName             保存彩色图文件路径，支持格式png、jpg、bmp
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int SaveRGBMap(const std::string& filePath, VSensorResult* presult);
		/*******************************************************************
		*名称：              CameraOut
		*功能：              获取相机采集
		*入口参数：
		*imgBufferGray		 存储采集灰度相机采集到的数据，分辨率1280*1024，ROI下为设置的分辨率
		*imgBufferRGB        存储采集彩色相机采集到的数据，分辨率1280*1024
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int CameraOut(BYTE* imgBufferGray, BYTE* imgBufferRGB);
		/*******************************************************************
		*名称：              DeviceUnInit
		*功能：              设备反初始化
		*出口参数：          成功返回0.否则返回非0错误码，参考VSensorDefine.h中错误码的定义
		*******************************************************************/
		int DeviceUnInit();
	public:
		class VSensorSDK;
		std::unique_ptr<VSensorSDK> pVSensorSDK;
	};
}
#endif

