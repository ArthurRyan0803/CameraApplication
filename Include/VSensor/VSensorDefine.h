#ifndef _VSENSORDEFINE_H_
#define _VSENSORDEFINE_H_

#define IMAGE_WIDTH     (1280)
#define IMAGE_HEIGHT    (1024)

///点云重建结果结构体
typedef struct
{
	float pointx[IMAGE_HEIGHT*IMAGE_WIDTH];				///有序点云x坐标
	float pointy[IMAGE_HEIGHT*IMAGE_WIDTH];				///有序点云y坐标
	float pointz[IMAGE_HEIGHT*IMAGE_WIDTH];				///有序点云x坐标
	BYTE pointr[IMAGE_HEIGHT*IMAGE_WIDTH];				///有序点云r
	BYTE pointg[IMAGE_HEIGHT*IMAGE_WIDTH];				///有序点云g
	BYTE pointb[IMAGE_HEIGHT*IMAGE_WIDTH];				///有序点云b
	bool mask[IMAGE_HEIGHT*IMAGE_WIDTH];				///图像掩模
	WORD DepthMap[IMAGE_HEIGHT*IMAGE_WIDTH];            ///深度图
	BYTE GrayMap[IMAGE_HEIGHT*IMAGE_WIDTH*2];		    ///灰度图
	BYTE RGBMap[IMAGE_HEIGHT*IMAGE_WIDTH*3];		    ///彩色图
	int nPointNum;										///点云数量
	int nFrameNum;                                      ///采集帧数
}VSensorResult;

///相机设备信息结构体
typedef struct
{
	char CameraName[32];                         ///设备名称
	char Address[32];                            ///设备地址
	UINT uInstance;                              ///设备索引
}VSensorCameraInfo;

///相机内参结构体
typedef struct
{
	double LPara[9];                             ///左相机参数 CMOS畸变参数：K1 K2 K3 P1 P2  CMOS内部参数：fx fy cx cy
	double RPara[9];                             ///右相机参数 CMOS畸变参数：K1 K2 K3 P1 P2  CMOS内部参数：fx fy cx cy
}VSensorCameraInternelPara;

///错误码
#define CAMERA_STATUS_SUCCESS                          0    ///< \~chinese 操作成功 \~english Successful
#define CAMERA_STATUS_FAILED                          -1    ///< \~chinese 操作失败 \~english operation failed
#define CAMERA_STATUS_INTERNAL_ERROR                  -2    ///< \~chinese 内部错误 \~english internal error
#define CAMERA_STATUS_UNKNOW                          -3    ///< \~chinese 未知错误 \~english unknown error
#define CAMERA_STATUS_NOT_SUPPORTED                   -4    ///< \~chinese 不支持该功能 \~english Does not support this feature
#define CAMERA_STATUS_NOT_INITIALIZED                 -5    ///< \~chinese 初始化未完成 \~english Incomplete initialization
#define CAMERA_STATUS_PARAMETER_INVALID               -6    ///< \~chinese 参数无效 \~english Invalid argument
#define CAMERA_STATUS_PARAMETER_OUT_OF_BOUND          -7    ///< \~chinese 参数越界 \~english Out of bounds of parameters
#define CAMERA_STATUS_UNENABLED                       -8    ///< \~chinese 未使能 \~english Not enabled
#define CAMERA_STATUS_USER_CANCEL                     -9    ///< \~chinese 用户手动取消了，比如roi面板点击取消，返回 \~english The user manually canceled, such as roi panel click cancel, return
#define CAMERA_STATUS_PATH_NOT_FOUND                  -10   ///< \~chinese 注册表中没有找到对应的路径 \~english The corresponding path was not found in the registry
#define CAMERA_STATUS_SIZE_DISMATCH                   -11   ///< \~chinese 获得图像数据长度和定义的尺寸不匹配 \~english The length of the obtained image data does not match the defined size
#define CAMERA_STATUS_TIME_OUT                        -12   ///< \~chinese 超时错误 \~english Timeout error
#define CAMERA_STATUS_IO_ERROR                        -13   ///< \~chinese 硬件IO错误 \~english Hardware IO error
#define CAMERA_STATUS_COMM_ERROR                      -14   ///< \~chinese 通讯错误 \~english Communication error
#define CAMERA_STATUS_BUS_ERROR                       -15   ///< \~chinese 总线错误 \~english Bus error
#define CAMERA_STATUS_NO_DEVICE_FOUND                 -16   ///< \~chinese 没有发现设备 \~english No device found
#define CAMERA_STATUS_NO_LOGIC_DEVICE_FOUND           -17   ///< \~chinese 未找到逻辑设备 \~english Logical device not found
#define CAMERA_STATUS_DEVICE_IS_OPENED                -18   ///< \~chinese 设备已经打开 \~english The device is already open
#define CAMERA_STATUS_DEVICE_IS_CLOSED                -19   ///< \~chinese 设备已经关闭 \~english Device is off
#define CAMERA_STATUS_DEVICE_VEDIO_CLOSED             -20   ///< \~chinese 没有打开设备视频，调用录像相关的函数时，如果相机视频没有打开，则回返回该错误。 \~english Without opening the device video, when the video-related function is called, if the camera video is not open, the error is returned back.
#define CAMERA_STATUS_NO_MEMORY                       -21   ///< \~chinese 没有足够系统内存 \~english Not enough system memory
#define CAMERA_STATUS_FILE_CREATE_FAILED              -22   ///< \~chinese 创建文件失败 \~english Failed to create file
#define CAMERA_STATUS_FILE_INVALID                    -23   ///< \~chinese 文件格式无效 \~english Invalid file format
#define CAMERA_STATUS_WRITE_PROTECTED                 -24   ///< \~chinese 写保护，不可写 \~english Write protection, not write
#define CAMERA_STATUS_GRAB_FAILED                     -25   ///< \~chinese 数据采集失败 \~english Data collection failed
#define CAMERA_STATUS_LOST_DATA                       -26   ///< \~chinese 数据丢失，不完整 \~english Loss of data, incomplete
#define CAMERA_STATUS_EOF_ERROR                       -27   ///< \~chinese 未接收到帧结束符 \~english No frame terminator received
#define CAMERA_STATUS_BUSY                            -28   ///< \~chinese 正忙(上一次操作还在进行中)，此次操作不能进行 \~english Busy (last operation is still in progress), this operation cannot be performed
#define CAMERA_STATUS_WAIT                            -29   ///< \~chinese 需要等待(进行操作的条件不成立)，可以再次尝试 \~english Need to wait (condition of operation is not established), can try again
#define CAMERA_STATUS_IN_PROCESS                      -30   ///< \~chinese 正在进行，已经被操作过 \~english Ongoing, has been operated
#define CAMERA_STATUS_IIC_ERROR                       -31   ///< \~chinese IIC传输错误 \~english IIC transmission error
#define CAMERA_STATUS_SPI_ERROR                       -32   ///< \~chinese SPI传输错误 \~english SPI transmission error
#define CAMERA_STATUS_USB_CONTROL_ERROR               -33   ///< \~chinese USB控制传输错误 \~english USB control transmission error
#define CAMERA_STATUS_USB_BULK_ERROR                  -34   ///< \~chinese USB BULK传输错误 \~english USB BULK transmission error
#define CAMERA_STATUS_SOCKET_INIT_ERROR               -35   ///< \~chinese 网络传输套件初始化失败 \~english Network Transport Suite Initialization Failed
#define CAMERA_STATUS_GIGE_FILTER_INIT_ERROR          -36   ///< \~chinese 网络相机内核过滤驱动初始化失败，请检查是否正确安装了驱动，或者重新安装。 \~english The webcam kernel filter driver failed to initialize. Please check if the driver is installed correctly or reinstall it.
#define CAMERA_STATUS_NET_SEND_ERROR                  -37   ///< \~chinese 网络数据发送错误 \~english Network data sending error
#define CAMERA_STATUS_DEVICE_LOST                     -38   ///< \~chinese 与网络相机失去连接，心跳检测超时 \~english Lost connection with webcam, heartbeat timeout
#define CAMERA_STATUS_DATA_RECV_LESS                  -39   ///< \~chinese 接收到的字节数比请求的少  \~english Received fewer bytes than requested
#define CAMERA_STATUS_FUNCTION_LOAD_FAILED            -40   ///< \~chinese 从文件中加载程序失败 \~english Failed to load program from file
#define CAMERA_STATUS_CRITICAL_FILE_LOST              -41   ///< \~chinese 程序运行所必须的文件丢失。 \~english The file necessary to run the program is missing.
#define CAMERA_STATUS_SENSOR_ID_DISMATCH              -42   ///< \~chinese 固件和程序不匹配，原因是下载了错误的固件。 \~english The firmware and program do not match because the wrong firmware was downloaded.
#define CAMERA_STATUS_OUT_OF_RANGE                    -43   ///< \~chinese 参数超出有效范围。 \~english The parameter is out of valid range.
#define CAMERA_STATUS_REGISTRY_ERROR                  -44   ///< \~chinese 安装程序注册错误。请重新安装程序，或者运行安装目录Setup/Installer.exe \~english Setup registration error. Please reinstall the program, or run the installation directory Setup/Installer.exe
#define CAMERA_STATUS_ACCESS_DENY                     -45   ///< \~chinese 禁止访问。指定相机已经被其他程序占用时，再申请访问该相机，会返回该状态。(一个相机不能被多个程序同时访问) \~english No Access. When the specified camera has been occupied by another program, it will return to this state if you request to access the camera. (A camera cannot be accessed simultaneously by multiple programs)
#define CAMERA_STATUS_CAMERA_NEED_RESET               -46   ///< \~chinese 表示相机需要复位后才能正常使用，此时请让相机断电重启，或者重启操作系统后，便可正常使用。 \~english It means that the camera needs to be reset before it can be used normally. At this time, please make the camera power off and restart, or restart the operating system, then it can be used normally.
#define CAMERA_STATUS_ISP_MOUDLE_NOT_INITIALIZED      -47   ///< \~chinese ISP模块未初始化 \~english ISP module is not initialized
#define CAMERA_STATUS_ISP_DATA_CRC_ERROR              -48   ///< \~chinese 数据校验错误 \~english Data check error
#define CAMERA_STATUS_MV_TEST_FAILED                  -49   ///< \~chinese 数据测试失败 \~english Data test failed
#define CAMERA_STATUS_INTERNAL_ERR1                   -50   ///< \~chinese 内部错误1 \~english Internal error 1
#define CAMERA_STATUS_U3V_NO_CONTROL_EP			      -51   ///< \~chinese U3V控制端点未找到 \~english U3V control endpoint not found
#define CAMERA_STATUS_U3V_CONTROL_ERROR			      -52   ///< \~chinese U3V控制通讯错误 \~english U3V control communication error
#define CAMERA_STATUS_INVALID_FRIENDLY_NAME		      -53   ///< \~chinese 无效的设备名，名字里不能包含以下字符(\/:*?"<>|") \~english Invalid device name, the name cannot contain the following characters (\/:*?"<>|")
#define VSENSOR_STATUS_NO_DEVICE_FOUND				  -100  ///< \~chinese 未连接设备 \~english Device not connected
#define VSENSOR_STATUS_SEND_INSTRUTION_FAILED		  -101  ///< \~chinese 串口发送指令失败 \~english Serial port sending instruction failed
#define VSENSOR_STATUS_RECEIVE_INSTRUTION_FAILED	  -102  ///< \~chinese 串口接收指令失败 \~english Serial port receiving instruction failed
#define VSENSOR_STATUS_PARA_RECEIVE_FAILED			  -103  ///< \~chinese 读取标定文件失败 \~english Failed to open calibration file
#define VSENSOR_STATUS_SERIAL_NUMBER_ERROR            -104  ///< \~chinese 相机编号异常 \~english Error serial number
#define VSENSOR_STATUS_JUMBOFRAME_CONNECT_FAILED      -105  ///< \~chinese 未打开巨型帧 \~english Disable jumbo frame
#define VSENSOR_STATUS_GIGE_CONNECT_FAILED            -106  ///< \~chinese 非千兆网通信 \~english Communication speed is not GigE
#define VSENSOR_STATUS_USB3_CONNECT_FAILED            -107  ///< \~chinese 未连接USB3.0接口 \~english USB 3.0 interface not connected
#define VSENSOR_STATUS_PREVIEW_IMAGE_FAILED			  -108  ///< \~chinese 预览图采集失败 \~english Get preview image buffer failed
#define VSENSOR_STATUS_CAPTURE_IMAGE_FAILED			  -109  ///< \~chinese 采集条纹图失败 \~english Capture failed
#define VSENSOR_STATUS_CONTINUOUS_MODE_FAILED		  -110  ///< \~chinese 非连续采集模式 \~english Status is not continueous mode
#define VSENSOR_STATUS_CONTINUOUSCAPTURE_TIMEOUT	  -111  ///< \~chinese 连续采集获取点云超时 \~english Get point cloud timeout
#define VSENSOR_STATUS_FILE_SAVE_FAILED				  -112  ///< \~chinese 文件保存出错 \~english Error saving file
#define VSENSOR_STATUS_FILE_FORMAT_FAILED			  -113  ///< \~chinese 保存文件格式有误\~english The format of the file is incorrect
#define VSENSOR_STATUS_PARA_OUTOF_RANGE				  -114  ///< \~chinese 参数不在范围内 \~english Parameter out of range
#define VSENSOR_STATUS_CAPTURING_FAILED				  -115  ///< \~chinese 相机正在采集 \~english Camera is capturing
#define VSENSOR_STATUS_CAPTUREMODE_FAILED	          -116  ///< \~chinese 相机采集模式有误 \~english Camera capture mode error
#define VSENSOR_STATUS_FUNCTION_NOTSUPPERT			  -117  ///< \~chinese 该设备不支持该功能 \~english This function is not supported
#define VSENSOR_STATUS_DOWNSAMPLING_FAILED			  -118  ///< \~chinese 降采样打开失败，请先关闭ROI模式 \~english Failed to open downsampling. Please turn off ROI mode first
#define VSENSOR_STATUS_ROI_FAILED					  -119  ///< \~chinese ROI打开失败，请先关闭降采样模式 \~english Failed to turn off ROI mode. Please close downsampling first
#define MODULE_STATUS_VOLTAGE_ERROR 			      -200	///< \~chinese 供电电压异常 \~english Module voltage abnormal
#define MODULE_STATUS_NO_PD_ERROR                     -201	///< \~chinese 无PD信号 \~english No PD signal
#define MODULE_STATUS_PD_ERROR 					      -202	///< \~chinese PD信号异常 \~english PD signal abnormal
#define MODULE_STATUS_OVERHEATING_ERROR 			  -203	///< \~chinese 模组过热 \~english Module is overheating
#define MODULE_STATUS_REGISTER12_ERROR 				  -204	///< \~chinese 12寄存器异常 \~english Register 12 error
#define MODULE_STATUS_REGISTER13_ERROR 				  -205	///< \~chinese 13寄存器异常 \~english Register 13 error
#define MODULE_STATUS_REGISTER33_ERROR 				  -206	///< \~chinese 33寄存器异常 \~english Register 33 error
#define MODULE_STATUS_TRIGGER_ERROR 				  -207	///< \~chinese 触发信号幅值异常 \~english Trigger voltage is abnormal
#define MODULE_STATUS_NO_TRIGGER_ERROR 				  -208	///< \~chinese 无硬件触发信号 \~english None tigger signal
#define MODULE_STATUS_UART_ERROR 					  -209	///< \~chinese UART返回值异常 \~english UART return value error
#define MODULE_STATUS_LASER_DAMPING_ERROR 			  -210	///< \~chinese 激光器衰减异常 \~english Laser damping
#define MODULE_STATUS_MEMS_ERROR 					  -211	///< \~chinese MEMS频率异常 \~english MEMS frequency is abnormal

#endif