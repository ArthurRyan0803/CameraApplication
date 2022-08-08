#ifndef _VSENSORDEFINE_H_
#define _VSENSORDEFINE_H_

#define IMAGE_WIDTH  (1280)
#define IMAGE_HEIGHT (1024)

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
	unsigned short DepthMap[IMAGE_HEIGHT*IMAGE_WIDTH];  ///深度图
	BYTE GrayMap[IMAGE_HEIGHT*IMAGE_WIDTH*2];		    ///灰度图
	BYTE RGBMap[IMAGE_HEIGHT*IMAGE_WIDTH*3];		    ///彩色图
	int pointnum;										///点云数量
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

//错误码
#define CAMERA_STATUS_SUCCESS                      0   ///< \~chinese 操作成功 \~english Successful
#define CAMERA_STATUS_FAILED                      -1   ///< \~chinese 操作失败 \~english operation failed
#define CAMERA_STATUS_INTERNAL_ERROR              -2   ///< \~chinese 内部错误 \~english internal error
#define CAMERA_STATUS_UNKNOW                      -3   ///< \~chinese 未知错误 \~english unknown error
#define CAMERA_STATUS_NOT_SUPPORTED               -4   ///< \~chinese 不支持该功能 \~english Does not support this feature
#define CAMERA_STATUS_NOT_INITIALIZED             -5   ///< \~chinese 初始化未完成 \~english Incomplete initialization
#define CAMERA_STATUS_PARAMETER_INVALID           -6   ///< \~chinese 参数无效 \~english Invalid argument
#define CAMERA_STATUS_PARAMETER_OUT_OF_BOUND      -7   ///< \~chinese 参数越界 \~english Out of bounds of parameters
#define CAMERA_STATUS_UNENABLED                   -8   ///< \~chinese 未使能 \~english Not enabled
#define CAMERA_STATUS_USER_CANCEL                 -9   ///< \~chinese 用户手动取消了，比如roi面板点击取消，返回 \~english The user manually canceled, such as roi panel click cancel, return
#define CAMERA_STATUS_PATH_NOT_FOUND              -10  ///< \~chinese 注册表中没有找到对应的路径 \~english The corresponding path was not found in the registry
#define CAMERA_STATUS_SIZE_DISMATCH               -11  ///< \~chinese 获得图像数据长度和定义的尺寸不匹配 \~english The length of the obtained image data does not match the defined size
#define CAMERA_STATUS_TIME_OUT                    -12  ///< \~chinese 超时错误 \~english Timeout error
#define CAMERA_STATUS_IO_ERROR                    -13  ///< \~chinese 硬件IO错误 \~english Hardware IO error
#define CAMERA_STATUS_COMM_ERROR                  -14  ///< \~chinese 通讯错误 \~english Communication error
#define CAMERA_STATUS_BUS_ERROR                   -15  ///< \~chinese 总线错误 \~english Bus error
#define CAMERA_STATUS_NO_DEVICE_FOUND             -16  ///< \~chinese 没有发现设备 \~english No device found
#define CAMERA_STATUS_NO_LOGIC_DEVICE_FOUND       -17  ///< \~chinese 未找到逻辑设备 \~english Logical device not found
#define CAMERA_STATUS_DEVICE_IS_OPENED            -18  ///< \~chinese 设备已经打开 \~english The device is already open
#define CAMERA_STATUS_DEVICE_IS_CLOSED            -19  ///< \~chinese 设备已经关闭 \~english Device is off
#define CAMERA_STATUS_DEVICE_VEDIO_CLOSED         -20  ///< \~chinese 没有打开设备视频，调用录像相关的函数时，如果相机视频没有打开，则回返回该错误。 \~english Without opening the device video, when the video-related function is called, if the camera video is not open, the error is returned back.
#define CAMERA_STATUS_NO_MEMORY                   -21  ///< \~chinese 没有足够系统内存 \~english Not enough system memory
#define CAMERA_STATUS_FILE_CREATE_FAILED          -22  ///< \~chinese 创建文件失败 \~english Failed to create file
#define CAMERA_STATUS_FILE_INVALID                -23  ///< \~chinese 文件格式无效 \~english Invalid file format
#define CAMERA_STATUS_WRITE_PROTECTED             -24  ///< \~chinese 写保护，不可写 \~english Write protection, not write
#define CAMERA_STATUS_GRAB_FAILED                 -25  ///< \~chinese 数据采集失败 \~english Data collection failed
#define CAMERA_STATUS_LOST_DATA                   -26  ///< \~chinese 数据丢失，不完整 \~english Loss of data, incomplete
#define CAMERA_STATUS_EOF_ERROR                   -27  ///< \~chinese 未接收到帧结束符 \~english No frame terminator received
#define CAMERA_STATUS_BUSY                        -28  ///< \~chinese 正忙(上一次操作还在进行中)，此次操作不能进行 \~english Busy (last operation is still in progress), this operation cannot be performed
#define CAMERA_STATUS_WAIT                        -29  ///< \~chinese 需要等待(进行操作的条件不成立)，可以再次尝试 \~english Need to wait (condition of operation is not established), can try again
#define CAMERA_STATUS_IN_PROCESS                  -30  ///< \~chinese 正在进行，已经被操作过 \~english Ongoing, has been operated
#define CAMERA_STATUS_IIC_ERROR                   -31  ///< \~chinese IIC传输错误 \~english IIC transmission error
#define CAMERA_STATUS_SPI_ERROR                   -32  ///< \~chinese SPI传输错误 \~english SPI transmission error
#define CAMERA_STATUS_USB_CONTROL_ERROR           -33  ///< \~chinese USB控制传输错误 \~english USB control transmission error
#define CAMERA_STATUS_USB_BULK_ERROR              -34  ///< \~chinese USB BULK传输错误 \~english USB BULK transmission error
#define CAMERA_STATUS_SOCKET_INIT_ERROR           -35  ///< \~chinese 网络传输套件初始化失败 \~english Network Transport Suite Initialization Failed
#define CAMERA_STATUS_GIGE_FILTER_INIT_ERROR      -36  ///< \~chinese 网络相机内核过滤驱动初始化失败，请检查是否正确安装了驱动，或者重新安装。 \~english The webcam kernel filter driver failed to initialize. Please check if the driver is installed correctly or reinstall it.
#define CAMERA_STATUS_NET_SEND_ERROR              -37  ///< \~chinese 网络数据发送错误 \~english Network data sending error
#define CAMERA_STATUS_DEVICE_LOST                 -38  ///< \~chinese 与网络相机失去连接，心跳检测超时 \~english Lost connection with webcam, heartbeat timeout
#define CAMERA_STATUS_DATA_RECV_LESS              -39  ///< \~chinese 接收到的字节数比请求的少  \~english Received fewer bytes than requested
#define CAMERA_STATUS_FUNCTION_LOAD_FAILED        -40  ///< \~chinese 从文件中加载程序失败 \~english Failed to load program from file
#define CAMERA_STATUS_CRITICAL_FILE_LOST          -41  ///< \~chinese 程序运行所必须的文件丢失。 \~english The file necessary to run the program is missing.
#define CAMERA_STATUS_SENSOR_ID_DISMATCH          -42  ///< \~chinese 固件和程序不匹配，原因是下载了错误的固件。 \~english The firmware and program do not match because the wrong firmware was downloaded.
#define CAMERA_STATUS_OUT_OF_RANGE                -43  ///< \~chinese 参数超出有效范围。 \~english The parameter is out of valid range.
#define CAMERA_STATUS_REGISTRY_ERROR              -44  ///< \~chinese 安装程序注册错误。请重新安装程序，或者运行安装目录Setup/Installer.exe \~english Setup registration error. Please reinstall the program, or run the installation directory Setup/Installer.exe
#define CAMERA_STATUS_ACCESS_DENY                 -45  ///< \~chinese 禁止访问。指定相机已经被其他程序占用时，再申请访问该相机，会返回该状态。(一个相机不能被多个程序同时访问) \~english No Access. When the specified camera has been occupied by another program, it will return to this state if you request to access the camera. (A camera cannot be accessed simultaneously by multiple programs)
#define CAMERA_STATUS_CAMERA_NEED_RESET           -46  ///< \~chinese 表示相机需要复位后才能正常使用，此时请让相机断电重启，或者重启操作系统后，便可正常使用。 \~english It means that the camera needs to be reset before it can be used normally. At this time, please make the camera power off and restart, or restart the operating system, then it can be used normally.
#define CAMERA_STATUS_ISP_MOUDLE_NOT_INITIALIZED  -47  ///< \~chinese ISP模块未初始化 \~english ISP module is not initialized
#define CAMERA_STATUS_ISP_DATA_CRC_ERROR          -48  ///< \~chinese 数据校验错误 \~english Data check error
#define CAMERA_STATUS_MV_TEST_FAILED              -49  ///< \~chinese 数据测试失败 \~english Data test failed
#define CAMERA_STATUS_INTERNAL_ERR1               -50  ///< \~chinese 内部错误1 \~english Internal error 1
#define CAMERA_STATUS_U3V_NO_CONTROL_EP			  -51  ///< \~chinese U3V控制端点未找到 \~english U3V control endpoint not found
#define CAMERA_STATUS_U3V_CONTROL_ERROR			  -52  ///< \~chinese U3V控制通讯错误 \~english U3V control communication error
#define CAMERA_STATUS_INVALID_FRIENDLY_NAME		  -53  ///< \~chinese 无效的设备名，名字里不能包含以下字符(\/:*?"<>|") \~english Invalid device name, the name cannot contain the following characters (\/:*?"<>|")
#define CAMERA_STATUS_SEND_INSTRUTION_FAILED      -54  ///< \~chinese 串口发送指令失败 \~english Serial port sending instruction failed
#define CAMERA_STATUS_PARAFILE_FAILED             -55  ///< \~chinese 标定文件打开失败 \~english Failed to open calibration file
#define CAMERA_STATUS_CAPTURE_IMAGE_FAILED        -56  ///< \~chinese 图像采集失败 \~english Image acquisition failed
#define CAMERA_STATUS_CONTINUOUS_MODE_FAILED      -57  ///< \~chinese 非连续采集模式 \~english Discontinuous acquisition mode
#define CAMERA_STATUS_CAPTURE_TIME_OUT            -58  ///< \~chinese 获取点云超时 \~english Get point cloud timeout
#define CAMERA_STATUS_NOT_USB3_PORT               -59  ///< \~chinese 未连接USB3.0接口 \~english USB 3.0 interface not connected
#define CAMERA_STATUS_NOT_CONNECT_DEVICE          -60  ///< \~chinese 未连接设备 \~english Device not connected
#define CAMERA_STATUS_COMMANDSOCKET_SEND_FAILED   -61  ///< \~chinese 网口发送指令失败 \~english Network port sending instruction failed
#define CAMERA_STATUS_INIT_FAILED                 -62  ///< \~chinese 初始化失败 \~english Initialization failed
#define CAMERA_STATUS_EXPTIME_OUTRANGE_FAILED     -63  ///< \~chinese 曝光时间设置超出范围 \~english Exposure time setting out of range
#define CAMERA_STATUS_FILE_SAVE_FAILED            -64  ///< \~chinese 点云文件保存出错 \~english Error saving point cloud file
#define CAMERA_STATUS_FILE_FORMAT_FAILED          -65  ///< \~chinese 点云文件格式有误，请使用ply、pcd、txt保存！\~english The format of the point cloud file is incorrect. Please save it with ply, pcd and txt
#define CAMERA_STATUS_PARA_RECEIVE_FAILED         -66  ///< \~chinese 参数接收有误 \~english Incorrect parameter reception
#define CAMERA_STATUS_WINSOCK_INIT_FAILED         -67  ///< \~chinese winsock初始化失败 \~english Winsock initialization failed
#define CAMERA_STATUS_SOCKET_INIT_FAILED          -68  ///< \~chinese 套接字初始化失败 \~english Socket initialization failed
#define CAMERA_STATUS_SETSOCKET_FAILED            -69  ///< \~chinese 设置套接字失败 \~english Failed to set socket
#define CAMERA_STATUS_SOCKET_NONBLOCKING_FAILED   -70  ///< \~chinese 套接字非阻塞态失败 \~english Socket non blocking failure
#define CAMERA_STATUS_BIND_PORT_FAILED            -71  ///< \~chinese 绑定端口失败 \~english Binding port failed
#define CAMERA_STATUS_DEVICE_CONNECTED            -72  ///< \~chinese 设备已连接 \~english Device connected
#define CAMERA_STATUS_PARA_OUTOF_RANGE            -73  ///< \~chinese 参数不在范围内 \~english Parameter out of range
#define CAMERA_STATUS_SET_EXPTIMEMODE_FAILED      -74  ///< \~chinese 设置曝光模式失败 \~english Failed to set exposure mode
#define CAMERA_STATUS_SET_CAPTUREMODE_FAILED      -75  ///< \~chinese 设置采集模式失败 \~english Failed to set collection mode
#define CAMERA_STATUS_SET_ITER_FAILED             -76  ///< \~chinese 设置迭代滤波失败 \~english Failed to set iterative filtering
#define CAMERA_STATUS_SET_TARGET_FAILED           -77  ///< \~chinese 设置目标亮度失败 \~english Failed to set target brightness
#define CAMERA_STATUS_LASER_OPEN_FAILED           -78  ///< \~chinese 激光器打开失败 \~english Laser on failed
#define CAMERA_STATUS_LASER_CLOSE_FAILED          -79  ///< \~chinese 激光器关闭失败 \~english Laser shutdown failed
#define CAMERA_STATUS_POINT_ZERO                  -80  ///< \~chinese 点云数量为0 \~english The number of point clouds is 0
#define CAMERA_STATUS_MINIMUM_OVER_MAXIMUM_ZERO   -81  ///< \~chinese 输入最小值超过最大值 \~english The minimum value entered exceeds the maximum value
#define CAMERA_STATUS_CAMEAR_CAPTURING            -82  ///< \~chinese 相机正在采集 \~english Camera is capturing
#define CAMERA_STATUS_CAMEAR_CAPTUREMODE          -83  ///< \~chinese 相机采集模式有误 \~english Camera capture mode error
#define CAMERA_STATUS_FUNCTION_NOTSUPPERT         -84  ///< \~chinese 不支持该功能 \~english This function is not supported
#define CAMERA_STATUS_SETCONNECT_FAILED           -85  ///< \~chinese 网络建立连接失败 \~english Network connection establishment failed
#define CAMERA_STATUS_GIGE_CONNECT_FAILED         -86  ///< \~chinese 连接失败请更换千兆网 \~english Connection failed, please replace Gigabit Network
#define CAMERA_STATUS_GRATINGIMAGE_FAILED         -87  ///< \~chinese 光栅图采集失败 \~english Raster acquisition failed
#define CAMERA_STATUS_DOWNSAMPLING_FAILED         -88  ///< \~chinese 降采样打开失败，请先关闭ROI模式 \~english Failed to open downsampling. Please turn off ROI mode first
#define CAMERA_STATUS_ROI_FAILED                  -89  ///< \~chinese ROI打开失败，请先关闭降采样模式 \~english Failed to turn off ROI mode. Please close downsampling first

#endif