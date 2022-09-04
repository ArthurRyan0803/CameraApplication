#ifndef _VSENSORDEFINE_H_
#define _VSENSORDEFINE_H_

#define IMAGE_WIDTH     (1280)
#define IMAGE_HEIGHT    (1024)

///�����ؽ�����ṹ��
typedef struct
{
	float pointx[IMAGE_HEIGHT*IMAGE_WIDTH];				///�������x����
	float pointy[IMAGE_HEIGHT*IMAGE_WIDTH];				///�������y����
	float pointz[IMAGE_HEIGHT*IMAGE_WIDTH];				///�������x����
	BYTE pointr[IMAGE_HEIGHT*IMAGE_WIDTH];				///�������r
	BYTE pointg[IMAGE_HEIGHT*IMAGE_WIDTH];				///�������g
	BYTE pointb[IMAGE_HEIGHT*IMAGE_WIDTH];				///�������b
	bool mask[IMAGE_HEIGHT*IMAGE_WIDTH];				///ͼ����ģ
	WORD DepthMap[IMAGE_HEIGHT*IMAGE_WIDTH];            ///���ͼ
	BYTE GrayMap[IMAGE_HEIGHT*IMAGE_WIDTH*2];		    ///�Ҷ�ͼ
	BYTE RGBMap[IMAGE_HEIGHT*IMAGE_WIDTH*3];		    ///��ɫͼ
	int nPointNum;										///��������
	int nFrameNum;                                      ///�ɼ�֡��
}VSensorResult;

///����豸��Ϣ�ṹ��
typedef struct
{
	char CameraName[32];                         ///�豸����
	char Address[32];                            ///�豸��ַ
	UINT uInstance;                              ///�豸����
}VSensorCameraInfo;

///����ڲνṹ��
typedef struct
{
	double LPara[9];                             ///��������� CMOS���������K1 K2 K3 P1 P2  CMOS�ڲ�������fx fy cx cy
	double RPara[9];                             ///��������� CMOS���������K1 K2 K3 P1 P2  CMOS�ڲ�������fx fy cx cy
}VSensorCameraInternelPara;

///������
#define CAMERA_STATUS_SUCCESS                          0    ///< \~chinese �����ɹ� \~english Successful
#define CAMERA_STATUS_FAILED                          -1    ///< \~chinese ����ʧ�� \~english operation failed
#define CAMERA_STATUS_INTERNAL_ERROR                  -2    ///< \~chinese �ڲ����� \~english internal error
#define CAMERA_STATUS_UNKNOW                          -3    ///< \~chinese δ֪���� \~english unknown error
#define CAMERA_STATUS_NOT_SUPPORTED                   -4    ///< \~chinese ��֧�ָù��� \~english Does not support this feature
#define CAMERA_STATUS_NOT_INITIALIZED                 -5    ///< \~chinese ��ʼ��δ��� \~english Incomplete initialization
#define CAMERA_STATUS_PARAMETER_INVALID               -6    ///< \~chinese ������Ч \~english Invalid argument
#define CAMERA_STATUS_PARAMETER_OUT_OF_BOUND          -7    ///< \~chinese ����Խ�� \~english Out of bounds of parameters
#define CAMERA_STATUS_UNENABLED                       -8    ///< \~chinese δʹ�� \~english Not enabled
#define CAMERA_STATUS_USER_CANCEL                     -9    ///< \~chinese �û��ֶ�ȡ���ˣ�����roi�����ȡ�������� \~english The user manually canceled, such as roi panel click cancel, return
#define CAMERA_STATUS_PATH_NOT_FOUND                  -10   ///< \~chinese ע�����û���ҵ���Ӧ��·�� \~english The corresponding path was not found in the registry
#define CAMERA_STATUS_SIZE_DISMATCH                   -11   ///< \~chinese ���ͼ�����ݳ��ȺͶ���ĳߴ粻ƥ�� \~english The length of the obtained image data does not match the defined size
#define CAMERA_STATUS_TIME_OUT                        -12   ///< \~chinese ��ʱ���� \~english Timeout error
#define CAMERA_STATUS_IO_ERROR                        -13   ///< \~chinese Ӳ��IO���� \~english Hardware IO error
#define CAMERA_STATUS_COMM_ERROR                      -14   ///< \~chinese ͨѶ���� \~english Communication error
#define CAMERA_STATUS_BUS_ERROR                       -15   ///< \~chinese ���ߴ��� \~english Bus error
#define CAMERA_STATUS_NO_DEVICE_FOUND                 -16   ///< \~chinese û�з����豸 \~english No device found
#define CAMERA_STATUS_NO_LOGIC_DEVICE_FOUND           -17   ///< \~chinese δ�ҵ��߼��豸 \~english Logical device not found
#define CAMERA_STATUS_DEVICE_IS_OPENED                -18   ///< \~chinese �豸�Ѿ��� \~english The device is already open
#define CAMERA_STATUS_DEVICE_IS_CLOSED                -19   ///< \~chinese �豸�Ѿ��ر� \~english Device is off
#define CAMERA_STATUS_DEVICE_VEDIO_CLOSED             -20   ///< \~chinese û�д��豸��Ƶ������¼����صĺ���ʱ����������Ƶû�д򿪣���ط��ظô��� \~english Without opening the device video, when the video-related function is called, if the camera video is not open, the error is returned back.
#define CAMERA_STATUS_NO_MEMORY                       -21   ///< \~chinese û���㹻ϵͳ�ڴ� \~english Not enough system memory
#define CAMERA_STATUS_FILE_CREATE_FAILED              -22   ///< \~chinese �����ļ�ʧ�� \~english Failed to create file
#define CAMERA_STATUS_FILE_INVALID                    -23   ///< \~chinese �ļ���ʽ��Ч \~english Invalid file format
#define CAMERA_STATUS_WRITE_PROTECTED                 -24   ///< \~chinese д����������д \~english Write protection, not write
#define CAMERA_STATUS_GRAB_FAILED                     -25   ///< \~chinese ���ݲɼ�ʧ�� \~english Data collection failed
#define CAMERA_STATUS_LOST_DATA                       -26   ///< \~chinese ���ݶ�ʧ�������� \~english Loss of data, incomplete
#define CAMERA_STATUS_EOF_ERROR                       -27   ///< \~chinese δ���յ�֡������ \~english No frame terminator received
#define CAMERA_STATUS_BUSY                            -28   ///< \~chinese ��æ(��һ�β������ڽ�����)���˴β������ܽ��� \~english Busy (last operation is still in progress), this operation cannot be performed
#define CAMERA_STATUS_WAIT                            -29   ///< \~chinese ��Ҫ�ȴ�(���в���������������)�������ٴγ��� \~english Need to wait (condition of operation is not established), can try again
#define CAMERA_STATUS_IN_PROCESS                      -30   ///< \~chinese ���ڽ��У��Ѿ��������� \~english Ongoing, has been operated
#define CAMERA_STATUS_IIC_ERROR                       -31   ///< \~chinese IIC������� \~english IIC transmission error
#define CAMERA_STATUS_SPI_ERROR                       -32   ///< \~chinese SPI������� \~english SPI transmission error
#define CAMERA_STATUS_USB_CONTROL_ERROR               -33   ///< \~chinese USB���ƴ������ \~english USB control transmission error
#define CAMERA_STATUS_USB_BULK_ERROR                  -34   ///< \~chinese USB BULK������� \~english USB BULK transmission error
#define CAMERA_STATUS_SOCKET_INIT_ERROR               -35   ///< \~chinese ���紫���׼���ʼ��ʧ�� \~english Network Transport Suite Initialization Failed
#define CAMERA_STATUS_GIGE_FILTER_INIT_ERROR          -36   ///< \~chinese ��������ں˹���������ʼ��ʧ�ܣ������Ƿ���ȷ��װ���������������°�װ�� \~english The webcam kernel filter driver failed to initialize. Please check if the driver is installed correctly or reinstall it.
#define CAMERA_STATUS_NET_SEND_ERROR                  -37   ///< \~chinese �������ݷ��ʹ��� \~english Network data sending error
#define CAMERA_STATUS_DEVICE_LOST                     -38   ///< \~chinese ���������ʧȥ���ӣ�������ⳬʱ \~english Lost connection with webcam, heartbeat timeout
#define CAMERA_STATUS_DATA_RECV_LESS                  -39   ///< \~chinese ���յ����ֽ������������  \~english Received fewer bytes than requested
#define CAMERA_STATUS_FUNCTION_LOAD_FAILED            -40   ///< \~chinese ���ļ��м��س���ʧ�� \~english Failed to load program from file
#define CAMERA_STATUS_CRITICAL_FILE_LOST              -41   ///< \~chinese ����������������ļ���ʧ�� \~english The file necessary to run the program is missing.
#define CAMERA_STATUS_SENSOR_ID_DISMATCH              -42   ///< \~chinese �̼��ͳ���ƥ�䣬ԭ���������˴���Ĺ̼��� \~english The firmware and program do not match because the wrong firmware was downloaded.
#define CAMERA_STATUS_OUT_OF_RANGE                    -43   ///< \~chinese ����������Ч��Χ�� \~english The parameter is out of valid range.
#define CAMERA_STATUS_REGISTRY_ERROR                  -44   ///< \~chinese ��װ����ע����������°�װ���򣬻������а�װĿ¼Setup/Installer.exe \~english Setup registration error. Please reinstall the program, or run the installation directory Setup/Installer.exe
#define CAMERA_STATUS_ACCESS_DENY                     -45   ///< \~chinese ��ֹ���ʡ�ָ������Ѿ�����������ռ��ʱ����������ʸ�������᷵�ظ�״̬��(һ��������ܱ��������ͬʱ����) \~english No Access. When the specified camera has been occupied by another program, it will return to this state if you request to access the camera. (A camera cannot be accessed simultaneously by multiple programs)
#define CAMERA_STATUS_CAMERA_NEED_RESET               -46   ///< \~chinese ��ʾ�����Ҫ��λ���������ʹ�ã���ʱ��������ϵ�������������������ϵͳ�󣬱������ʹ�á� \~english It means that the camera needs to be reset before it can be used normally. At this time, please make the camera power off and restart, or restart the operating system, then it can be used normally.
#define CAMERA_STATUS_ISP_MOUDLE_NOT_INITIALIZED      -47   ///< \~chinese ISPģ��δ��ʼ�� \~english ISP module is not initialized
#define CAMERA_STATUS_ISP_DATA_CRC_ERROR              -48   ///< \~chinese ����У����� \~english Data check error
#define CAMERA_STATUS_MV_TEST_FAILED                  -49   ///< \~chinese ���ݲ���ʧ�� \~english Data test failed
#define CAMERA_STATUS_INTERNAL_ERR1                   -50   ///< \~chinese �ڲ�����1 \~english Internal error 1
#define CAMERA_STATUS_U3V_NO_CONTROL_EP			      -51   ///< \~chinese U3V���ƶ˵�δ�ҵ� \~english U3V control endpoint not found
#define CAMERA_STATUS_U3V_CONTROL_ERROR			      -52   ///< \~chinese U3V����ͨѶ���� \~english U3V control communication error
#define CAMERA_STATUS_INVALID_FRIENDLY_NAME		      -53   ///< \~chinese ��Ч���豸���������ﲻ�ܰ��������ַ�(\/:*?"<>|") \~english Invalid device name, the name cannot contain the following characters (\/:*?"<>|")
#define VSENSOR_STATUS_NO_DEVICE_FOUND				  -100  ///< \~chinese δ�����豸 \~english Device not connected
#define VSENSOR_STATUS_SEND_INSTRUTION_FAILED		  -101  ///< \~chinese ���ڷ���ָ��ʧ�� \~english Serial port sending instruction failed
#define VSENSOR_STATUS_RECEIVE_INSTRUTION_FAILED	  -102  ///< \~chinese ���ڽ���ָ��ʧ�� \~english Serial port receiving instruction failed
#define VSENSOR_STATUS_PARA_RECEIVE_FAILED			  -103  ///< \~chinese ��ȡ�궨�ļ�ʧ�� \~english Failed to open calibration file
#define VSENSOR_STATUS_SERIAL_NUMBER_ERROR            -104  ///< \~chinese �������쳣 \~english Error serial number
#define VSENSOR_STATUS_JUMBOFRAME_CONNECT_FAILED      -105  ///< \~chinese δ�򿪾���֡ \~english Disable jumbo frame
#define VSENSOR_STATUS_GIGE_CONNECT_FAILED            -106  ///< \~chinese ��ǧ����ͨ�� \~english Communication speed is not GigE
#define VSENSOR_STATUS_USB3_CONNECT_FAILED            -107  ///< \~chinese δ����USB3.0�ӿ� \~english USB 3.0 interface not connected
#define VSENSOR_STATUS_PREVIEW_IMAGE_FAILED			  -108  ///< \~chinese Ԥ��ͼ�ɼ�ʧ�� \~english Get preview image buffer failed
#define VSENSOR_STATUS_CAPTURE_IMAGE_FAILED			  -109  ///< \~chinese �ɼ�����ͼʧ�� \~english Capture failed
#define VSENSOR_STATUS_CONTINUOUS_MODE_FAILED		  -110  ///< \~chinese �������ɼ�ģʽ \~english Status is not continueous mode
#define VSENSOR_STATUS_CONTINUOUSCAPTURE_TIMEOUT	  -111  ///< \~chinese �����ɼ���ȡ���Ƴ�ʱ \~english Get point cloud timeout
#define VSENSOR_STATUS_FILE_SAVE_FAILED				  -112  ///< \~chinese �ļ�������� \~english Error saving file
#define VSENSOR_STATUS_FILE_FORMAT_FAILED			  -113  ///< \~chinese �����ļ���ʽ����\~english The format of the file is incorrect
#define VSENSOR_STATUS_PARA_OUTOF_RANGE				  -114  ///< \~chinese �������ڷ�Χ�� \~english Parameter out of range
#define VSENSOR_STATUS_CAPTURING_FAILED				  -115  ///< \~chinese ������ڲɼ� \~english Camera is capturing
#define VSENSOR_STATUS_CAPTUREMODE_FAILED	          -116  ///< \~chinese ����ɼ�ģʽ���� \~english Camera capture mode error
#define VSENSOR_STATUS_FUNCTION_NOTSUPPERT			  -117  ///< \~chinese ���豸��֧�ָù��� \~english This function is not supported
#define VSENSOR_STATUS_DOWNSAMPLING_FAILED			  -118  ///< \~chinese ��������ʧ�ܣ����ȹر�ROIģʽ \~english Failed to open downsampling. Please turn off ROI mode first
#define VSENSOR_STATUS_ROI_FAILED					  -119  ///< \~chinese ROI��ʧ�ܣ����ȹرս�����ģʽ \~english Failed to turn off ROI mode. Please close downsampling first
#define MODULE_STATUS_VOLTAGE_ERROR 			      -200	///< \~chinese �����ѹ�쳣 \~english Module voltage abnormal
#define MODULE_STATUS_NO_PD_ERROR                     -201	///< \~chinese ��PD�ź� \~english No PD signal
#define MODULE_STATUS_PD_ERROR 					      -202	///< \~chinese PD�ź��쳣 \~english PD signal abnormal
#define MODULE_STATUS_OVERHEATING_ERROR 			  -203	///< \~chinese ģ����� \~english Module is overheating
#define MODULE_STATUS_REGISTER12_ERROR 				  -204	///< \~chinese 12�Ĵ����쳣 \~english Register 12 error
#define MODULE_STATUS_REGISTER13_ERROR 				  -205	///< \~chinese 13�Ĵ����쳣 \~english Register 13 error
#define MODULE_STATUS_REGISTER33_ERROR 				  -206	///< \~chinese 33�Ĵ����쳣 \~english Register 33 error
#define MODULE_STATUS_TRIGGER_ERROR 				  -207	///< \~chinese �����źŷ�ֵ�쳣 \~english Trigger voltage is abnormal
#define MODULE_STATUS_NO_TRIGGER_ERROR 				  -208	///< \~chinese ��Ӳ�������ź� \~english None tigger signal
#define MODULE_STATUS_UART_ERROR 					  -209	///< \~chinese UART����ֵ�쳣 \~english UART return value error
#define MODULE_STATUS_LASER_DAMPING_ERROR 			  -210	///< \~chinese ������˥���쳣 \~english Laser damping
#define MODULE_STATUS_MEMS_ERROR 					  -211	///< \~chinese MEMSƵ���쳣 \~english MEMS frequency is abnormal

#endif