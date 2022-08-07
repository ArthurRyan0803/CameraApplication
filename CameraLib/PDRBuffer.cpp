#include "PDRBuffer.h"
#include "CameraApi.h"


using namespace CameraLib;


PDRBuffer::PDRBuffer(const cv::Size& image_size)
{
	int gray_size = image_size.width * image_size.height;

	left = std::shared_ptr<BYTE>(CameraAlignMalloc(gray_size, 4), CameraAlignFree);
	right = std::shared_ptr<BYTE>(CameraAlignMalloc(gray_size, 4), CameraAlignFree);

	left_isp = std::shared_ptr<BYTE>(CameraAlignMalloc(gray_size, 4), CameraAlignFree);
	right_isp = std::shared_ptr<BYTE>(CameraAlignMalloc(gray_size, 4), CameraAlignFree);
	rgb_isp = std::shared_ptr<BYTE>(CameraAlignMalloc(gray_size * 3, 4), CameraAlignFree);
}
