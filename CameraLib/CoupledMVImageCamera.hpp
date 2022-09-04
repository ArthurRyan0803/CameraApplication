#pragma once

#include "MVImageCamera.h"

namespace CameraLib
{
	class CAMERALIB_SDK CoupledMVImageCamera : public MVImageCamera
	{
	public:
		CoupledMVImageCamera(const tSdkCameraDevInfo& info): MVImageCamera(info) {}
	};
}
