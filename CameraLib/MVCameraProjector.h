#pragma once

#include "PDNImageCamera.h"
#include "FrameListener.h"

#include <mutex>
#include <condition_variable>

namespace CameraLib
{
	class CAMERALIB_SDK MVCameraProjector: public FrameListener,  public std::enable_shared_from_this<MVCameraProjector>
	{
	private:
		std::shared_ptr<PDNImageCamera> camera_;
		CameraHandle cam_handle_;

		std::mutex frames__mutex_;
		std::condition_variable frames_cv_;

		volatile uint8_t frames_count_ {0};
		volatile uint8_t rcv_frames_count_ {0};

		void setPatternSwitchInterval(uint16_t ms);

	public:
		MVCameraProjector(std::shared_ptr<PDNImageCamera> camera);
		~MVCameraProjector() override;

		void loadPattern() const;
		void projectPattern(uint8_t index) const;
		void projectPatterns(uint8_t start_index, uint8_t end_index);
		uint8_t getPatternIndex() const;
		void frameReadyCallback(cv::InputArray data) override;

		class ResponseLengthError: std::exception
		{

		public:
			uint8_t rcv_length {0}, right_length {0};

			ResponseLengthError(uint8_t rcv_length, uint8_t right_length);

			char const* what() const override;
		};
	};
}
