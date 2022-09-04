#include "MVCameraProjector.h"
#include "MindVision/CameraApi.h"
#include "MVDef.h"


#define CAM_CONTROL_UART_BASE				(0x8000)
#define CAM_CONTROL_UART_TX					(CAM_CONTROL_UART_BASE + 1)
#define CAM_CONTROL_UART_RX					(CAM_CONTROL_UART_BASE + 2)
#define SLEEP_MS 100	
#define SLEEP() std::this_thread::sleep_for(std::chrono::microseconds(SLEEP_MS))

#define SEND_COMMAND(HANDLE, BUFFER, LEN) if(CameraSpecialControl((HANDLE), CAM_CONTROL_UART_TX, (0 << 16) | (LEN), BUFFER) != (LEN)) \
											throw std::runtime_error("Failed to send projection command!")

#define RCV_COMMAND(HANDLE, COUNT, BUFFER, LEN) if(((COUNT) = CameraSpecialControl((HANDLE), CAM_CONTROL_UART_RX, (0 << 16) | (LEN - 1), BUFFER)) <= 0) \
											throw std::runtime_error("Failed to send projection command!")


using namespace CameraLib;

class HardwareTriggerSwitch
{
private:
	std::shared_ptr<PDNImageCamera> camera_;

public:
	explicit HardwareTriggerSwitch(std::shared_ptr<PDNImageCamera> camera): camera_(camera)
	{
		camera_->enableHardwareTrigger();
	}

	~HardwareTriggerSwitch()
	{
		camera_->disableHardwareTrigger();
	}
};

MVCameraProjector::MVCameraProjector(const std::shared_ptr<PDNImageCamera> camera)
{
	camera_ = camera;
	assert(camera->isOpened());

	cam_handle_ = camera_->getCameraHandle();

	std::array<uint8_t, 256> buffer{};
	CameraSpecialControl(cam_handle_, CAM_CONTROL_UART_RX, (0 << 16) | buffer.size() - 1, buffer.data());
}


MVCameraProjector::~MVCameraProjector()
{

}


void CameraLib::MVCameraProjector::setPatternSwitchInterval(uint16_t ms)
{
	uint8_t x1 = *((uint8_t*)&ms + 1), x2 = *((uint8_t*)&ms);
	std::array<uint8_t, 5> buffer{ 0x9f, 0xfd, 0x01, x1, x2};
	SEND_COMMAND(cam_handle_, &buffer, buffer.size());
	SLEEP();

	int rec_count;
	RCV_COMMAND(cam_handle_, rec_count, buffer.data(), buffer.size());

	if(rec_count != 1)
		throw ResponseLengthError(rec_count, 1);

	if(buffer[0] != x1)
		throw std::runtime_error("Failed to set pattern switch interval!");
}


void MVCameraProjector::loadPattern() const
{
	uint8_t cmd = 0x4f;
	SEND_COMMAND(cam_handle_, &cmd, 1);
	SLEEP();
	
	std::array<uint8_t, 10> buffer {};
	int rec_count;

	while(true)
	{
		RCV_COMMAND(cam_handle_, rec_count, buffer.data(), buffer.size());

		if(rec_count == 2)
		{
			if(buffer[0] != 0x4f || buffer[1] != 0xf4)
				throw std::runtime_error("Projector load pattern failed!");

			break;
		}

		if(rec_count == 1)
		{
			if(buffer[0] == 0xff)
			{
				SLEEP();
			}

			if(buffer[0] != 0x4f)
				throw std::runtime_error("Projector load pattern failed!");

			break;
		}
		
		throw ResponseLengthError(rec_count, 1);
	}
}


void MVCameraProjector::projectPattern(uint8_t index) const
{
	std::array<uint8_t, 10> buffer {0x9f, 0xff, index};
	SEND_COMMAND(cam_handle_, buffer.data(), 3);
	SLEEP();

	int rec_count;
	RCV_COMMAND(cam_handle_, rec_count, buffer.data(), 10);

	if(rec_count != 1)
		
	
	if(memcmp(buffer.data(), &index, 1))
		throw std::runtime_error("Failed to set projection pattern index!");
}

void MVCameraProjector::projectPatterns(uint8_t start_index, uint8_t end_index)
{
	double exp_us;
	camera_->getExposure(exp_us);
	uint16_t exp_ms = std::ceil(exp_us / 1000);

	std::shared_ptr<MVCameraProjector> this_ptr(this, [](MVCameraProjector*) {  });
	camera_->registerFrameListener(this_ptr);

	setPatternSwitchInterval(exp_ms * 4);
	HardwareTriggerSwitch trigger_switch(camera_);

	try
	{
		frames_count_ = end_index - start_index + 1;
		rcv_frames_count_ = 0;
		size_t len = frames_count_ + 1;

		std::array<uint8_t, 4> send_buffer {0x9f, 0xf9, start_index, end_index};
		SEND_COMMAND(cam_handle_, send_buffer.data(), 4);

		bool time_out = false;
		std::unique_lock lock(frames__mutex_);
		if(frames_cv_.wait_for(lock, std::chrono::milliseconds(exp_ms * frames_count_ * 100)) == std::cv_status::timeout)
			time_out = true;
		
		std::vector<uint8_t> rec_buffer(len + 1);
		size_t rec_count;
		RCV_COMMAND(cam_handle_, rec_count, rec_buffer.data(), rec_buffer.size());

		if(time_out)
			throw std::runtime_error("Failed to receive enough frames");

		if(rec_count != frames_count_ + 1)
			throw ResponseLengthError(rec_count, frames_count_ + 1);

	}
	catch(const std::exception& e)
	{
		throw e;
	}
}


uint8_t MVCameraProjector::getPatternIndex() const
{
	std::array<uint8_t, 3> buffer {0x9f, 0xfc, 0x02};
	SEND_COMMAND(cam_handle_, buffer.data(), 3);
	SLEEP();
	uint8_t rec_count;
	RCV_COMMAND(cam_handle_, rec_count, buffer.data(), 3);
	if(rec_count != 1)
		throw ResponseLengthError(rec_count, 1);

	return buffer[0];
}


void MVCameraProjector::frameReadyCallback(cv::InputArray data)
{
	std::unique_lock lock(frames__mutex_);
	rcv_frames_count_ += 1;
	if(rcv_frames_count_ == frames_count_)
	{
		rcv_frames_count_ = 0;
		frames_cv_.notify_one();
	}
}

MVCameraProjector::ResponseLengthError::ResponseLengthError(uint8_t rcv_length, uint8_t right_length): rcv_length(rcv_length), right_length(right_length)
{

}

char const* MVCameraProjector::ResponseLengthError::what() const
{
	throw std::runtime_error("Supposed to receive " + std::to_string(right_length) + " byte from projector but " + std::to_string(rcv_length) + " bytes were received!");
}
