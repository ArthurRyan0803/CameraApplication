#include "ParameterDialog.h"


using namespace CameraLib;


void updateValueController(QSpinBox* spinbox, QSlider* slider, double value, double min, double max, double nominator=1)
{
	int value_int = static_cast<int>(value / nominator),
	min_int = static_cast<int>(min / nominator),
	max_int = static_cast<int>(max / nominator);
	
	spinbox->setValue(value_int);
	spinbox->setMinimum(min_int);
	spinbox->setMaximum(max_int);

	slider->setValue(value_int);
	slider->setMinimum(min_int);
	slider->setMaximum(max_int);
}


ParameterDialog::ParameterDialog(std::shared_ptr<Camera> left_camera, std::shared_ptr<Camera> right_camera, QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	connect(ui.rbLeftCamera, &QRadioButton::toggled, this, &ParameterDialog::cameraRadioButtonClicked);
	connect(ui.rbRightCamera, &QRadioButton::toggled, this, &ParameterDialog::cameraRadioButtonClicked);

	// Exposure
	connect(ui.sliderGrayExposure, &QSlider::valueChanged, this, &ParameterDialog::sliderValueChanged);
	connect(ui.sliderColorExposure, &QSlider::valueChanged, this, &ParameterDialog::sliderValueChanged);
    connect(ui.sbGrayExposure, SIGNAL(valueChanged(int)), ui.sliderGrayExposure, SLOT(setValue(int)));
    connect(ui.sbColorExposure, SIGNAL(valueChanged(int)), ui.sliderColorExposure, SLOT(setValue(int)));

	// Gain
	connect(ui.sliderGrayGain, &QSlider::valueChanged, this, &ParameterDialog::sliderValueChanged);
	connect(ui.sliderColorGain, &QSlider::valueChanged, this, &ParameterDialog::sliderValueChanged);
    connect(ui.sbGrayGain, SIGNAL(valueChanged(int)), ui.sliderGrayGain, SLOT(setValue(int)));
    connect(ui.sbColorGain, SIGNAL(valueChanged(int)), ui.sliderColorGain, SLOT(setValue(int)));

	connect(ui.btnFillLightOn, &QPushButton::clicked, this, &ParameterDialog::turnOffFillLight);
	connect(ui.btnFillLightOff, &QPushButton::clicked, this, &ParameterDialog::turnOnFillLight);
	connect(ui.btnWhiteBalance, &QPushButton::clicked, this, &ParameterDialog::whiteBalance);
	
	left_camera_ = left_camera;
	right_camera_ = right_camera;

	ui.rbLeftCamera->setChecked(true);
}

ParameterDialog::~ParameterDialog() = default;

void ParameterDialog::updateParametersUi(const std::shared_ptr<Camera>& camera) const
{
	// Exposure
	// Exposure Gray
	double exposure = 0, exp_min = 0, exp_max = 0, exp_step = 0;
	camera->getExposure(exposure, Camera::Gray);
	camera->getExposureRange(exp_min, exp_max, exp_step, Camera::Gray);
	updateValueController(ui.sbGrayExposure, ui.sliderGrayExposure, exposure, exp_min, exp_max, 1000);
	
	// Exposure Color
	camera->getExposure(exposure, Camera::Color);
	camera->getExposureRange(exp_min, exp_max, exp_step, Camera::Gray);
	updateValueController(ui.sbColorExposure, ui.sliderColorExposure, exposure, exp_min, exp_max, 1000);
	
	// Gain
	// Gain gray
	int gain = 0, gain_min = 0, gain_max = 0, gain_step = 0;
	camera->getAnalogGain(gain, Camera::Gray);
	camera->getAnalogGainRange(gain_min, gain_max, gain_step, Camera::Gray);
	updateValueController(ui.sbGrayGain, ui.sliderGrayGain, gain, gain_min, gain_max);

	// Gain color
	camera->getAnalogGain(gain, Camera::Color);
	camera->getAnalogGainRange(gain_min, gain_max, gain_step, Camera::Color);
	updateValueController(ui.sbColorGain, ui.sliderColorGain, gain, gain_min, gain_max);
}

void ParameterDialog::setSelectedCamera(const std::shared_ptr<Camera>& camera)
{
	selected_camera_ = camera;
	updateParametersUi(selected_camera_);
}

void ParameterDialog::setCameraExposure(int value, Camera::SensorType type) const
{
	selected_camera_->setExposure(value * 1000, type);
}

void ParameterDialog::setCameraGain(int value, Camera::SensorType type) const
{
	selected_camera_->setAnalogGain(value, type);
}

void ParameterDialog::turnOnFillLight() const
{
	selected_camera_->turnOnFillLight();
}

void ParameterDialog::turnOffFillLight() const
{
	selected_camera_->turnOffFillLight();
}

void ParameterDialog::whiteBalance() const
{
	selected_camera_->whiteBalance();
}

void ParameterDialog::cameraRadioButtonClicked()
{
	auto obj = dynamic_cast<QRadioButton*>(sender());
	assert(obj);

	if(!obj->isChecked())
		return;

	if(obj == ui.rbLeftCamera)
		setSelectedCamera(left_camera_);
	else
		setSelectedCamera(right_camera_);
}

void ParameterDialog::sliderValueChanged() const
{
	auto slider = dynamic_cast<QSlider*>(sender());
	assert(slider);

	auto value = slider->value();

	if(slider == ui.sliderGrayExposure)
	{
		ui.sbGrayExposure->setValue(value);
		setCameraExposure(value, Camera::Gray);
	}
	else if(slider == ui.sliderColorExposure)
	{
		ui.sbColorExposure->setValue(value);
		setCameraExposure(value, Camera::Color);
	}
	else if(slider == ui.sliderGrayGain)
	{
		ui.sbGrayGain->setValue(value);
		setCameraGain(value, Camera::Gray);
	}
	else if(slider == ui.sliderColorGain)
	{
		ui.sbColorGain->setValue(value);
		setCameraGain(value, Camera::Color);
	}
}
