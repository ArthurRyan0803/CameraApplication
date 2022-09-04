#include "ParameterSettingDialog.h"


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


ParameterSettingDialog::ParameterSettingDialog(std::shared_ptr<Camera> camera, QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	// Exposure
	connect(ui.sliderExposure, &QSlider::valueChanged, this, &ParameterSettingDialog::sliderValueChanged);
    connect(ui.sbExposure, SIGNAL(valueChanged(int)), ui.sliderExposure, SLOT(setValue(int)));

	// Gain
	connect(ui.sliderGain, &QSlider::valueChanged, this, &ParameterSettingDialog::sliderValueChanged);
    connect(ui.sbGain, SIGNAL(valueChanged(int)), ui.sliderGain, SLOT(setValue(int)));

	connect(ui.btnWhiteBalance, &QPushButton::clicked, this, &ParameterSettingDialog::whiteBalance);
	
	camera_ = camera;

	updateParametersUi(camera);
}

ParameterSettingDialog::~ParameterSettingDialog() = default;

void ParameterSettingDialog::updateParametersUi(const std::shared_ptr<Camera>& camera) const
{
	// Exposure
	double exposure = 0, exp_min = 0, exp_max = 0, exp_step = 0;
	camera->getExposure(exposure);
	camera->getExposureRange(exp_min, exp_max, exp_step);
	updateValueController(ui.sbExposure, ui.sliderExposure, exposure, exp_min, exp_max, 1000);
	
	// Gain
	int gain = 0, gain_min = 0, gain_max = 0, gain_step = 0;
	camera->getGain(gain);
	camera->getGainRange(gain_min, gain_max, gain_step);
	updateValueController(ui.sbGain, ui.sliderGain, gain, gain_min, gain_max);
}

void ParameterSettingDialog::whiteBalance() const
{
	camera_->whiteBalance();
}

void ParameterSettingDialog::sliderValueChanged() const
{
	auto slider = dynamic_cast<QSlider*>(sender());
	assert(slider);

	auto value = slider->value();

	if(slider == ui.sliderExposure)
	{
		ui.sbExposure->setValue(value);
		camera_->setExposure(value * 1000);
	}
	else if(slider == ui.sliderGain)
	{
		ui.sbGain->setValue(value);
		camera_->setGain(value);
	}
}
