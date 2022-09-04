#pragma once

#include <QDialog>
#include "ui_ParameterDialog.h"
#include "PDRImageCamera.h"

class ParameterDialog : public QDialog
{
	Q_OBJECT

public:
	ParameterDialog(
		std::shared_ptr<CameraLib::PDRImageCamera> left_camera, 
		std::shared_ptr<CameraLib::PDRImageCamera> right_camera, 
		QWidget *parent
	);

	~ParameterDialog() override;

private:

	std::shared_ptr<CameraLib::PDRImageCamera> left_camera_;
	std::shared_ptr<CameraLib::PDRImageCamera> right_camera_;
	std::shared_ptr<CameraLib::PDRImageCamera> selected_camera_;
	
	Ui::ParameterDialogClass ui{};
	CameraLib::PDRImageCamera::SensorType sensor_;

	void updateParametersUi(const std::shared_ptr<CameraLib::PDRImageCamera>& camera) const;
	void setSelectedCamera(const std::shared_ptr<CameraLib::PDRImageCamera>& camera);
	void setCameraExposure(int value, CameraLib::PDRImageCamera::SensorType type) const;
	void setCameraGain(int value, CameraLib::PDRImageCamera::SensorType type) const;
	void turnOnFillLight() const;
	void turnOffFillLight() const;
	void whiteBalance() const;

	void cameraRadioButtonClicked();
	void sliderValueChanged() const;
};
