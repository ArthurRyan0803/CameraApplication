#pragma once

#include <QDialog>
#include <Camera.hpp>

#include "ui_ParameterSettingDialog.h"


class ParameterSettingDialog : public QDialog
{
	Q_OBJECT

public:
	ParameterSettingDialog(std::shared_ptr<CameraLib::Camera> camera, QWidget *parent=nullptr);

	~ParameterSettingDialog() override;

private:
	std::shared_ptr<CameraLib::Camera> camera_;

	Ui::ParameterDialogClass ui{};

	void updateParametersUi(const std::shared_ptr<CameraLib::Camera>& camera) const;
	void whiteBalance() const;
	void sliderValueChanged() const;
};
