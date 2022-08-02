#include "CalibrationSettingDialog.h"


CalibBoardSettingsDialog::CalibBoardSettingsDialog(CalibrationBoardSettings& settings, QWidget *parent)
	: QDialog(parent), settings_(settings)
{
	ui_.setupUi(this);

	connect(ui_.btnRestore, &QPushButton::clicked, this, &CalibBoardSettingsDialog::restoreButtonClicked);
	connect(ui_.btnOK, &QPushButton::clicked, this, &CalibBoardSettingsDialog::okButtonClicked);
	connect(ui_.btnCancel, &QPushButton::clicked, this, &CalibBoardSettingsDialog::reject);

	restore();
}

void CalibBoardSettingsDialog::restore() const
{
	ui_.spHorizontalCount->setValue(static_cast<int>(settings_.horizontal_count));
	ui_.spVerticalCount->setValue(static_cast<int>(settings_.vertical_count));
	ui_.dspInterval->setValue(settings_.interval);
	ui_.spCalibImagesCount->setValue(settings_.images_count);
}

void CalibBoardSettingsDialog::restoreButtonClicked() const
{
	restore();
}

void CalibBoardSettingsDialog::okButtonClicked()
{
	settings_.horizontal_count = ui_.spHorizontalCount->value();
	settings_.vertical_count = ui_.spVerticalCount->value();
	settings_.interval = ui_.dspInterval->value();
	settings_.images_count = ui_.spCalibImagesCount->value();

	accept();
}
