#include "CalibBoardSettingsDialog.h"


CalibBoardSettingsDialog::CalibBoardSettingsDialog(CalibrationBoardSettings& settings, QWidget *parent)
	: QDialog(parent), settings_(settings)
{
	ui_.setupUi(this);

	connect(ui_.btnDefault, &QPushButton::clicked, this, &CalibBoardSettingsDialog::defaultButtonClicked);
	connect(ui_.btnOK, &QPushButton::clicked, this, &CalibBoardSettingsDialog::accept);
	connect(ui_.btnCancel, &QPushButton::clicked, this, &CalibBoardSettingsDialog::reject);
}

void CalibBoardSettingsDialog::defaultButtonClicked() const
{
	ui_.spHorizontalCount->setValue(static_cast<int>(settings_.horizontal_count));
	ui_.spVerticalCount->setValue(static_cast<int>(settings_.vertical_count));
	ui_.dspInterval->setValue(settings_.interval);
}

void CalibBoardSettingsDialog::okButtonClicked()
{
	settings_.horizontal_count = ui_.spHorizontalCount->value();
	settings_.vertical_count = ui_.spVerticalCount->value();
	settings_.interval = ui_.dspInterval->value();

	accept();
}
