#pragma once

#include <QDialog>
#include "ui_CalibBoardSettingsDialog.h"
#include "CalibrationBoardSettings.hpp"

class CalibBoardSettingsDialog : public QDialog
{
	Q_OBJECT

public:
	CalibBoardSettingsDialog(CalibrationBoardSettings& settings, QWidget *parent = nullptr);
	~CalibBoardSettingsDialog() = default;

private:
	Ui::CalibBoardSettingsDialogClass ui_{};
	CalibrationBoardSettings& settings_;

	void defaultButtonClicked() const;
	void okButtonClicked();
};
