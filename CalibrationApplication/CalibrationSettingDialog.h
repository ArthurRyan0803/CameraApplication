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
	Ui::CalibrationSettingDialogClass ui_{};
	CalibrationBoardSettings& settings_;

	void restore() const;

	void restoreButtonClicked() const;
	void okButtonClicked();
};
