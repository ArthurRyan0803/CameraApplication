#pragma once

#include <QDialog>
#include "ui_FunctionalDialog.h"

class FunctionalDialog : public QDialog
{
	Q_OBJECT

public:
	FunctionalDialog(QWidget *parent, const std::function<void()>&& func, const std::string& message);
	~FunctionalDialog() override = default;

private:
	Ui::FunctionalDialogClass ui;
	std::function<void()> func_;

	void showEvent(QShowEvent* event) override;
};
