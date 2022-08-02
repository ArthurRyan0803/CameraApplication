#pragma once

#include <QMainWindow>
#include <memory>

#include "AbstractCamerasFactory.h"
#include "ui_MainNavigationWindow.h"
#include "SingleViewCalibrationWindow.h"
#include "DualViewsCalibrationWindow.h"


class MainNavigationWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainNavigationWindow(QWidget *parent = nullptr);
	~MainNavigationWindow() override = default;

private:
	std::map<std::string, std::vector<std::string>> cameras_ids_;

	Ui::MainNavigationWindowClass ui_;

	std::unique_ptr<QMainWindow> calib_window_;

	void showEvent(QShowEvent* event) override;
	void buttonClicked();
	void cameraCategorySelectionChanged();
};
