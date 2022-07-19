#pragma once

#include <QMainWindow>

#include "AbstractCamerasFactory.h"
#include "ui_MainNavigationWindow.h"

class MainNavigationWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainNavigationWindow(QWidget *parent = nullptr);
	~MainNavigationWindow() override = default;

private:
	std::map<std::string, std::vector<std::string>> cameras_ids_;

	Ui::MainNavigationWindowClass ui_;

	void showEvent(QShowEvent* event) override;
	void buttonClicked() const;
	void cameraCategorySelectionChanged();
};
