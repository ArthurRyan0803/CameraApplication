#include <boost/algorithm/string.hpp>

#include "MainNavigationWindow.h"
#include "SingleCameraCalibrationDialog.h"
#include "AbstractCamerasFactory.h"
#include "Utils.hpp"
#include "WebCamerasFactory.h"
#include "Logger.hpp"


MainNavigationWindow::MainNavigationWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui_.setupUi(this);
	connect(ui_.btnSingleCalib, &QPushButton::clicked, this, &MainNavigationWindow::buttonClicked);
	connect(ui_.cbCamCategory, &QComboBox::currentIndexChanged, this, &MainNavigationWindow::cameraCategorySelectionChanged);

	AbstractCamerasFactory::registerFactory( WebCamerasFactory::name, WebCamerasFactory::instance());
}


void MainNavigationWindow::showEvent(QShowEvent* e)
{
	QMainWindow::showEvent(e);

	auto factories = AbstractCamerasFactory::getAllFactories();
	for(const auto& pair: factories)
	{
		auto& factory = pair.second;
		std::string camera_category = factory->getName();
		auto ids = factory->enumerateCameraIDs();
		if(!ids.empty())
		{
			cameras_ids_[camera_category] = ids;
			auto& message = boost::format("Find camera ids {%1%} of %2%") % boost::join(ids, ",") % camera_category;
			Logger::log(message.str());
		}
		else
		{
			Logger::log("Cannot find any camera of " + camera_category);
		}
	}

	if(cameras_ids_.empty())
	{
		ui_.gbCalibration->setEnabled(false);
		return;
	}

	ui_.cbCamCategory->clear();
	ui_.cbCamId->clear();

	for(auto& pair: cameras_ids_)
	{
		ui_.cbCamCategory->addItem(QString::fromStdString(pair.first));
	}
}


void MainNavigationWindow::buttonClicked() const
{
	const auto button = sender();
	std::string cam_category = ui_.cbCamCategory->currentText().toStdString();
	std::string cam_id = ui_.cbCamId->currentText().toStdString();

	auto camera = AbstractCamerasFactory::getCameraFactory(cam_category)->createCamera(cam_id);

	if(button == ui_.btnSingleCalib)
	{
		SingleCameraCalibrationDialog dialog(camera);
		dialog.exec();
	}
	else
	{
		throw std::exception("Unrecognized button!");
	}
}


void MainNavigationWindow::cameraCategorySelectionChanged()
{
	auto cbCamCategory = dynamic_cast<QComboBox*>(sender());
	ui_.cbCamId->clear();

	auto selected_category = cbCamCategory->currentText();
	QList<QString> list;
	for(auto& id: cameras_ids_[selected_category.toStdString()])
	{
		list.append(QString::fromStdString(id));
	}
	auto items = QStringList::fromList(list);
	ui_.cbCamId->addItems(items);
}
