#include <boost/algorithm/string.hpp>

#include "MainNavigationWindow.h"

#include "AbstractCamerasFactory.h"
#include "WebCamerasFactory.h"
#include "PDNCamerasFactory.h"
#include "Logger.hpp"


MainNavigationWindow::MainNavigationWindow(QWidget *parent)
	: QMainWindow(parent)
{
	ui_.setupUi(this);
	connect(ui_.btnSingleCalib, &QPushButton::clicked, this, &MainNavigationWindow::buttonClicked);
	connect(ui_.btnDualViewCalib, &QPushButton::clicked, this, &MainNavigationWindow::buttonClicked);
	//connect(ui_.cbCamCategory, &QComboBox::currentIndexChanged, this, &MainNavigationWindow::cameraCategorySelectionChanged);
	connect(ui_.cbCamCategory, SIGNAL(currentIndexChanged(int)), this, SLOT(cameraCategorySelectionChanged(int)));

	AbstractCamerasFactory::registerFactory( WebCamerasFactory::name, WebCamerasFactory::instance());
	AbstractCamerasFactory::registerFactory( PDNCamerasFactory::name, PDNCamerasFactory::instance());
}


void MainNavigationWindow::showEvent(QShowEvent* e)
{
	QMainWindow::showEvent(e);

	auto factories = AbstractCamerasFactory::getAllFactories();
	for(const auto& pair: factories)
	{
		auto& factory = pair.second;
		std::string camera_category = factory->getName();
		auto ids = factory->enumerateCamerasIDs();
		if(!ids.empty())
		{
			cameras_ids_[camera_category] = ids;
			auto message = boost::format("Find camera ids {%1%} of %2%") % boost::join(ids, ",") % camera_category;
			GET_LOGGER().debug(message.str());
		}
		else
		{
			GET_LOGGER().debug("Cannot find any camera of " + camera_category);
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


void MainNavigationWindow::buttonClicked()
{
	const auto button = sender();
	std::string cam_category = ui_.cbCamCategory->currentText().toStdString();
	std::string cam_id = ui_.cbCamId->currentText().toStdString();

	auto camera = AbstractCamerasFactory::getCameraFactory(cam_category)->createCamera(cam_id);

	calib_window_.reset();

	if(button == ui_.btnSingleCalib)
		calib_window_ = std::make_unique<SingleViewCalibrationWindow>(camera);
	else if(button == ui_.btnDualViewCalib)
		calib_window_ = std::make_unique<DualViewsCalibrationWindow>(camera);
	
	if(calib_window_)
	{
		calib_window_->setWindowModality(Qt::WindowModality::ApplicationModal);
		calib_window_->show();
	}
	else
	{
		throw std::runtime_error("Unrecognized button!");
	}
}


void MainNavigationWindow::cameraCategorySelectionChanged(int index)
{
	auto cbCamCategory = dynamic_cast<QComboBox*>(sender());
	ui_.cbCamId->clear();

	auto selected_category = cbCamCategory->currentText();
	QVector<QString> list;
	for(auto& id: cameras_ids_[selected_category.toStdString()])
		list.push_back(QString::fromStdString(id));

	auto items = QStringList::fromVector(list);
	ui_.cbCamId->addItems(items);
}
