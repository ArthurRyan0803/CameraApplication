#include <boost/algorithm/string.hpp>

#include "MainNavigationWindow.h"

#include "AbstractCamerasFactory.h"
#include "PDNCamerasFactory.h"
#include "WebCamerasFactory.h"
#include "Logger.hpp"


#define WEB_CAM "WebCamera"
#define PDN_CAM "PDNCamera"


std::map<std::string, std::shared_ptr<CameraLib::AbstractCamerasFactory>> factories;


MainNavigationWindow::MainNavigationWindow(QWidget *parent)
	: QMainWindow(parent), logger_(GET_LOGGER())
{
	ui_.setupUi(this);
	connect(ui_.btnSingleCalib, &QPushButton::clicked, this, &MainNavigationWindow::buttonClicked);
	connect(ui_.btnDualViewCalib, &QPushButton::clicked, this, &MainNavigationWindow::buttonClicked);
	connect(ui_.cbCamCategory, &QComboBox::currentTextChanged, this, &MainNavigationWindow::cameraCategorySelectionChanged);

	factories[WEB_CAM] = std::make_shared<CameraLib::WebCamerasFactory>();
	factories[PDN_CAM] = std::make_shared<CameraLib::PDNCamerasFactory>();
}


void MainNavigationWindow::showEvent(QShowEvent* e)
{
	QMainWindow::showEvent(e);
	
	for(const auto& pair: factories)
	{
		auto& factory = pair.second;
		std::string camera_category = pair.first;
		auto ids = factory->enumerateCamerasIDs();
		if(!ids.empty())
		{
			cameras_ids_[camera_category] = ids;
			auto message = boost::format("Find camera ids {%1%} of %2%") % boost::join(ids, ",") % camera_category;
			logger_.debug(message.str());
		}
		else
		{
			logger_.debug("Cannot find any camera of " + camera_category);
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

	auto camera = factories[cam_category]->createCamera(cam_id);

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


void MainNavigationWindow::cameraCategorySelectionChanged()
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
