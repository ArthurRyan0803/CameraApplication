#include <opencv2/imgproc.hpp>				 // Do not swap #include
#include <opencv2/imgcodecs.hpp>

#include <QPainter>
#include <QFileDialog>
#include <QMessageBox>

#include "AbstractCalibrationWindow.h"
#include "FunctionalDialog.h"
#include "CalibrationMethods.h"
#include "CalibrationSettingDialog.h"
#include "CalibrationParams.hpp"
#include "UICommon.h"
#include "Utils.hpp"
#include "Logger.hpp"


using namespace CameraLib;
static Logger& logger_ = GET_LOGGER();


AbstractCalibrationWindow::AbstractCalibrationWindow(
	const std::shared_ptr<Camera>& camera,
	QWidget *parent
)
	: QMainWindow(parent),  camera_(camera), is_calibrating_(false)
{
	ui_.setupUi(this);
	
	connect(ui_.btnOneShot, &QPushButton::clicked, this, &AbstractCalibrationWindow::oneShotButtonClicked);
	connect(ui_.btnCapture, &QPushButton::clicked, this, &AbstractCalibrationWindow::captureButtonClicked);
	connect(ui_.cmbCalibPattern, &QComboBox::currentTextChanged, this, &AbstractCalibrationWindow::calibPatternChanged);
	connect(ui_.ckbDetectCalibBoard, &QCheckBox::stateChanged, this, &AbstractCalibrationWindow::detectBoardCheckboxStateChanged);
	connect(ui_.btnSaveImage, &QPushButton::clicked, this, &AbstractCalibrationWindow::saveImageButtonClicked);
	connect(ui_.btnParameter, &QPushButton::clicked, this, &AbstractCalibrationWindow::parameterButtonClicked);
	connect(ui_.btnCalibBoardSetting, &QPushButton::clicked, this, &AbstractCalibrationWindow::calibBoardSettingsButtonClicked);

	connect(ui_.btnCalibration, &QPushButton::clicked, this, &AbstractCalibrationWindow::calibrationButtonClicked);
	connect(ui_.btnGrabCalibImage, &QPushButton::clicked, this, &AbstractCalibrationWindow::grabCalibImageButtonClicked);
	connect(ui_.btnFinshGrabCalibImages, &QPushButton::clicked, this, &AbstractCalibrationWindow::finishCalibrationImageGrabbing);

	for (auto& pair : UICommon::string_to_calib_pattern_map)
		ui_.cmbCalibPattern->addItem(QString::fromStdString(pair.first));
	
	camera->setFrameReadyCallback([this](cv::InputArray data) { cameraFrameReadyCallback(data); });
	camera->open();
	cam_resolution_ = camera->getCurrentResolution();
}


void AbstractCalibrationWindow::closeEvent(QCloseEvent* e)
{
	QMainWindow::closeEvent(e);
	camera_->close();
}


void AbstractCalibrationWindow::oneShotButtonClicked()
{
	shotImage();
	ui_.btnSaveImage->setEnabled(true);
}


void AbstractCalibrationWindow::captureButtonClicked()
{
	if(camera_->isCapturing())
	{
		camera_->stopCapture();
		ui_.btnCapture->setText("Start capturing");
		ui_.btnOneShot->setEnabled(true);
		ui_.ckbDetectCalibBoard->setChecked(false);	// This method will trigger slot [ detectBoardCheckboxStateChanged() ]
		ui_.ckbDetectCalibBoard->setEnabled(false);
	}
	else
	{
		camera_->startCapture();
		ui_.btnCapture->setText("Stop capturing");
		ui_.btnOneShot->setEnabled(false);
		ui_.ckbDetectCalibBoard->setEnabled(true);
		ui_.btnSaveImage->setEnabled(true);
		ui_.ckbDetectCalibBoard->setEnabled(true);
	}
}


void AbstractCalibrationWindow::detectBoardCheckboxStateChanged()
{
	auto checkbox = dynamic_cast<QCheckBox*>(sender());
	if(checkbox->isChecked())
	{
		startCalibBoardDetectThread();
	}
	else
	{
		stopCalibBoardDetectThread();
	}
}


void AbstractCalibrationWindow::calibPatternChanged()
{
	auto box = dynamic_cast<QComboBox*>(sender());
	auto text  = box->currentText();
	calib_pattern_ = UICommon::string_to_calib_pattern_map.find(text.toStdString())->second;
}


void AbstractCalibrationWindow::saveImageButtonClicked()
{
	auto filename = QFileDialog::getSaveFileName(this, "Save image", "", "*.png");
	saveImage(filename.toStdString());
}


void AbstractCalibrationWindow::calibrationButtonClicked()
{
	if(!is_calibrating_)
	{
		auto folder = QFileDialog::getExistingDirectory(this, "Please select a folder to store the planarCalibration files!");
		if(folder.isEmpty())
			return;

		calib_files_folder_ = folder.toStdString();
		is_calibrating_ = true;

		// Change controls' status
		ui_.btnCalibration->setText("Stop stereo calibration");
		ui_.btnGrabCalibImage->setText("Grab Calibration Image");
		ui_.gbOptions->setEnabled(false);
		ui_.gbCamOperations->setEnabled(false);
		ui_.btnGrabCalibImage->setEnabled(true);

		// Enable real-time preview
		camera_->startCapture();
		startCalibBoardDetectThread();
		
	}
	else
	{
		calib_files_folder_ = "";
		is_calibrating_ = false;

		ui_.btnCalibration->setText("Start stereo calibration");
		ui_.gbOptions->setEnabled(true);
		ui_.gbCamOperations->setEnabled(true);
		ui_.btnGrabCalibImage->setEnabled(false);

		// Disable real-time preview
		camera_->stopCapture();
		stopCalibBoardDetectThread();
		abortCalibration();
	}
}


void AbstractCalibrationWindow::grabCalibImageButtonClicked()
{
	shotCalibImage();
}


void AbstractCalibrationWindow::finishCalibrationImageGrabbing()
{
	calibrate(calib_files_folder_);
}


void AbstractCalibrationWindow::parameterButtonClicked() const
{
	camera_->showParameterDialog();
}


void AbstractCalibrationWindow::calibBoardSettingsButtonClicked()
{
	auto button = dynamic_cast<QPushButton*>(sender());
	if(button != ui_.btnCalibBoardSetting)
		return;
	CalibBoardSettingsDialog dialog(calib_board_settings_, this);
	dialog.exec();
}
