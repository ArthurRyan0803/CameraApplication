#pragma once

#include <QMainWindow>
#include <Mutex>
#include <qwidget.h>
#include <VSensorPointCloudCamera.h>
#include <Logger.hpp>

#include "ui_CamerasImagesPreviewWindow.h"
#include "Params.hpp"

class CamerasImagesPreviewWindow : public QMainWindow
{
	Q_OBJECT

public:
	CamerasImagesPreviewWindow(
		std::shared_ptr<CameraLib::VSensorPointCloudCamera> left_camera, 
		std::shared_ptr<CameraLib::VSensorPointCloudCamera> right_camera,
		std::shared_ptr<DualCamerasParams> cameras_params, QWidget *parent = nullptr
	);
	~CamerasImagesPreviewWindow();

private:
	std::shared_ptr<DualCamerasParams> cameras_params_;

	std::shared_ptr<CameraLib::VSensorPointCloudCamera> left_camera_, right_camera_, selected_camera_;

	std::array<std::unique_ptr<QImage>, 6> q_images_;
	std::array<std::mutex, 6> mutexes_;
	std::array<QWidget*, 6> canvas_;

	Ui::CamerasImagesPreviewWindowClass ui;

	// Slot
	void oneShot();
	void selectedCameraChanged();
	void grayExpValueChanged();
	void grayGainValueChanged();
	void colorExpValueChanged();
	void colorGainValueChanged();
	void whiteBalance();

	void overwriteParams();
	void restoreParams();

	void showEvent( QShowEvent* e ) override;
	void closeEvent( QCloseEvent* e ) override;

	bool eventFilter(QObject* obj, QEvent* e);
	bool paintImage(QWidget* canvas, std::unique_ptr<QImage>& qimage);

	void updateParametersUi(const CameraLib::VSensorPointCloudCamera& camera) const;
};
