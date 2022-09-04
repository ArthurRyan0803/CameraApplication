#pragma once

#include <QDialog>

#include "ui_CloudProcessParamsDialog.h"
#include "Params.hpp"

class CloudProcessParamsDialog : public QDialog
{
	Q_OBJECT

public:
	CloudProcessParamsDialog(PointCloudProcessParams& params, QWidget *parent = nullptr);
	~CloudProcessParamsDialog();

private:
	Ui::CloudProcessParamsDialogClass ui;

    PointCloudProcessParams modified_params_;
    PointCloudProcessParams& original_params_;

	void voxelSizeChanged();
    void filterNeighborsChanged();
    void filterStddevThdChanged();
    void icpItersChanged();
    void stitchedCloudProcessEnabledChanged();
    void btnOKClicked();
    void btnCancelClicked();
    void btnRestoreClicked();

    void updateUIValues(const PointCloudProcessParams& params);
};
