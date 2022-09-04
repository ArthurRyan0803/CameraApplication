#include "CloudProcessParamsDialog.h"


#define INITIALIZE_SPIN_BOX(BOX, VALUE, MIN, MAX) (BOX)->setValue(VALUE); (BOX)->setMinimum(MIN); (BOX)->setMaximum(MAX);


CloudProcessParamsDialog::CloudProcessParamsDialog(PointCloudProcessParams& params, QWidget *parent)
	: QDialog(parent), original_params_(params), modified_params_(params)
{
	ui.setupUi(this);

	// Slots
	connect(ui.dsbVoxelSize, &QDoubleSpinBox::textChanged, this, &CloudProcessParamsDialog::voxelSizeChanged);
	connect(ui.sbFilterNeighbors, &QSpinBox::textChanged, this, &CloudProcessParamsDialog::filterNeighborsChanged);
	connect(ui.dsbFilterStddevThd, &QDoubleSpinBox::textChanged, this, &CloudProcessParamsDialog::filterStddevThdChanged);
	connect(ui.sbICPIters, &QSpinBox::textChanged, this, &CloudProcessParamsDialog::icpItersChanged);
	connect(ui.cbStitchProcess, &QCheckBox::stateChanged, this, &CloudProcessParamsDialog::stitchedCloudProcessEnabledChanged);

	connect(ui.btnOK, &QPushButton::clicked, this, &CloudProcessParamsDialog::btnOKClicked);
	connect(ui.btnCancel, &QPushButton::clicked, this, &CloudProcessParamsDialog::btnCancelClicked);
	connect(ui.btnRestore, &QPushButton::clicked, this, &CloudProcessParamsDialog::btnRestoreClicked);

	// UI initialization
	updateUIValues(original_params_);
}

void CloudProcessParamsDialog::voxelSizeChanged()
{
	auto obj = dynamic_cast<QDoubleSpinBox*>(sender());
	assert(obj);

	modified_params_.voxel_size = obj->value();
}

void CloudProcessParamsDialog::filterNeighborsChanged()
{
	auto obj = dynamic_cast<QSpinBox*>(sender());
	assert(obj);

	modified_params_.filter_neighbors = obj->value();
}

void CloudProcessParamsDialog::filterStddevThdChanged()
{
	auto obj = dynamic_cast<QDoubleSpinBox*>(sender());
	assert(obj);

	modified_params_.filter_stddev_thd = obj->value();
}

void CloudProcessParamsDialog::icpItersChanged()
{
	auto obj = dynamic_cast<QSpinBox*>(sender());
	assert(obj);

	modified_params_.icp_iters = obj->value();
}

void CloudProcessParamsDialog::stitchedCloudProcessEnabledChanged()
{
	auto obj = dynamic_cast<QCheckBox*>(sender());
	assert(obj);

	auto enabled = obj->checkState() == Qt::CheckState::Checked;
	modified_params_.process_stitched_cloud = enabled;
}

void CloudProcessParamsDialog::btnOKClicked()
{
	original_params_ = modified_params_;
	close();
}

void CloudProcessParamsDialog::btnCancelClicked()
{
	close();
}

void CloudProcessParamsDialog::btnRestoreClicked()
{
	updateUIValues(original_params_);
}

void CloudProcessParamsDialog::updateUIValues(const PointCloudProcessParams& params)
{
	INITIALIZE_SPIN_BOX(ui.dsbVoxelSize, params.voxel_size, 0, 100);
	INITIALIZE_SPIN_BOX(ui.sbFilterNeighbors, params.filter_neighbors, 0, 100);
	INITIALIZE_SPIN_BOX(ui.dsbFilterStddevThd, params.filter_stddev_thd, 0.1, 100);
	INITIALIZE_SPIN_BOX(ui.sbICPIters, params.icp_iters, 1, 50);
	ui.cbStitchProcess->setCheckState(params.process_stitched_cloud ? Qt::CheckState::Checked: Qt::Unchecked);
}

CloudProcessParamsDialog::~CloudProcessParamsDialog() { }
