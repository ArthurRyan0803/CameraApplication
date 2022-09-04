#include <functional>
#include "FunctionalDialog.h"
#include <thread>
#include <QMetaMethod>

#include "CalibrationMethods.h"


FunctionalDialog::FunctionalDialog(QWidget *parent, const std::function<void()>&& func, const std::string& message)
	: QDialog(parent)
{
	ui.setupUi(this);

	func_ = func;
	ui.label->setText(QString::fromStdString(message));

	setWindowTitle("");
	setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::WindowSystemMenuHint | Qt::CustomizeWindowHint);
}

void FunctionalDialog::showEvent(QShowEvent* event)
{
	QDialog::showEvent(event);

	std::thread thread([this]
	{
		func_();
		QMetaObject::invokeMethod(this, [this]{close();}, Qt::ConnectionType::QueuedConnection);
	});

	thread.detach();
}
