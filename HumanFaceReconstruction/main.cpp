#include <QtWidgets/QApplication>
#include <vtk-9.0/vtkObject.h>

#include "HumanFaceReconstruction.h"
#include "Logger.hpp"

int main(int argc, char *argv[])
{
    try
    {
        qputenv("QT_ENABLE_HIGHDPI_SCALING", "1");
        QGuiApplication::setHighDpiScaleFactorRoundingPolicy(Qt::HighDpiScaleFactorRoundingPolicy::PassThrough);

        vtkObject::GlobalWarningDisplayOff();
	    QApplication a(argc, argv);
	    HumanFaceReconstruction w;
	    w.showMaximized();
	    return a.exec();
    }
    catch (const std::exception& e)
    {
	    auto& logger = GET_LOGGER();
        logger.error(e.what());
        return -1;
    }
}
