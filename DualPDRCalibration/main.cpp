#include "DualPDRCalibrationWindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DualPDRCalibrationWindow w;
    w.showMaximized();
    return a.exec();
}
