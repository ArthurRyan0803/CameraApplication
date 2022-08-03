#include <QtWidgets/QApplication>
#include <vtkObject.h>

#include "MainNavigationWindow.h"
#include "Logger.hpp"



int main(int argc, char *argv[])
{
    try
    {
        vtkObject::GlobalWarningDisplayOff();
	    QApplication a(argc, argv);
	    MainNavigationWindow w;
	    w.show();
	    return a.exec();
    }
    catch(const std::exception& e)
    {
        auto& logger = GET_LOGGER();
        logger.error("Application error!");
        logger.error("Exception: " + std::string(e.what()));
	    return -1;
    }
}
