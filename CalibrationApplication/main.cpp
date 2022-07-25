#include "MainNavigationWindow.h"
#include <QtWidgets/QApplication>
#include "Logger.hpp"


int main(int argc, char *argv[])
{
    try
    {
	    QApplication a(argc, argv);
	    MainNavigationWindow w;
	    w.show();
	    return a.exec();
    }
    catch(const std::exception& e)
    {
        auto& logger = Logger::instance(__FILE__);
        logger.error("Application error!");
        logger.error("Exception: " + std::string(e.what()));
	    return -1;
    }
}
