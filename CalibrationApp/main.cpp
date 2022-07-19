#include "MainNavigationWindow.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainNavigationWindow w;
    w.show();
    return a.exec();
}
