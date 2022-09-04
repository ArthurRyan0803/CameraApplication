#include "ProjectionPatternPreview.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    std::shared_ptr<ProjectionPatternPreview> w = std::make_shared<ProjectionPatternPreview>();        // enable shared from this
    w->show();
    return a.exec();
}
