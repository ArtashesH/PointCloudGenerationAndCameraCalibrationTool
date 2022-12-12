#include "PointCloudGenerationTriEye.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
   
    QApplication a(argc, argv);
    PointCloudGenerationTriEye w;
    w.show();
    return a.exec();
}
