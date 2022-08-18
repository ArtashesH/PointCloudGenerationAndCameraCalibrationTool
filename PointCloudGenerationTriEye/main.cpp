#include "PointCloudGenerationTriEye.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{



   /* //calculate focal lenght
    float pixelSizeInMM = 0.007;
    float sensorHeightMM = 6.72;
    float objectHeightInPixel = 25;
    float sensorHeightInPixel = 960;
    float objectHeightOnSensorMM = sensorHeightMM * objectHeightInPixel / sensorHeightInPixel;
    float distanceToObjectMM = 10000;
    float realObjectSizeInMM = 50;
    float focalLenghtInMM = objectHeightOnSensorMM * distanceToObjectMM / realObjectSizeInMM;
    float sx = 1.0 / (pixelSizeInMM) ;
    float focalLenghtInPixlH = focalLenghtInMM *  sx;
    std::cout << "Focal lenght in MM  " << focalLenghtInMM << std::endl;
    std::cout << "Focal lenght in Pixels " << focalLenghtInPixlH << std::endl;
    return 0;*/
    
    QApplication a(argc, argv);
    PointCloudGenerationTriEye w;
    w.show();
    return a.exec();
}
