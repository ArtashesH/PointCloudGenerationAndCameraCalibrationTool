#include <QFileDialog>
#include <QDebug>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <QMessageBox>




#include "PointCloudGenerationTriEye.h"



//Constructor for creating main widget
PointCloudGenerationTriEye::PointCloudGenerationTriEye(QWidget *parent)
    : QWidget(parent)
{


    m_dataSelectionWidget = new DataSelectionWidget;
    m_calibDataSelectionWidget = new CalibrationDataSelectionWidget;

    this->setFixedHeight(400);
    this->setFixedWidth(1100);
    m_mainLayout = new QVBoxLayout;
    m_mainLayout->setSpacing(100);

 


   
    m_runCalibrationButton = new QPushButton("Run Calibration");
    m_runCalibrationButton->setFixedHeight(50);
    m_runCalibrationButton->setFixedWidth(180);


    m_selectImagesFolderButton= new QPushButton("Select Images For Calibration");
    m_selectImagesFolderButton->setFixedHeight(50);
    m_selectImagesFolderButton->setFixedWidth(180);



    m_selectDepthAndAggrDataButton = new QPushButton("Input Depth Data");
    m_selectDepthAndAggrDataButton->setFixedHeight(50);
    m_selectDepthAndAggrDataButton->setFixedWidth(180);


    m_selectCalibrationDataButton = new QPushButton("Input Calibration Data");
    m_selectCalibrationDataButton->setFixedHeight(50);
    m_selectCalibrationDataButton->setFixedWidth(180);


    m_runPointCloudGenerationButon = new QPushButton("Point Cloud Generation");
    m_runPointCloudGenerationButon->setFixedHeight(50);
    m_runPointCloudGenerationButon->setFixedWidth(180);


    m_checkboxForDepthFiler = new QCheckBox;
    m_depthFilterLabel = new QLabel;
    m_depthFilterLabel->setText("Depth Filer");

    m_checkboxForPointCloudFiler = new QCheckBox;
    m_checkboxForPointCloudFiler->setEnabled(false);
    m_pointCloudFilterLabel = new QLabel;
    m_pointCloudFilterLabel->setText("Point Cloud Filer");




    m_squareSizeLabel = new QLabel;
    m_squareSizeLabel->setText("Square Size in cm");

    m_lineForSquareSize = new QLineEdit;
   
    QRegExpValidator* rxv = new QRegExpValidator(QRegExp("\\d*"), this);
    m_lineForSquareSize->setText(0);
    m_lineForSquareSize->setFixedHeight(30);
    m_lineForSquareSize->setFixedWidth(50);
    m_lineForSquareSize->setValidator(rxv);


    m_checkerboardWidthLabel = new QLabel;
    m_checkerboardWidthLabel->setText("Checkerboard Width");

    m_lineForCheckerBoardWidth = new QLineEdit;
    m_lineForCheckerBoardWidth->setFixedHeight(30);
    m_lineForCheckerBoardWidth->setFixedWidth(50);
    m_lineForCheckerBoardWidth->setValidator(rxv);


    m_checkerboardHeightLabel = new QLabel;
    m_checkerboardHeightLabel->setText("Checkerboard Height");

    m_lineForCheckerBoardHeight = new QLineEdit;
    m_lineForCheckerBoardHeight->setFixedHeight(30);
    m_lineForCheckerBoardHeight->setFixedWidth(50);
    m_lineForCheckerBoardHeight->setValidator(rxv);



    m_calibrationLayout = new QHBoxLayout;
    m_calibrationLayout->addWidget(m_runCalibrationButton);
    m_calibrationLayout->addWidget(m_selectImagesFolderButton);


    m_progressBar = new QProgressBar;
    m_progressBar->setMinimum(0);
    m_progressBar->setMaximum(100);

    m_calibrationLayout->SetFixedSize;
    m_calibrationLayout->setMargin(10);
    m_calibrationLayout->addWidget(m_squareSizeLabel);
    m_calibrationLayout->setSpacing(20);
    m_calibrationLayout->setMargin(10);
    m_calibrationLayout->addWidget(m_lineForSquareSize);

    m_calibrationLayout->addWidget(m_checkerboardWidthLabel);
    m_calibrationLayout->addWidget(m_lineForCheckerBoardWidth);

    m_calibrationLayout->addWidget(m_checkerboardHeightLabel);
    m_calibrationLayout->addWidget(m_lineForCheckerBoardHeight);

    m_progressBarLayout = new QHBoxLayout;
    m_progressBarLayout->addWidget(m_progressBar);




    m_pointCloudGenerationLayout = new QHBoxLayout;
    //m_pointCloudGenerationLayout->setGeometry(QRect(10,10,400,400));
 //   m_pointCloudGenerationLayout->setSpacing(100);
  //  m_pointCloudGenerationLayout->setMargin(1);
    m_pointCloudGenerationLayout->addWidget(m_selectDepthAndAggrDataButton);
    m_pointCloudGenerationLayout->setMargin(10);

    m_pointCloudGenerationLayout->addWidget(m_selectCalibrationDataButton);
    m_pointCloudGenerationLayout->setMargin(10);
    m_pointCloudGenerationLayout->addWidget(m_runPointCloudGenerationButon);

    m_pointCloudGenerationLayout->setSpacing(20);
    m_pointCloudGenerationLayout->addWidget(m_depthFilterLabel);
    //m_pointCloudGenerationLayout->setMargin(200);
   // m_pointCloudGenerationLayout->setSpacing(1);
    m_pointCloudGenerationLayout->addWidget(m_checkboxForDepthFiler);

    m_pointCloudGenerationLayout->addWidget(m_pointCloudFilterLabel);
    m_pointCloudGenerationLayout->addWidget(m_checkboxForPointCloudFiler);
   /// m_pointCloudGenerationLayout->addWidget(m_progressBar);

   // m_pointCloudGenerationLayout->setContentsMargins(0, 0, 0, 0);
   // m_pointCloudGenerationLayout->setSpacing(20);
   
   // QHBoxLayout* tmpLayTest = new QHBoxLayout;
   // tmpLayTest->SetFixedSize;
   // tmpLayTest->addWidget(m_depthFilterLabel);
   // tmpLayTest->addWidget(m_checkboxForDepthFiler);

   // layout = new QGridLayout;
   // layout->addWidget(m_runPointCloudGenerationButton, 0, 0);
   // layout->addLayout(tmpLayTest,0,1);
    //layout->addWidget(m_depthFilterLabel, 0, 1);
    //layout->addWidget(m_checkboxForDepthFiler, 0, 2);
    //layout->addRow(m_runPointCloudGenerationButton, m_depthFilterLabel);
    //layout->
    //layout->addWidget(m_checkboxForDepthFiler);
    
    
    m_mainLayout->addLayout(m_calibrationLayout);
  //  m_mainLayout->addLayout(m_progressBarLayout);
    m_mainLayout->addLayout(m_pointCloudGenerationLayout);
    
    this->setLayout(m_mainLayout);
    this->show();

   /* QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &PointCloudGenerationTriEye::updateProgress);
    timer->start(100);
   */

    connect(m_selectImagesFolderButton, SIGNAL(clicked()), this, SLOT(selectCheckerboardImagesFolderSlot()));
    connect(m_runCalibrationButton, SIGNAL(clicked()), this, SLOT(runCalibrationSlot()));
    connect(m_selectDepthAndAggrDataButton, SIGNAL(clicked()), this, SLOT(runDepthAndAggrDataSelectionSlot()));
    connect(m_dataSelectionWidget, SIGNAL(widgetClosed()), this, SLOT(closeDataSelectionWidget()));  
    connect(m_selectCalibrationDataButton, SIGNAL(clicked()), this, SLOT(runCalibrationDataSelectionSlot()));
    connect(m_calibDataSelectionWidget, SIGNAL(widgetClosed()), this, SLOT(closeCalibDataSelectionWidget()));
    connect(m_calibDataSelectionWidget, SIGNAL(calibrationFinished()), this, SLOT(closeCalibDataSelectionWidget()));
    connect(m_runPointCloudGenerationButon, SIGNAL(clicked()), this, SLOT(runPointCloudGenerationSlot()));
   // this->setLayout(m_mainLayout);
    //this->setCentralWidget()
   // ui.setupUi(this);
    //ui.centralWidget(this);

}

//Select folder for checkerboard calibration
void PointCloudGenerationTriEye::selectCheckerboardImagesFolderSlot()
{
    QFileDialog dialog;
   // QFileDialog* fileDialog = new QFileDialog;
   // fileDialog->setDefaultSuffix("txt");
   // QString grayScaleImagePath = fileDialog->getExistingDirectory(this, "Curr Dir");//  getOpenFileName(this, "Grayscale Images", "Image", "Images(*.tiff)");
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    //dialog.setOption(QFileDialog::DontUseNativeDialog, true);
    dialog.setOption(QFileDialog::ShowDirsOnly, true);
   //dialog.setOptions(QFileDialog::DontUseNativeDialog);
   // QString strFileName = dialog.getExistingDirectory(this, tr("Select File"), "", tr("All Files (*.*)"));

    QString filename = dialog.getExistingDirectory(this, "Choose Folder");
    std::cout << "selected folder path " << filename.toStdString() << std::endl;
    m_checkerboardImagesFolderPath = filename;

}


void PointCloudGenerationTriEye::updateProgress(const unsigned int& updateSeconds)
{
    m_progressBar->setValue(/*m_progressBar->value() + */updateSeconds);
    /*if (m_progressBar->value() == 100) {
        m_progressBar->setValue(0);
    }*/
}

void PointCloudGenerationTriEye::runCalibrationSlot()
{


    // Creating vector to store vectors of 3D points for each checkerboard image
    QMessageBox messageBox;
   
    m_lineForSquareSize->update();
    m_lineForCheckerBoardWidth->update();
    m_lineForCheckerBoardHeight->update();
    if (m_lineForSquareSize->text().toUInt() == 0) {
        
        messageBox.critical(0, "Error", "PLease specify checkerboard square size !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }


    if (m_lineForCheckerBoardWidth->text().toUInt() == 0) {
        messageBox.critical(0, "Error", "PLease specify checkerboard width !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }
    if (m_lineForCheckerBoardHeight->text().toUInt() == 0) {
        messageBox.critical(0, "Error", "PLease specify checkerboard height !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }


    if (m_checkerboardImagesFolderPath == "") {
        messageBox.critical(0, "Error", "PLease specify folder path for checkerboard images!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }
   

    std::vector<std::vector<cv::Point3f> > objpoints;
    float square_size = m_lineForSquareSize->text().toUInt();

    unsigned int checkerboardWidth = m_lineForCheckerBoardWidth->text().toUInt();
    unsigned int checkerboardHeight = m_lineForCheckerBoardHeight->text().toUInt();
    int CHECKERBOARD[2]{ checkerboardWidth,checkerboardHeight};

    // Creating vector to store vectors of 2D points for each checkerboard image

    std::vector<std::vector<cv::Point2f> > imgpoints;



    // Defining the world coordinates for 3D points

    std::vector<cv::Point3f> objp;

    for (int i{ 0 }; i < CHECKERBOARD[1]; i++)

    {

        for (int j{ 0 }; j < CHECKERBOARD[0]; j++) {
            cv::Point3f currentCorner3DPoint;
            currentCorner3DPoint.x = j * square_size;
            currentCorner3DPoint.y = i * square_size;
            currentCorner3DPoint.z = 0;
            objp.push_back(currentCorner3DPoint);
        }
    }





    // Extracting path of individual image stored in a given directory

    std::vector<cv::String> images;

    // Path of the folder containing checkerboard images

    std::string path = m_checkerboardImagesFolderPath.toStdString();// "E:/UpworkProjects/PointCloudGenerationUtility_TriEye/gd_data_with_calibration_04_05_22-20220508T135052Z-001/gd_data_with_calibration_04_05_22/Calibration/experiment_2022_05_03_T_13_02_51_scene3_15meters/*.tiff";
    //  std::string path = "E:/UpworkProjects/PointCloudGenerationUtility_TriEye/gd_data_with_calibration_13_06_22-20220613T121718Z-001/gd_data_with_calibration_13_06_22/Calibration/experiment_2022_06_12_T_15_36_30_20m/*.tiff";

    path += "/*.tiff";
    cv::glob(path, images);



    cv::Mat frame, gray;

    // vector to store the pixel coordinates of detected checker board corners

    std::vector<cv::Point2f> corner_pts;

    bool success;



    // Looping over all the images in the directory
   
    if (images.size() == 0) {
        messageBox.critical(0, "Error", "Specified folder does not contain tiff images !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }
    for (int i{ 0 }; i < images.size(); i++)

    {
        updateProgress(i*100/images.size());
        frame = cv::imread(images[i]);
        std::cout << "Current Image path " << images[i] << std::endl;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);



        // Finding checker board corners

        // If desired number of corners are found in the image then success = true 

        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);



        /*
    51
         * If desired number of corner are detected,
    52
         * we refine the pixel coordinates and display
    53
         * them on the images of checker board
    54
        */

        if (success)

        {

          //Initial working version  cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 40, 0.001);

            // refining pixel coordinates for given 2d points.

           //Initial working version cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            
            cv::cornerSubPix(gray, corner_pts, cv::Size(3, 3), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board

            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);



            objpoints.push_back(objp);

            imgpoints.push_back(corner_pts);

        }



        cv::imshow("Image", frame);

        cv::waitKey(10);

    }



    cv::destroyAllWindows();



    cv::Mat cameraMatrix, distCoeffs, R, T;



    /*
  78
     * Performing camera calibration by
  79
     * passing the value of known 3D points (objpoints)
  80
     * and corresponding pixel coordinates of the
  81
     * detected corners (imgpoints)
  82
    */
    if (objpoints.size() == 0 || imgpoints.size() == 0) {
        messageBox.critical(0, "Error", "Cannot find any corners in checkerboards !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;

    }
    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);
    updateProgress(100);
    m_progressBar->show();
    QString calibDataResFileName = QFileDialog::getSaveFileName(this, "Save file", "", ".yml");
    cv::FileStorage calibDatSt(calibDataResFileName.toStdString() + ".yml", cv::FileStorage::WRITE);
    calibDatSt << "K"<<cameraMatrix;
    calibDatSt << "D" << distCoeffs;
    calibDatSt.release();


    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;

    std::cout << "distCoeffs : " << distCoeffs << std::endl;

    std::cout << "Rotation vector : " << R << std::endl;

    std::cout << "Translation vector : " << T << std::endl;


 

    /*calibDatSt.open("testCalib.yml", cv::FileStorage::READ);
    cv::Mat cameraKMatrix;
    calibDatSt["K"] >> cameraKMatrix;
    std::cout << "Final camera matrix after calibration \n";
    std::cout << cameraKMatrix << std::endl;
    */



}

void PointCloudGenerationTriEye::outlierRemoval(const   pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud)

 {


 std::cerr << "Cloud before filtering: " << std::endl;
 std::cerr << *inputCloud << std::endl;

 // Create the filtering object
//Outlier removal filter
/* pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
 sor.setInputCloud(inputCloud);
 sor.setMeanK(50);
 sor.setStddevMulThresh(0.05);
 sor.filter(*outputCloud);
 sor.setNegative(true);
 sor.filter(*outputCloud);*/


 //Radius filter

 pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
 outrem.setInputCloud(inputCloud);
 outrem.setRadiusSearch(0.009);
 outrem.setMinNeighborsInRadius(400);
 /*outrem.setRadiusSearch(0.9);
 outrem.setMinNeighborsInRadius(10);
 */
 // apply filter
 outrem.filter(*outputCloud);
}


QVector<QString>  PointCloudGenerationTriEye::getAllDepthImagePathsInMixedFolder(const QString& folderPath)
{
    QVector<QString> depthImagePaths;
    std::vector<cv::String> images;



    std::string path = folderPath.toStdString();

    path += "/*.tiff";
    cv::glob(path, images);
    for (int i = 0; i < images.size(); ++i) {
        std::string baseFilename = images[i].substr(images[i].find_last_of("/\\") + 1);      
        std::string baseFileTypeName = cv::String(baseFilename).substr(0,cv::String(baseFilename).find_first_of("_"));
        if (baseFileTypeName == "depth") {
            depthImagePaths.push_back(/*images[i].c_str()*/baseFilename.c_str());
        }
       
    }


    return depthImagePaths;
}

QVector<QString> PointCloudGenerationTriEye::getAllAggrImagePathsInMixedFolder(const QString& folderPath)
{
    QVector<QString> aggrImagePaths;

    std::vector<cv::String> images;

    std::string path = folderPath.toStdString();

    path += "/*.tiff";
    cv::glob(path, images);
    for (int i = 0; i < images.size(); ++i) {
        std::string baseFilename = images[i].substr(images[i].find_last_of("/\\") + 1);
        std::string baseFileTypeName = cv::String(baseFilename).substr(0, cv::String(baseFilename).find_first_of("_"));
        if (baseFileTypeName == "aggregate") {
            aggrImagePaths.push_back(baseFilename.c_str());
        }

    }

    return aggrImagePaths;
}



QVector<QPair<QString, QString>> PointCloudGenerationTriEye::getDepthAndAggrPairsForExperimentFolder(const std::vector<cv::String>& depthImages, const std::vector<cv::String>& aggrImages)
{
    QVector<QString> depthImagesBaseNames;
    QVector<QString> aggrImagesBaseNames;

    QVector<QPair<QString, QString>> depthAndAggrImagePairsVec;

    for (int i = 0; i < depthImages.size(); ++i) {
        std::string baseFilename = depthImages[i].substr(depthImages[i].find_last_of("/\\") + 1);
        depthImagesBaseNames.push_back(QString(baseFilename.c_str()));
    }
    for (int i = 0; i < aggrImages.size(); ++i) {
        std::string baseFilename = aggrImages[i].substr(aggrImages[i].find_last_of("/\\") + 1);
        aggrImagesBaseNames.push_back(QString(baseFilename.c_str()));
    }
    qDebug() << "Aggr images in experiment folder ";
    for (int i = 0; i < depthImagesBaseNames.size(); ++i) {
        QPair<QString, QString> tmpPair;
        tmpPair.first = depthImages[i].c_str();
        tmpPair.second = "";
        for (int j = 0; j < aggrImagesBaseNames.size(); ++j) {
            if (aggrImagesBaseNames[j] == depthImagesBaseNames[i]) {
                tmpPair.second = aggrImages[j].c_str();
                break;
            }
        }
        depthAndAggrImagePairsVec.push_back(tmpPair);

    }
    return depthAndAggrImagePairsVec;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudGenerationTriEye::pointCloudGenerationForOnePair(const cv::Mat& depthImg, const cv::Mat& aggrImage, const cv::Mat& cameraMatrix)
{


  

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new  pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudFiltered(new  pcl::PointCloud<pcl::PointXYZRGB>());


    cv::Mat inputDepthImage = depthImg.clone();

    if (m_checkboxForDepthFiler->isChecked()) {
        inputDepthImage.convertTo(inputDepthImage, CV_32FC1);
        cv::Mat outputMat;
        cv::bilateralFilter(inputDepthImage, outputMat, 35, 135, 135);
        inputDepthImage = outputMat;
        inputDepthImage.convertTo(inputDepthImage, CV_16SC1);


    }

   // cv::Mat aggrImage = cv::imread(grayScaleImagePath.toStdString());
    cv::Mat depthInFLoat = cv::Mat(inputDepthImage.rows, inputDepthImage.cols, CV_32F);
    std::cout << "Depth  image channels count " << inputDepthImage.channels() << std::endl;
    float minDepthValue = 100000;
    float maxDepthValue = 00;
    for (int i = 0; i < depthInFLoat.rows; ++i) {
        for (int j = 0; j < depthInFLoat.cols; ++j) {
            depthInFLoat.at<float>(i, j) = (float)(inputDepthImage.at<unsigned short>(i, j)) / 1000.0;
        }
    }
    for (int i = 0; i < depthInFLoat.rows; ++i) {
        for (int j = 0; j < depthInFLoat.cols; ++j) {
            //std::cout << inputDepthImage.at<unsigned short>(i, j)/1000.0 << std::endl;
            if (depthInFLoat.at<float>(i, j) > maxDepthValue) {
                maxDepthValue = depthInFLoat.at<float>(i, j);
            }
            if (depthInFLoat.at<float>(i, j) < minDepthValue) {
                minDepthValue = depthInFLoat.at<float>(i, j);
            }
            //depthInFLoat.at<unsigned short>(i, j) = inputDepthImage.at<unsigned short>(i, j) ;
        }
    }
    std::cout << "MIn value in depth " << minDepthValue << std::endl;
    std::cout << "Max value in depth " << maxDepthValue << std::endl;
    std::cout << "Type of depth image  " << inputDepthImage.depth() << std::endl;
    //return 0;
    std::cout << "current value " << (inputDepthImage.at<unsigned short>(591, 990)) << std::endl;
    cv::Mat points3d;


    for (int i = 0; i < inputDepthImage.rows; ++i) {
        for (int j = 0; j < inputDepthImage.cols; ++j) {
            pcl::PointXYZRGB tmpPoint;

            const float depthConst = 0.01;
            if (aggrImage.rows == 0 || aggrImage.cols == 0) {
                tmpPoint.r = 125;
                tmpPoint.g = 125;
                tmpPoint.b = 125;
            }
            else {
                tmpPoint.r = aggrImage.at<cv::Vec3b>(i, j)[0];
                tmpPoint.g = aggrImage.at<cv::Vec3b>(i, j)[0];
                tmpPoint.b = aggrImage.at<cv::Vec3b>(i, j)[0];
            }
            tmpPoint.x = ((float)j - m_cameraMatrix.at<double>(0, 2)) * (float)(inputDepthImage.at<ushort>(i, j) * depthConst) / m_cameraMatrix.at<double>(0, 0);
            tmpPoint.y = ((float)i - m_cameraMatrix.at<double>(1, 2)) * (float)(inputDepthImage.at<ushort>(i, j) * depthConst) / m_cameraMatrix.at<double>(1, 1); /*6478.693369417085*//*6104.450415013259*//*1033.093649821804  /*6690.421208001283*/;
            tmpPoint.z = inputDepthImage.at<ushort>(i, j) * depthConst;

            pointCloud->push_back(tmpPoint);

        }
    }
    pointCloud->height = 1;
    pointCloud->width = pointCloud->points.size();
    pointCloud->is_dense = true;
    //pointCloud->points.resize(pointCloud->width * pointCloud->height);
    std::cout << "Type of data cloud 3d " << points3d.depth() << std::endl;
    std::cout << "before writign \n";
    //std::cout << "POint cloud data " << points3d.at<float>(321, 990) << std::endl;


    

    if (m_checkboxForPointCloudFiler->isChecked()) {
        outlierRemoval(pointCloud, pointCloudFiltered);
        return pointCloudFiltered;
    
    }
    else {
        return pointCloud;
    }
}


void PointCloudGenerationTriEye::runDepthAndAggrDataSelectionSlot()
{
    
    this->setDisabled(true);

    m_dataSelectionWidget->show();

    return;
    QFileDialog* fileDialog = new QFileDialog;
    fileDialog->setDefaultSuffix("txt");
    QString grayScaleImagePath = fileDialog->getOpenFileName(this, "Grayscale Images", "Image", "Images(*.tiff)");
    QMessageBox messageBox;
    if (grayScaleImagePath == "")
    {
        messageBox.critical(0, "Error", "GrayScale image was not selected !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }


    QString depthImagePath = fileDialog->getOpenFileName(this, "Depth Images", "Image", "Images(*.tiff)");
   
    if (depthImagePath == "")
    {
        messageBox.critical(0, "Error", "Depth image was not selected !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }

    QString calibFilePath = fileDialog->getOpenFileName(this, "Calibration File", "File", "Images(*.yml)");


   

    if (calibFilePath == "")
    {
        messageBox.critical(0, "Error", "Calibration file was not selected !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }


    cv::FileStorage calibDatSt;
    calibDatSt.open(calibFilePath.toStdString(), cv::FileStorage::READ);
    cv::Mat cameraKMatrix;
    calibDatSt["K"] >> cameraKMatrix;
    cv::Mat distCoeff;
    calibDatSt["D"] >> distCoeff;
    std::cout << "Final camera matrix after calibration \n";
    std::cout << cameraKMatrix.at<double>(0,0) << std::endl;
    //return;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new  pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudFiltered(new  pcl::PointCloud<pcl::PointXYZRGB>());

    
    cv::Mat inputDepthImage = cv::imread(depthImagePath.toStdString(), -1);

    if (m_checkboxForDepthFiler->isChecked()) {
        inputDepthImage.convertTo(inputDepthImage, CV_32FC1);
        cv::Mat outputMat;
        cv::bilateralFilter(inputDepthImage, outputMat, 35, 135, 135);
        inputDepthImage = outputMat;
        inputDepthImage.convertTo(inputDepthImage, CV_16SC1);


    }
      
    cv::Mat aggrImage = cv::imread(grayScaleImagePath.toStdString());
    cv::Mat depthInFLoat = cv::Mat(inputDepthImage.rows, inputDepthImage.cols, CV_32F);
    std::cout << "Depth  image channels count " << inputDepthImage.channels() << std::endl;
    float minDepthValue = 100000;
    float maxDepthValue = 00;
    for (int i = 0; i < depthInFLoat.rows; ++i) {
        for (int j = 0; j < depthInFLoat.cols; ++j) {
            depthInFLoat.at<float>(i, j) = (float)(inputDepthImage.at<unsigned short>(i, j)) / 1000.0;
        }
    }
    for (int i = 0; i < depthInFLoat.rows; ++i) {
        for (int j = 0; j < depthInFLoat.cols; ++j) {
            //std::cout << inputDepthImage.at<unsigned short>(i, j)/1000.0 << std::endl;
            if (depthInFLoat.at<float>(i, j) > maxDepthValue) {
                maxDepthValue = depthInFLoat.at<float>(i, j);
            }
            if (depthInFLoat.at<float>(i, j) < minDepthValue) {
                minDepthValue = depthInFLoat.at<float>(i, j);
            }
            //depthInFLoat.at<unsigned short>(i, j) = inputDepthImage.at<unsigned short>(i, j) ;
        }
    }
    std::cout << "MIn value in depth " << minDepthValue << std::endl;
    std::cout << "Max value in depth " << maxDepthValue << std::endl;
    std::cout << "Type of depth image  " << inputDepthImage.depth() << std::endl;
    //return 0;
    std::cout << "current value " << (inputDepthImage.at<unsigned short>(591, 990)) << std::endl;
    cv::Mat points3d;
 

    for (int i = 0; i < inputDepthImage.rows; ++i) {
        for (int j = 0; j < inputDepthImage.cols; ++j) {
            pcl::PointXYZRGB tmpPoint;
        
            const float depthConst = 0.01;
            tmpPoint.r = aggrImage.at<cv::Vec3b>(i, j)[0];
            tmpPoint.g = aggrImage.at<cv::Vec3b>(i, j)[0];
            tmpPoint.b = aggrImage.at<cv::Vec3b>(i, j)[0];
            tmpPoint.x = ((float)j - cameraKMatrix.at<double>(0,2)) * (float)(inputDepthImage.at<ushort>(i, j) * depthConst) / cameraKMatrix.at<double>(0, 0);
            tmpPoint.y = ((float)i - cameraKMatrix.at<double>(1, 2)) * (float)(inputDepthImage.at<ushort>(i, j) * depthConst) / cameraKMatrix.at<double>(1, 1); /*6478.693369417085*//*6104.450415013259*//*1033.093649821804  /*6690.421208001283*/;
            tmpPoint.z = inputDepthImage.at<ushort>(i, j) * depthConst;
           
            pointCloud->push_back(tmpPoint);

        }
    }
    pointCloud->height = 1;
    pointCloud->width = pointCloud->points.size();
    pointCloud->is_dense = true;
    //pointCloud->points.resize(pointCloud->width * pointCloud->height);
    std::cout << "Type of data cloud 3d " << points3d.depth() << std::endl;
    std::cout << "before writign \n";
    //std::cout << "POint cloud data " << points3d.at<float>(321, 990) << std::endl;


    QString pointCloudResFileName = QFileDialog::getSaveFileName(this, "Save point cloud file", "", ".ply");

    if (m_checkboxForPointCloudFiler->isChecked()) {
        outlierRemoval(pointCloud, pointCloudFiltered);
        pcl::io::savePLYFileASCII(pointCloudResFileName.toStdString() + ".ply", *pointCloudFiltered);
    }
    else {
        pcl::io::savePLYFileASCII(pointCloudResFileName.toStdString() + ".ply", *pointCloud);
    }
 

}

void PointCloudGenerationTriEye::runPointCloudGenerationSlot()
{

    QMessageBox messageBox;
    
   

    if (m_dataSelectionWidget->getSelectedDataType() == 0) {
        messageBox.information(0, "Info", "PLease select depth data!!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;

    }
    if (m_cameraMatrix.rows == 0 || m_cameraMatrix.cols == 0) {
        messageBox.information(0, "Info", "Please select calibration file or do calibration!!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }

    cv::Mat aggrImg;
    cv::Mat currDepthImg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new  pcl::PointCloud<pcl::PointXYZRGB>());
    if (m_dataSelectionWidget->getSelectedDataType() == 1  || m_dataSelectionWidget->getSelectedDataType() == 2) {
       
        if (m_dataSelectionWidget->getSelectedDataType() == 1) {
            currDepthImg = cv::imread(m_dataSelectionWidget->getSimpleDepthPath().toStdString(), -1);
        }
        else if (m_dataSelectionWidget->getSelectedDataType() == 2) {
            aggrImg = cv::imread(m_dataSelectionWidget->getDepthAndAggrImagePath().second.toStdString());
            currDepthImg = cv::imread(m_dataSelectionWidget->getDepthAndAggrImagePath().first.toStdString(),-1);

        }
        
        pointCloud = pointCloudGenerationForOnePair(currDepthImg, aggrImg, m_cameraMatrix);
        QString pointCloudResFileName = QFileDialog::getSaveFileName(this, "Save point cloud file", "", ".ply");
        pcl::io::savePLYFileASCII(pointCloudResFileName.toStdString() + ".ply", *pointCloud);
    }
    else if (m_dataSelectionWidget->getSelectedDataType() == 3) {
        QString depthImgFolderPath = m_dataSelectionWidget->getDepthImagesFolderPath();
        std::vector<cv::String> images;

        // Path of the folder containing checkerboard images

        std::string path = depthImgFolderPath.toStdString();

        path += "/*.tiff";
        cv::glob(path, images);



        // Looping over all the images in the directory

        if (images.size() == 0) {
            messageBox.critical(0, "Error", "Specified Depth Images folder Does Not Contain Tiff Images !!!");
            messageBox.setFixedSize(500, 200);
            messageBox.show();
            return;
        }
        for (int i{ 0 }; i < images.size(); i++) {
            currDepthImg = cv::imread(images[i], -1);
            pointCloud = pointCloudGenerationForOnePair(currDepthImg, aggrImg, m_cameraMatrix);           
            std::string currPointCloudSavePath = images[i].substr(0, images[i].find_last_of(".")) + ".ply";
            pcl::io::savePLYFileASCII(currPointCloudSavePath, *pointCloud);
        }
        
    }
    else if (m_dataSelectionWidget->getSelectedDataType() == 4) {
       
        std::cout << "Selcted deptha and aggr images folder \n";
        QString depthAndAggrImagesFolderPath = m_dataSelectionWidget->getDepthAndAggrImagesFolderPath();
        QVector<QString> depthImagesInSelectedFolder =  getAllDepthImagePathsInMixedFolder(depthAndAggrImagesFolderPath);
        QVector<QString> aggrImagesInSelectedFolder = getAllAggrImagePathsInMixedFolder(depthAndAggrImagesFolderPath);

        QVector<QString> depthImageEndings;
        QVector<QString> aggrImageEndings;

        for (int i = 0; i < depthImagesInSelectedFolder.size(); ++i) {
            QString currentDepthImageName = depthImagesInSelectedFolder[i];
            std::string imageIndexName = cv::String(currentDepthImageName.toStdString()).substr(cv::String(currentDepthImageName.toStdString()).find_first_of("_")+1);
            depthImageEndings.push_back(QString(imageIndexName.c_str()));
         //   qDebug() << "Image index name depth  " << imageIndexName.c_str();

        }


        for (int i = 0; i < aggrImagesInSelectedFolder.size(); ++i) {
            QString currentAggrImageName = aggrImagesInSelectedFolder[i];
            std::string imageIndexName = cv::String(currentAggrImageName.toStdString()).substr(cv::String(currentAggrImageName.toStdString()).find_first_of("_") + 1);
            aggrImageEndings.push_back(QString(imageIndexName.c_str()));
          //  qDebug() << "Image index name aggr  " << imageIndexName.c_str();

        }
        if (depthImageEndings.size() == 0) {
            messageBox.critical(0, "Error", "Specified Folder Does Not Contain Depth Images !!!");
            messageBox.setFixedSize(500, 200);
            messageBox.show();
            return;
        }

        for (int i = 0; i < depthImagesInSelectedFolder.size(); ++i) {
            currDepthImg = cv::Mat();
            aggrImg = cv::Mat();
            std::string currentDepthImagePath = depthAndAggrImagesFolderPath.toStdString() + "/" + depthImagesInSelectedFolder[i].toStdString();
            currDepthImg = cv::imread(currentDepthImagePath, -1);
            for (int j = 0; j < aggrImagesInSelectedFolder.size(); ++j) {

                if (depthImageEndings[i] == aggrImageEndings[j]) {                               
                    std::string currentAggrImagePath = depthAndAggrImagesFolderPath.toStdString() + "/" + aggrImagesInSelectedFolder[j].toStdString();
                    aggrImg = cv::imread(currentAggrImagePath);
                    break;                                     
                }               

            }
            pointCloud = pointCloudGenerationForOnePair(currDepthImg, aggrImg, m_cameraMatrix);
            std::string baseFilename = depthImageEndings[i].toStdString();
            baseFilename = baseFilename.substr(0, baseFilename.find_last_of("."));
            std::string currPointCloudSavePath = depthAndAggrImagesFolderPath.toStdString() + "/" + baseFilename + ".ply";
            pcl::io::savePLYFileASCII(currPointCloudSavePath, *pointCloud);
            qDebug() << "POINT CLODU SIOZE " << pointCloud->width;

        }

    }
    else if (m_dataSelectionWidget->getSelectedDataType() == 5) {
        //Selected experiment folder
        QString experFolderPath = m_dataSelectionWidget->getExperimentFolderPath();
        std::string pathDepth = experFolderPath.toStdString() + "/Raven/DepthImage";
        std::vector<cv::String> imagesDepth;
        pathDepth += "/*.tiff";
        cv::glob(pathDepth, imagesDepth);

        std::string pathAggr = experFolderPath.toStdString() + "/Raven/AggrImage";
        std::vector<cv::String> imagesAggr;
        pathAggr += "/*.tiff";
        cv::glob(pathAggr, imagesAggr);


        QVector<QPair<QString, QString>>  depthAndAggrImagesPairsVec =  getDepthAndAggrPairsForExperimentFolder(imagesDepth, imagesAggr);
      
        qDebug() << "After getting depth image ";

        if (imagesDepth.size() == 0) {
            messageBox.critical(0, "Error", "Specified Depth Images folder Does Not Contain Tiff Images !!!");
            messageBox.setFixedSize(500, 200);
            messageBox.show();
            return;
        }

        QString pointCloudFolderPath = experFolderPath + "/Raven/PointCloud";
        if (!QDir(pointCloudFolderPath).exists()) {
            QDir().mkdir(pointCloudFolderPath);
        }
        qDebug() << "Found depth and aggre image paris , with size " << depthAndAggrImagesPairsVec.size();
        for (int g = 0; g < depthAndAggrImagesPairsVec.size(); ++g) {
            currDepthImg = cv::Mat();
            aggrImg = cv::Mat();
            qDebug() << "Agrimage path ";
            qDebug() << depthAndAggrImagesPairsVec[g].second.toStdString().c_str();
            qDebug() << depthAndAggrImagesPairsVec[g].first.toStdString().c_str();

            if (depthAndAggrImagesPairsVec[g].second != "") {
                aggrImg = cv::imread(depthAndAggrImagesPairsVec[g].second.toStdString());
            }
            currDepthImg = cv::imread(depthAndAggrImagesPairsVec[g].first.toStdString(),-1);
            pointCloud = pointCloudGenerationForOnePair(currDepthImg, aggrImg, m_cameraMatrix);
            std::string baseFilename = depthAndAggrImagesPairsVec[g].first.toStdString().substr(depthAndAggrImagesPairsVec[g].first.toStdString().find_last_of("/\\") + 1);
            baseFilename = baseFilename.substr(0, baseFilename.find_last_of("."));
            std::string currPointCloudSavePath = pointCloudFolderPath.toStdString() + "/" + baseFilename + ".ply";
            pcl::io::savePLYFileASCII(currPointCloudSavePath, *pointCloud);
            qDebug() << "POINT CLODU SIOZE " << pointCloud->width;
        }

       /*if (imagesAggr.size() == imagesDepth.size()) {
            QString pointCloudFolderPath = experFolderPath + "/Raven/PointCloud";
            if (!QDir(pointCloudFolderPath).exists()) {
                QDir().mkdir(pointCloudFolderPath);
            }
            for (int i{ 0 }; i < imagesAggr.size(); i++) {
                currDepthImg = cv::imread(imagesDepth[i], -1);
                aggrImg = cv::imread(imagesAggr[i]);
                pointCloud = pointCloudGenerationForOnePair(currDepthImg, aggrImg, m_cameraMatrix);
                std::string baseFilename = imagesAggr[i].substr(imagesAggr[i].find_last_of("/\\") + 1);
                baseFilename = baseFilename.substr(0, baseFilename.find_last_of("."));
                std::string currPointCloudSavePath = pointCloudFolderPath.toStdString() + "/" + baseFilename + ".ply";
                pcl::io::savePLYFileASCII(currPointCloudSavePath, *pointCloud);
            }
        }*/



    }

}


void PointCloudGenerationTriEye::runCalibrationDataSelectionSlot()
{
    this->setDisabled(true);
    m_calibDataSelectionWidget->show();
    return;

}

void PointCloudGenerationTriEye::closeDataSelectionWidget()
{
    m_dataSelectionWidget->hide();
    this->setDisabled(false);
 }


void PointCloudGenerationTriEye::closeCalibDataSelectionWidget()
{
    m_calibDataSelectionWidget->hide();
    this->setDisabled(false);
    m_cameraMatrix = m_calibDataSelectionWidget->getCalibCameraMat();
    m_distCoeffs = m_calibDataSelectionWidget->getCalibDistMat();
}

PointCloudGenerationTriEye::~PointCloudGenerationTriEye()
{}
