#include <QFileDialog>
#include <QDesktopWidget>
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


    QDesktopWidget* widget = qApp->desktop();
    m_screenSize = widget->availableGeometry(widget->primaryScreen());
    
   


    qDebug() << " desktop screen size " << m_screenSize.height() << "   " << m_screenSize.width();
  
    m_mainLayout = new QVBoxLayout;
    //Initialize push buttons

    m_runCalibrationButton = new QPushButton("Run Calibration");
    m_selectImagesFolderButton= new QPushButton("Select Images For Calibration");
    m_selectDepthAndAggrDataButton = new QPushButton("Input Depth Data");
    m_selectCalibrationDataButton = new QPushButton("Input Calibration Data");
    m_runPointCloudGenerationButon = new QPushButton("Point Cloud Generation");
 

    //Checkbox and label for applying depth filter
    m_checkboxForDepthFiler = new QCheckBox;
    m_depthFilterLabel = new QLabel;
    m_depthFilterLabel->setText("Depth Filer");

    //Checkbox and label for applying point cloud filter
    m_checkboxForPointCloudFiler = new QCheckBox;
    m_pointCloudFilterLabel = new QLabel;
    m_pointCloudFilterLabel->setText("Point Cloud Filer");


    //Set square size layout in CM
    m_squareSizeLabel = new QLabel;
    m_squareSizeLabel->setText("Square Size in cm");

    m_lineForSquareSize = new QLineEdit;
   
    
    QRegExpValidator* rxv = new QRegExpValidator(QRegExp("\\d*"), this);
    m_lineForSquareSize->setText(0);
    m_lineForSquareSize->setValidator(rxv);

    //Set checkerboard  width from corners
    m_checkerboardWidthLabel = new QLabel;
    m_checkerboardWidthLabel->setText("Checkerboard Width");

    m_lineForCheckerBoardWidth = new QLineEdit;
    m_lineForCheckerBoardWidth->setValidator(rxv);

    //Set checkerboard height in corners
    m_checkerboardHeightLabel = new QLabel;
    m_checkerboardHeightLabel->setText("Checkerboard Height");

    m_lineForCheckerBoardHeight = new QLineEdit;
    m_lineForCheckerBoardHeight->setValidator(rxv);


    //Calibratio layout
    m_calibrationLayout = new QHBoxLayout;
    m_calibrationLayout->addWidget(m_runCalibrationButton);
    m_calibrationLayout->addWidget(m_selectImagesFolderButton);

    //Progress bar for showin the process
    m_progressBar = new QProgressBar;
    m_progressBar->setMinimum(0);
    m_progressBar->setMaximum(100);


    //Add items to calibration widget
    m_calibrationLayout->addWidget(m_squareSizeLabel);
    m_calibrationLayout->addWidget(m_lineForSquareSize);
    m_calibrationLayout->addWidget(m_checkerboardWidthLabel);
    m_calibrationLayout->addWidget(m_lineForCheckerBoardWidth);
    m_calibrationLayout->addWidget(m_checkerboardHeightLabel);
    m_calibrationLayout->addWidget(m_lineForCheckerBoardHeight);


    m_progressBarLayout = new QHBoxLayout;
    m_progressBarLayout->addWidget(m_progressBar);


    //Add items to point cloud generation widget 
    m_pointCloudGenerationLayout = new QHBoxLayout;
    m_pointCloudGenerationLayout->addWidget(m_selectDepthAndAggrDataButton);
    m_pointCloudGenerationLayout->addWidget(m_selectCalibrationDataButton);
    m_pointCloudGenerationLayout->addWidget(m_runPointCloudGenerationButon);
    m_pointCloudGenerationLayout->addSpacing(100);
    m_pointCloudGenerationLayout->addWidget(m_depthFilterLabel);
    m_pointCloudGenerationLayout->setSpacing(1);
    m_pointCloudGenerationLayout->addWidget(m_checkboxForDepthFiler);
    m_pointCloudGenerationLayout->addWidget(m_pointCloudFilterLabel);
    m_pointCloudGenerationLayout->addWidget(m_checkboxForPointCloudFiler);

    
    // Add components to main layout
    m_mainLayout->addLayout(m_calibrationLayout);
    m_mainLayout->addLayout(m_progressBarLayout);
    m_mainLayout->addLayout(m_pointCloudGenerationLayout);
    
    this->setLayout(m_mainLayout);
   
    this->setMinimumWidth(m_screenSize.width() / 2.5);
    this->setMinimumHeight(m_screenSize.height() / 2.5);
    this->show();

 
    //Connect signals to slots
    connect(m_selectImagesFolderButton, SIGNAL(clicked()), this, SLOT(selectCheckerboardImagesFolderSlot()));
    connect(m_runCalibrationButton, SIGNAL(clicked()), this, SLOT(runCalibrationSlot()));
    connect(m_selectDepthAndAggrDataButton, SIGNAL(clicked()), this, SLOT(runDepthAndAggrDataSelectionSlot()));
    connect(m_dataSelectionWidget, SIGNAL(widgetClosed()), this, SLOT(closeDataSelectionWidget()));  
    connect(m_selectCalibrationDataButton, SIGNAL(clicked()), this, SLOT(runCalibrationDataSelectionSlot()));
    connect(m_calibDataSelectionWidget, SIGNAL(widgetClosed()), this, SLOT(closeCalibDataSelectionWidget()));
    connect(m_calibDataSelectionWidget, SIGNAL(calibrationFinished()), this, SLOT(closeCalibDataSelectionWidget()));
    connect(m_runPointCloudGenerationButon, SIGNAL(clicked()), this, SLOT(runPointCloudGenerationSlot()));
 

}

//Select folder for checkerboard calibration
void PointCloudGenerationTriEye::selectCheckerboardImagesFolderSlot()
{
    updateProgress(0);
    QFileDialog dialog;
   
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    dialog.setOption(QFileDialog::ShowDirsOnly, true);


    QString filename = dialog.getExistingDirectory(this, "Choose Folder");
    std::cout << "selected folder path " << filename.toStdString() << std::endl;
    m_checkerboardImagesFolderPath = filename;

}

//Update progress bar while processing calibration or point cloud generation
void PointCloudGenerationTriEye::updateProgress(const unsigned int& updateSeconds)
{
    m_progressBar->setValue(updateSeconds);
  
}

//Run calibration 
void PointCloudGenerationTriEye::runCalibrationSlot()
{

    updateProgress(0);
    // Creating vector to store vectors of 3D points for each checkerboard image
    QMessageBox messageBox;
    m_lineForSquareSize->update();
    m_lineForCheckerBoardWidth->update();
    m_lineForCheckerBoardHeight->update();
    
    if (m_lineForSquareSize->text().toUInt() == 0) {       
        messageBox.critical(0, "Error", "Please specify checkerboard square size !!!");               
        messageBox.show();
        return;
    }

    if (m_lineForCheckerBoardWidth->text().toUInt() == 0) {
        messageBox.critical(0, "Error", "Please specify checkerboard width !!!");     
        messageBox.show();
        return;
    }

    if (m_lineForCheckerBoardHeight->text().toUInt() == 0) {
        messageBox.critical(0, "Error", "Please specify checkerboard height !!!");       
        messageBox.show();
        return;
    }

    if (m_checkerboardImagesFolderPath == "") {
        messageBox.critical(0, "Error", "Please specify folder path for checkerboard images!");    
        messageBox.show();
        return;
    }
  
    std::vector<std::vector<cv::Point3f> > objpoints;
    float square_size = m_lineForSquareSize->text().toUInt();

    unsigned int checkerboardWidth = m_lineForCheckerBoardWidth->text().toUInt() - 1;
    unsigned int checkerboardHeight = m_lineForCheckerBoardHeight->text().toUInt() - 1;
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
    std::string path = m_checkerboardImagesFolderPath.toStdString();

    path += "/*.tiff";
    cv::glob(path, images);

    cv::Mat frame, gray;

    // vector to store the pixel coordinates of detected checker board corners

    std::vector<cv::Point2f> corner_pts;

    bool success;

    // Looping over all the images in the directory
   
    if (images.size() == 0) {
        messageBox.critical(0, "Error", "Specified folder does not contain tiff images !!!");
        //messageBox.setFixedSize(500, 200);
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

        bool isCheckerboardSizeCorrect = cv::checkChessboard(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]));
        std::cout << " is checkerboard  detected correctly "<< CHECKERBOARD[0]<<"  "<<CHECKERBOARD[1] <<"  "<< isCheckerboardSizeCorrect << std::endl;;
    
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        std::cout << "After find checssboard corners function \n";

    

        if (success)

        {

          //Initial working version  cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 40, 0.001);

            // refining pixel coordinates for given 2d points.

           //Initial working version cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            
            cv::cornerSubPix(gray, corner_pts, cv::Size(3, 3), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board

            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            if (objpoints.size() < 50 ) {

                objpoints.push_back(objp);

                imgpoints.push_back(corner_pts);
            }

        }
        if (success) {
            cv::imshow("Image", frame);

            cv::waitKey(10);
        }
        

    }

    cv::destroyAllWindows();
    cv::Mat cameraMatrix, distCoeffs, R, T;

    if (objpoints.size() == 0 || imgpoints.size() == 0) {
        messageBox.critical(0, "Error", "Could not detect checkerboard corners with specified width and height !!!");
        messageBox.show();
        updateProgress(0);
        return;

    }
    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);
    updateProgress(100);
    QString calibDataResFileName = QFileDialog::getSaveFileName(this, "Save file", "", ".yml");
    updateProgress(0);
    cv::FileStorage calibDatSt(calibDataResFileName.toStdString() + ".yml", cv::FileStorage::WRITE);
    calibDatSt << "K"<<cameraMatrix;
    calibDatSt << "D" << distCoeffs;
    calibDatSt.release();

    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;
    std::cout << "Rotation vector : " << R << std::endl;

    std::cout << "Translation vector : " << T << std::endl;

}

//Point cloud filter with outer removal
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudGenerationTriEye::outlierRemoval(const   pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud)//, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud)
 {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud  (new  pcl::PointCloud<pcl::PointXYZRGB>());
    // Create the filtering object
    //Outlier removal filter

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(inputCloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.05);
    sor.filter(*outputCloud);    
    return outputCloud;

}
//Get all depth images in mixed folder
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
            depthImagePaths.push_back(baseFilename.c_str());
        }
       
    }

    return depthImagePaths;
}
//Get all aggr images in mixed folder
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
        if (baseFileTypeName == "agg") {
            aggrImagePaths.push_back(baseFilename.c_str());
        }

    }

    return aggrImagePaths;
}


//Get pair of depth and aggr images in experiment folder
QVector<QPair<QString, QString>> PointCloudGenerationTriEye::getDepthAndAggrPairsForExperimentFolder(const std::vector<cv::String>& depthImages, const std::vector<cv::String>& aggrImages)
{
    updateProgress(0);
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

//Point cloud generation for one depth and aggr image pair.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudGenerationTriEye::pointCloudGenerationForOnePair(const cv::Mat& depthImg, const cv::Mat& aggrImage, const cv::Mat& cameraMatrix)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new  pcl::PointCloud<pcl::PointXYZRGB>());
    cv::Mat inputDepthImage = depthImg.clone();

    if (m_checkboxForDepthFiler->isChecked()) {
        inputDepthImage.convertTo(inputDepthImage, CV_32FC1);
        cv::Mat outputMat;
        cv::bilateralFilter(inputDepthImage, outputMat, 35, 135, 135);
        inputDepthImage = outputMat;
        inputDepthImage.convertTo(inputDepthImage, CV_16SC1);


    }

    cv::Mat depthInFLoat = cv::Mat(inputDepthImage.rows, inputDepthImage.cols, CV_32F);
    float minDepthValue = 100000;
    float maxDepthValue = 00;
    for (int i = 0; i < depthInFLoat.rows; ++i) {
        for (int j = 0; j < depthInFLoat.cols; ++j) {
            depthInFLoat.at<float>(i, j) = (float)(inputDepthImage.at<unsigned short>(i, j)) / 1000.0;
        }
    }
    for (int i = 0; i < depthInFLoat.rows; ++i) {
        for (int j = 0; j < depthInFLoat.cols; ++j) {         
            if (depthInFLoat.at<float>(i, j) > maxDepthValue) {
                maxDepthValue = depthInFLoat.at<float>(i, j);
            }
            if (depthInFLoat.at<float>(i, j) < minDepthValue) {
                minDepthValue = depthInFLoat.at<float>(i, j);
            }              
        }
    }
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
        
    if ( m_checkboxForPointCloudFiler->isChecked()) {
        //Apply guided filter to point cloud if checkbox is selected
        float radius = 0.05;
        float epsilon = 0.5;
        guidedFilter(pointCloud, radius, epsilon);          
        std::cout << "Point cloud filtered points size " << pointCloud->width << "   " << pointCloud->height << std::endl;
        return pointCloud;           
    }
    else {
        return pointCloud;
    }
}

//Run depth and aggr images selection  slot
void PointCloudGenerationTriEye::runDepthAndAggrDataSelectionSlot()
{
    updateProgress(0);
    this->setDisabled(true);

    m_dataSelectionWidget->show();

    return;
   

}

//Guided filter for point cloud post processing
void PointCloudGenerationTriEye::guidedFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius, double epsilon) 
{

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    std::cout << "Point cloud size before kdtree " << cloud->size() << std::endl;
    kdtree.setInputCloud(cloud);
    kdtree.setEpsilon(epsilon);

    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        pcl::PointXYZRGB searchPoint = cloud->points[i];
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3)
        {

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr neigbors(new pcl::PointCloud<pcl::PointXYZRGB>);
            Eigen::MatrixXd neighbors_as_matrix(3, pointIdxRadiusSearch.size());

            for (std::size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
            {
                neigbors->points.push_back(cloud->points[pointIdxRadiusSearch[j]]);
                neighbors_as_matrix(0, j) = cloud->points[pointIdxRadiusSearch[j]].x;
                neighbors_as_matrix(1, j) = cloud->points[pointIdxRadiusSearch[j]].y;
                neighbors_as_matrix(2, j) = cloud->points[pointIdxRadiusSearch[j]].z;
            }

            Eigen::Vector3d mean;
            mean = neighbors_as_matrix.rowwise().mean();
            neighbors_as_matrix.transposeInPlace();
            Eigen::MatrixXd centered = neighbors_as_matrix.rowwise() - neighbors_as_matrix.colwise().mean();
            Eigen::MatrixXd cov = (centered.adjoint() * centered) / double(neighbors_as_matrix.rows() - 1);

            Eigen::MatrixXd e = (cov + epsilon * Eigen::MatrixXd::Identity(3, 3));
            e = e.inverse();

            Eigen::MatrixXd A = cov * e;
            Eigen::MatrixXd b = mean - A * mean;

            Eigen::Vector3d searchPointEigenType;
            searchPointEigenType[0] = searchPoint.x;
            searchPointEigenType[1] = searchPoint.y;
            searchPointEigenType[2] = searchPoint.z;

            searchPointEigenType = A * searchPointEigenType + b;

            searchPoint.x = searchPointEigenType[0];
            searchPoint.y = searchPointEigenType[1];
            searchPoint.z = searchPointEigenType[2];
            cloud->points[i] = searchPoint;
        }
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

}


//Point cloud generation

void PointCloudGenerationTriEye::runPointCloudGenerationSlot()
{

    std::cout << "Start point cloud  generation !!!!!!!!!!!!!!!!!  \n";
    updateProgress(0);
    QMessageBox messageBox;
    


    if (m_dataSelectionWidget->getSelectedDataType() == 0) {
        messageBox.information(0, "Info", "PLease select depth data!!!");
        messageBox.show();
        return;

    }
    if (m_cameraMatrix.rows == 0 || m_cameraMatrix.cols == 0) {
        messageBox.information(0, "Info", "Please select calibration file or do calibration!!!");
        messageBox.show();
        return;
    }

    cv::Mat aggrImg;
    cv::Mat currDepthImg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new  pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB> pointCloudFiltered;
    
    if (m_dataSelectionWidget->getSelectedDataType() == 1  || m_dataSelectionWidget->getSelectedDataType() == 2) {
        
        if (m_dataSelectionWidget->getSelectedDataType() == 1) {
            currDepthImg = cv::imread(m_dataSelectionWidget->getSimpleDepthPath().toStdString(), -1);
        }
        else if (m_dataSelectionWidget->getSelectedDataType() == 2) {
            aggrImg = cv::imread(m_dataSelectionWidget->getDepthAndAggrImagePath().second.toStdString());
            currDepthImg = cv::imread(m_dataSelectionWidget->getDepthAndAggrImagePath().first.toStdString(),-1);

        }
        
        pointCloud = (pointCloudGenerationForOnePair(currDepthImg, aggrImg, m_cameraMatrix)); 
        
     
        updateProgress(100);
        
        QString pointCloudResFileName = QFileDialog::getSaveFileName(this, "Save point cloud file", "", ".ply");   
        cv::waitKey(100);
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
            messageBox.show();
            return;
        }
        for (int i{ 0 }; i < images.size(); i++) {
            currDepthImg = cv::imread(images[i], -1);
            pointCloud = pointCloudGenerationForOnePair(currDepthImg, aggrImg, m_cameraMatrix);           
            
            std::string currPointCloudSavePath = images[i].substr(0, images[i].find_last_of(".")) + ".ply";
            pcl::io::savePLYFileASCII(currPointCloudSavePath, *pointCloud);  
          
            float currentValueForProgressBar = (i + 1) * 100 / images.size();
            std::cout << "Current depth processing progress bar " <<currentValueForProgressBar<<"\n";
            m_progressBar->show();
            updateProgress(currentValueForProgressBar);
            cv::waitKey(10);         
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
       

        }


        for (int i = 0; i < aggrImagesInSelectedFolder.size(); ++i) {
            QString currentAggrImageName = aggrImagesInSelectedFolder[i];
            std::string imageIndexName = cv::String(currentAggrImageName.toStdString()).substr(cv::String(currentAggrImageName.toStdString()).find_first_of("_") + 1);
            aggrImageEndings.push_back(QString(imageIndexName.c_str()));
         

        }
        if (depthImageEndings.size() == 0) {
            messageBox.critical(0, "Error", "Specified Folder Does Not Contain Depth Images !!!");
    
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
            cv::waitKey(10);
            updateProgress((i +1) * 100 / depthImagesInSelectedFolder.size());
            

        }

    }
    else if (m_dataSelectionWidget->getSelectedDataType() == 5) {
        //Selected experiment folder
        QString experFolderPath = m_dataSelectionWidget->getExperimentFolderPath();
        std::string pathDepth = experFolderPath.toStdString() + "/DepthImage";
        std::vector<cv::String> imagesDepth;
        pathDepth += "/*.tiff";
        cv::glob(pathDepth, imagesDepth);

        std::string pathAggr = experFolderPath.toStdString() + "/AggrImage";
        std::vector<cv::String> imagesAggr;
        pathAggr += "/*.tiff";
        cv::glob(pathAggr, imagesAggr);


        QVector<QPair<QString, QString>>  depthAndAggrImagesPairsVec =  getDepthAndAggrPairsForExperimentFolder(imagesDepth, imagesAggr);
      
        if (imagesDepth.size() == 0) {
            messageBox.critical(0, "Error", "Specified Depth Images folder Does Not Contain Tiff Images !!!");
            messageBox.show();
            return;
        }

        QString pointCloudFolderPath = experFolderPath + "/PointCloud";
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
            cv::waitKey(10);
            updateProgress((g + 1)* 100 / depthAndAggrImagesPairsVec.size());
         
        }

    }

    return ;
}


void PointCloudGenerationTriEye::runCalibrationDataSelectionSlot()
{
    updateProgress(0);
    this->setDisabled(true);
    m_calibDataSelectionWidget->updateProgress(0);
    m_calibDataSelectionWidget->show();
  
    return;

}

void PointCloudGenerationTriEye::closeDataSelectionWidget()
{
    updateProgress(0);
    m_dataSelectionWidget->hide();
    this->setDisabled(false);
 }


void PointCloudGenerationTriEye::closeCalibDataSelectionWidget()
{
    updateProgress(0);
    m_calibDataSelectionWidget->hide();
    this->setDisabled(false);
    m_cameraMatrix = m_calibDataSelectionWidget->getCalibCameraMat();
    m_distCoeffs = m_calibDataSelectionWidget->getCalibDistMat();
}

PointCloudGenerationTriEye::~PointCloudGenerationTriEye()
{}
