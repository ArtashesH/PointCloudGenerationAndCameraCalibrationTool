
#include <QDebug>
#include <QValidator>
#include <QRegExp>
#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QDirIterator>
#include <QdesktopWidget>
#include <QRect>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <iostream>

#include "CalibrationDataSelectionWidget.h"

CalibrationDataSelectionWidget::CalibrationDataSelectionWidget(QWidget* parent)
    : QWidget(parent)
{
    

    //Camera calibration and  data selection widget 
    
    QScreen* screen = QGuiApplication::primaryScreen();
    QDesktopWidget* widget = qApp->desktop();
    m_screenSize = widget->availableGeometry(widget->primaryScreen());
 
    this->setWindowTitle("Calibration Data Selection Widget");

    m_gridLayout = new QGridLayout;
    m_gridLayoutForCalibSett = new QGridLayout;

    //Select  calibration data (.yaml file) button
    m_selectCalibrationFileButton = new QPushButton;
    m_selectCalibrationFileButton->setText("Select Calibration File");
    m_selectCalibrationFileButton->setFixedHeight(m_screenSize.height() / 20);
    m_selectCalibrationFileButton->setFixedWidth(m_screenSize.width() / 10);

    //Select checkerboard images button
    m_selectImagesForCalibButton = new QPushButton;
    m_selectImagesForCalibButton->setText("Select Images For Calibration");
    m_selectImagesForCalibButton->setFixedHeight(m_screenSize.height() / 20);
    m_selectImagesForCalibButton->setFixedWidth(m_screenSize.width() / 10);

    //Set square size in Cm label 
    m_squareSizeLabel = new QLabel;
    m_squareSizeLabel->setText("Square Size in cm");

    // Progres bar for showing calibration process
    m_progressBar = new QProgressBar;
    m_progressBar->setMinimum(0);
    m_progressBar->setMaximum(100);



    m_progressBarLayout = new QVBoxLayout;
    m_progressBarLayout->addWidget(m_progressBar);

    m_lineForSquareSize = new QLineEdit;

    //Check square size always greate than zero
    QRegExpValidator* rxv = new QRegExpValidator(QRegExp("\\d*"), this);
    m_lineForSquareSize->setText(0);
    m_lineForSquareSize->setValidator(rxv);

    //Set checkerboard corners count width
    m_checkerboardWidthLabel = new QLabel;
    m_checkerboardWidthLabel->setText("Checkerboard Width");

    m_lineForCheckerBoardWidth = new QLineEdit;
    m_lineForCheckerBoardWidth->setValidator(rxv);

    //Set checkerboard corners count height
    m_checkerboardHeightLabel = new QLabel;
    m_checkerboardHeightLabel->setText("Checkerboard Height");

    m_lineForCheckerBoardHeight = new QLineEdit;
    m_lineForCheckerBoardHeight->setValidator(rxv);



    m_gridLayoutForCalibSett->addWidget(m_squareSizeLabel, 0, 0);
    m_gridLayoutForCalibSett->addWidget(m_lineForSquareSize, 0, 1);

    m_gridLayoutForCalibSett->addWidget(m_checkerboardWidthLabel, 1, 0);
    m_gridLayoutForCalibSett->addWidget(m_lineForCheckerBoardWidth, 1, 1);


    m_gridLayoutForCalibSett->addWidget(m_checkerboardHeightLabel, 2, 0); 
    m_gridLayoutForCalibSett->addWidget(m_lineForCheckerBoardHeight, 2, 1);

    m_gridLayout->addWidget(m_selectCalibrationFileButton, 0, 0);
    m_gridLayout->addWidget(m_selectImagesForCalibButton, 0, 1); 
    m_gridLayout->addLayout(m_gridLayoutForCalibSett, 0, 2);
     

    m_progressBarLayout->addLayout(m_gridLayout);

    this->setLayout(m_progressBarLayout);

    //Connect bottons to slots
    connect(m_selectCalibrationFileButton, SIGNAL(clicked()), this, SLOT(selectCalibrationFileSlot()));
    connect(m_selectImagesForCalibButton, SIGNAL(clicked()), this, SLOT(selectImagesForCalibrationSlot()));
   
}

void CalibrationDataSelectionWidget::closeEvent(QCloseEvent* ev)
{

    QWidget::closeEvent(ev);
    blockSignals(false);
    ev->ignore();
    emit widgetClosed();
}


void CalibrationDataSelectionWidget::selectCalibrationFileSlot()
{
    QFileDialog* fileDialog = new QFileDialog;
    QString calibFilePath = fileDialog->getOpenFileName(this, "Calibration File", "File", "Images(*.yml)");
    QMessageBox messageBox;


    if (calibFilePath == "")
    {
        messageBox.critical(0, "Error", "Calibration file was not selected !!!");      
        messageBox.show();
        return;
    }


    cv::FileStorage calibDatSt;
    calibDatSt.open(calibFilePath.toStdString(), cv::FileStorage::READ);
  
    calibDatSt["K"] >> m_cameraMatrix;// cameraKMatrix;    
    calibDatSt["D"] >> m_distCoeffs;//  distCoeff;
    emit calibrationFinished();
}


void CalibrationDataSelectionWidget::selectImagesForCalibrationSlot()
{
    QMessageBox messageBox;
    updateProgress(0);
    m_lineForSquareSize->update();
    m_lineForCheckerBoardWidth->update();
    m_lineForCheckerBoardHeight->update();
    if (m_lineForSquareSize->text().toUInt() == 0) {
        messageBox.critical(0, "Error", "PLease specify checkerboard square size !!!");     
        messageBox.show();
        return;
    }


    if (m_lineForCheckerBoardWidth->text().toUInt() == 0) {
        messageBox.critical(0, "Error", "PLease specify checkerboard width !!!");      
        messageBox.show();
        return;
    }
    if (m_lineForCheckerBoardHeight->text().toUInt() == 0) {
        messageBox.critical(0, "Error", "PLease specify checkerboard height !!!");    
        messageBox.show();
        return;
    }

    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    dialog.setOption(QFileDialog::ShowDirsOnly, false);
    QString filename = dialog.getExistingDirectory(this, "Choose Folder");
   
    std::vector<std::vector<cv::Point3f> > objpoints;
    float square_size = m_lineForSquareSize->text().toUInt();

    unsigned int checkerboardWidth = m_lineForCheckerBoardWidth->text().toUInt() - 1;
    unsigned int checkerboardHeight = m_lineForCheckerBoardHeight->text().toUInt() - 1; 
    int CHECKERBOARD[2]{ checkerboardWidth,checkerboardHeight };

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

    std::string path = filename.toStdString();

    path += "/*.tiff";
    cv::glob(path, images);

    cv::Mat frame, gray;

    // vector to store the pixel coordinates of detected checker board corners

    std::vector<cv::Point2f> corner_pts;

    bool success;


    // Looping over all the images in the directory

    if (images.size() == 0) {
        messageBox.critical(0, "Error", "Specified folder does not contain tiff images !!!");      
        messageBox.show();
        return;
    }
    for (int i{ 0 }; i < images.size(); i++)

    {
     
        frame = cv::imread(images[i]);        
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Finding checker board corners

        // If desired number of corners are found in the image then success = true 

        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);




        if (success)

        {

            //Initial working version  cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 40, 0.001);

            // refining pixel coordinates for given 2d points.

           //Initial working version cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            cv::cornerSubPix(gray, corner_pts, cv::Size(3, 3), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board

            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);


            if (objpoints.size() < 50) {
                
                objpoints.push_back(objp);
                imgpoints.push_back(corner_pts);
            }

        }
       
        updateProgress(i * 100 / images.size());
        if (success) {
            cv::imshow("Image", frame);
            cv::waitKey(10);
        }

    }

    cv::destroyAllWindows();

    cv::Mat cameraMatrix, distCoeffs, R, T;
    std::cout << "images for corener detection size  " << objpoints.size() << std::endl;

    if (objpoints.size() == 0 || imgpoints.size() == 0) {      
        messageBox.critical(0, "Error", "Could not detect checkerboard corners with specified width and height !!!");
        updateProgress(0);
        messageBox.show();
        return;
       
    }
    cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);
    updateProgress(100);
    m_cameraMatrix = cameraMatrix.clone();
    m_distCoeffs = distCoeffs.clone();
    QString calibDataResFileName = QFileDialog::getSaveFileName(this, "Save file", "", ".yml");
    updateProgress(0);
    cv::FileStorage calibDatSt(calibDataResFileName.toStdString() + ".yml", cv::FileStorage::WRITE);
    calibDatSt << "K" << cameraMatrix;
    calibDatSt << "D" << distCoeffs;
    calibDatSt.release();
    
    emit calibrationFinished();

}


void CalibrationDataSelectionWidget::updateProgress(const unsigned int& updateSeconds)
{
    m_progressBar->setValue(updateSeconds);

}


cv::Mat CalibrationDataSelectionWidget::getCalibCameraMat() const
{
    return m_cameraMatrix;
}


cv::Mat CalibrationDataSelectionWidget::getCalibDistMat() const
{
    return m_distCoeffs;
}


CalibrationDataSelectionWidget::~CalibrationDataSelectionWidget()
{

}