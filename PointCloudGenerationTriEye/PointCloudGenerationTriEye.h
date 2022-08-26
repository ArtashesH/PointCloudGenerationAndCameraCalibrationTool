#pragma once

#include <QtWidgets/QMainWindow>
#include <Qlayout>
#include "ui_PointCloudGenerationTriEye.h"
#include <QPushbutton>
#include <QLineEdit>
#include <QLabel>
#include <QCheckbox>
#include <QFormLayout>
#include <QProgressBar>
#include <QTimer>
#include <QPair>
#include <QRect>
#include <QVector>


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>


#include <opencv2/core/core.hpp>


#include "DataSelectionWidget.h"
#include "CalibrationDataSelectionWidget.h"

class PointCloudGenerationTriEye : public QWidget
{
    Q_OBJECT

public:
    PointCloudGenerationTriEye(QWidget *parent = nullptr);
    void updateProgress(const unsigned int& updateSeconds);
    ~PointCloudGenerationTriEye();


private:
    void outlierRemoval(const   pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud);
    QVector<QString> getAllDepthImagePathsInMixedFolder(const QString& folderPath);
    QVector<QString> getAllAggrImagePathsInMixedFolder(const QString& folderPath);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudGenerationForOnePair(const cv::Mat& depthImg, const cv::Mat& aggrImage, const cv::Mat& cameraMatrix);
    QVector<QPair<QString, QString>> getDepthAndAggrPairsForExperimentFolder(const std::vector<cv::String>& depthImages, const std::vector<cv::String>& aggrImages);
public slots:
    void selectCheckerboardImagesFolderSlot();
    void runCalibrationSlot();
    void runDepthAndAggrDataSelectionSlot();
    void runPointCloudGenerationSlot();
    void closeDataSelectionWidget();
    void runCalibrationDataSelectionSlot();
    void closeCalibDataSelectionWidget();


private:
    Ui::PointCloudGenerationTriEyeClass ui;
    //Main layout
    QVBoxLayout* m_mainLayout;
    //Layout for calibration items
    QHBoxLayout* m_calibrationLayout;
    //Layout for point cloud generation items
    QHBoxLayout* m_pointCloudGenerationLayout;
    QHBoxLayout* m_progressBarLayout;

    //QGridLayout*  layout;

    //Buttons 
    QPushButton* m_selectImagesFolderButton;
    QPushButton* m_runCalibrationButton;
    QPushButton* m_selectDepthAndAggrDataButton;
    QPushButton* m_selectCalibrationDataButton;
    QPushButton* m_runPointCloudGenerationButon;

    //Checkbox for filtering point cloud
    QCheckBox* m_checkboxForDepthFiler;
    QLabel* m_depthFilterLabel;


    //Checkbox for point cloud filter
    QCheckBox* m_checkboxForPointCloudFiler;
    QLabel* m_pointCloudFilterLabel;

    //Set Chckerboard square real size 
    QLineEdit* m_lineForSquareSize;
    QLabel* m_squareSizeLabel;


    //Set checkerboard width and height (corners count)
    QLineEdit* m_lineForCheckerBoardWidth;
    QLabel* m_checkerboardWidthLabel;

    QLineEdit* m_lineForCheckerBoardHeight;
    QLabel* m_checkerboardHeightLabel;


    QString m_checkerboardImagesFolderPath;
    QProgressBar* m_progressBar;

    //Data selection widget for point cloud generation 
    DataSelectionWidget* m_dataSelectionWidget;
    CalibrationDataSelectionWidget* m_calibDataSelectionWidget;


    //Screen size 
    QRect m_screenSize;

    //Calibration Data
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;

};
