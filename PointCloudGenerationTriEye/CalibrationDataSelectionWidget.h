#pragma once


#include <QtWidgets/QMainWindow>
#include <Qlayout>
#include <QPushbutton>
#include <QLineEdit>
#include <QLabel>
#include <QGridLayout>
#include <QCheckbox>
#include <QFormLayout>
#include <QProgressBar>
#include <QCloseEvent>
#include <QTimer>
#include <QString>

#include <opencv2/core/core.hpp>


//Camera calibration widget with buttons 

class CalibrationDataSelectionWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CalibrationDataSelectionWidget(QWidget* parent = nullptr);
    void closeEvent(QCloseEvent* ev);
    void updateProgress(const unsigned int& updateSeconds);

    ~CalibrationDataSelectionWidget();


signals:
    void widgetClosed();
    void calibrationFinished();


private:
   

private:
    QPushButton* m_selectCalibrationFileButton;
    QPushButton* m_selectImagesForCalibButton;
    QGridLayout* m_gridLayout;
    QGridLayout* m_gridLayoutForCalibSett;

    QProgressBar* m_progressBar;
    QVBoxLayout* m_progressBarLayout;

    QLineEdit* m_lineForSquareSize;
    QLabel* m_squareSizeLabel;


    //Set checkerboard width and height (corners count)
    QLineEdit* m_lineForCheckerBoardWidth;
    QLabel* m_checkerboardWidthLabel;

    QLineEdit* m_lineForCheckerBoardHeight;
    QLabel* m_checkerboardHeightLabel;

    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;

    //Screen size 
    QRect m_screenSize;



public slots:
    void selectCalibrationFileSlot();
    void selectImagesForCalibrationSlot();


public:
    cv::Mat getCalibCameraMat() const;
    cv::Mat getCalibDistMat() const;
};

