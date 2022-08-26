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

class CalibrationDataSelectionWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CalibrationDataSelectionWidget(QWidget* parent = nullptr);
    void closeEvent(QCloseEvent* ev);
    /*unsigned int getSelectedDataType();
    QString getSimpleDepthPath() const;
    std::pair<QString, QString> getDepthAndAggrImagePath() const;
    QString getDepthImagesFolderPath() const;
    QString getDepthAndAggrImagesFolderPath() const;
    QString getExperimentFolderPath() const;*/
    ~CalibrationDataSelectionWidget();


signals:
    void widgetClosed();
    void calibrationFinished();

private:
    QPushButton* m_selectCalibrationFileButton;
    QPushButton* m_selectImagesForCalibButton;
    QGridLayout* m_gridLayout;
    QGridLayout* m_gridLayoutForCalibSett;


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

    /*QPushButton* m_selectFolderWithMultDepthImagesButton;
    QPushButton* m_selectFolderWithMultipleDepthAndAggImagePairsButton;
    QPushButton* m_selectExperimentFolderButton;
    QPushButton* m_applySelectedDataButton;
    QGridLayout* m_gridLayout;
    QString m_simpleDepthImagePath;
    std::pair<QString, QString> m_depthAndAggrImagePath;
    QString m_depthImagesFolderPath;
    QString m_depthAndAggrImagesFolderPath;
    QString m_experimentFolderPath;
    unsigned int m_selectedDataType;*/

public slots:
    void selectCalibrationFileSlot();
    void selectImagesForCalibrationSlot();
    /*void selectSimpleDepthImageSlot();
    void selectPairOfDepthAndAggImageSlot();
    void selectFolderWithMultDepthImagesSlot();
    void selectFolderWithMultipleDepthAndAggImagePairsSlot();
    void selectExperimentFolderSlot();
    void applySelectedDataSlot();*/

public:
    cv::Mat getCalibCameraMat() const;
    cv::Mat getCalibDistMat() const;
};

