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

class DataSelectionWidget : public QWidget
{
    Q_OBJECT
public:
    explicit DataSelectionWidget(QWidget* parent = nullptr);
    void closeEvent(QCloseEvent* ev);
    unsigned int getSelectedDataType();
    QString getSimpleDepthPath() const;
    std::pair<QString, QString> getDepthAndAggrImagePath() const;
    QString getDepthImagesFolderPath() const;
    QString getDepthAndAggrImagesFolderPath() const;
    QString getExperimentFolderPath() const;
    ~DataSelectionWidget();


signals:
    void widgetClosed();

private:
    QPushButton* m_selectSimpleDepthImageButton;
    QPushButton* m_selectPairOfDepthAndAggImageButton;
    QPushButton* m_selectFolderWithMultDepthImagesButton;
    QPushButton* m_selectFolderWithMultipleDepthAndAggImagePairsButton;
    QPushButton* m_selectExperimentFolderButton;
    QPushButton* m_applySelectedDataButton;
    QGridLayout* m_gridLayout;
    QString m_simpleDepthImagePath;
    std::pair<QString, QString> m_depthAndAggrImagePath;
    QString m_depthImagesFolderPath;
    QString m_depthAndAggrImagesFolderPath;
    QString m_experimentFolderPath;
    unsigned int m_selectedDataType;
    QHBoxLayout* m_progressBarLayout;
    QProgressBar* m_progressBar;
        //Screen size 
    QRect m_screenSize;

private:
    bool isCurrentFolderExperiment(const QString& currentFolderPath);

public slots:
    void selectSimpleDepthImageSlot();
    void selectPairOfDepthAndAggImageSlot();
    void selectFolderWithMultDepthImagesSlot();
    void selectFolderWithMultipleDepthAndAggImagePairsSlot();
    void selectExperimentFolderSlot();
    void applySelectedDataSlot();


};
