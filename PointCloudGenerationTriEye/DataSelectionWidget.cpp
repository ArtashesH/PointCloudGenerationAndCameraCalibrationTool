#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QDirIterator>
#include "DataSelectionWidget.h"

DataSelectionWidget::DataSelectionWidget(QWidget* parent)
    : QWidget(parent)
{
    this->setFixedHeight(400);
    this->setFixedWidth(500);
    this->setWindowTitle("Depth And Aggr Data Selection Widget");

    m_selectedDataType = 0;

    m_gridLayout = new QGridLayout;
    m_selectSimpleDepthImageButton = new QPushButton;
    m_selectSimpleDepthImageButton->setText("Simple Depth");

    m_selectPairOfDepthAndAggImageButton = new QPushButton;
    m_selectPairOfDepthAndAggImageButton->setText("Pair Depth and Agg");


    m_selectFolderWithMultDepthImagesButton = new QPushButton;
    m_selectFolderWithMultDepthImagesButton->setText("Folder With Multiple Depths");



    m_selectFolderWithMultipleDepthAndAggImagePairsButton = new QPushButton;
    m_selectFolderWithMultipleDepthAndAggImagePairsButton->setText("Folder With Multiple Depths and Aggs");


    m_selectExperimentFolderButton = new QPushButton;
    m_selectExperimentFolderButton->setText("Experiment Folder");


    m_applySelectedDataButton = new QPushButton;
    m_applySelectedDataButton->setText("Apply Selected");


    m_gridLayout->addWidget(m_selectSimpleDepthImageButton, 0, 0);
    m_gridLayout->addWidget(m_selectPairOfDepthAndAggImageButton, 0, 1);
    m_gridLayout->addWidget(m_selectFolderWithMultDepthImagesButton, 1, 0);
    m_gridLayout->addWidget(m_selectFolderWithMultipleDepthAndAggImagePairsButton, 2, 0);
    m_gridLayout->addWidget(m_selectExperimentFolderButton, 1, 1);
    //m_gridLayout->addWidget(m_applySelectedDataButton, 2, 1);

    this->setLayout(m_gridLayout);



    //Connect signal slots

    connect(m_selectSimpleDepthImageButton, SIGNAL(clicked()), this, SLOT(selectSimpleDepthImageSlot()));
    connect(m_selectPairOfDepthAndAggImageButton, SIGNAL(clicked()), this, SLOT(selectPairOfDepthAndAggImageSlot()));
    connect(m_selectFolderWithMultDepthImagesButton, SIGNAL(clicked()), this, SLOT(selectFolderWithMultDepthImagesSlot()));
    connect(m_selectFolderWithMultipleDepthAndAggImagePairsButton, SIGNAL(clicked()), this, SLOT(selectFolderWithMultipleDepthAndAggImagePairsSlot()));
    connect(m_selectExperimentFolderButton, SIGNAL(clicked()), this, SLOT(selectExperimentFolderSlot()));
    connect(m_applySelectedDataButton, SIGNAL(clicked()), this, SLOT(applySelectedDataSlot()));
}


//Slots implementation

void DataSelectionWidget::selectSimpleDepthImageSlot()
{
    QFileDialog* fileDialog = new QFileDialog;
    fileDialog->setDefaultSuffix("tiff");
    QString depthImagePath = fileDialog->getOpenFileName(this, "Depth Images", "Image", "Images(*.tiff)");
    m_simpleDepthImagePath = depthImagePath;
    QMessageBox messageBox;
    if (depthImagePath == "")
    {
        messageBox.critical(0, "Error", "Depth image was not selected !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }
    m_selectedDataType = 1;
    messageBox.information(0, "Info", "Selected one depth image !!!");
    messageBox.setFixedSize(500, 200);
    messageBox.show();
    emit widgetClosed();
   // this->hide();
  //  emit widgetClosed();


}

void DataSelectionWidget::selectPairOfDepthAndAggImageSlot()
{

    QFileDialog* fileDialog = new QFileDialog;
    fileDialog->setDefaultSuffix("tiff");
    QMessageBox messageBox;


    QString depthImagePath = fileDialog->getOpenFileName(this, "Depth Images", "Image", "Images(*.tiff)");
   
    if (depthImagePath == "")
    {
        messageBox.critical(0, "Error", "Depth image was not selected !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }

    QString aggrImagePath = fileDialog->getOpenFileName(this, "Aggr Images", "Image", "Images(*.tiff)");

    if (aggrImagePath == "")
    {
        messageBox.critical(0, "Error", "Aggr image was not selected !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }
    m_depthAndAggrImagePath.first = depthImagePath;
    m_depthAndAggrImagePath.second = aggrImagePath;
    m_selectedDataType = 2;
   // this->hide();
    messageBox.information(0, "Info", "Selected depth and aggr image pair !!!");
    messageBox.setFixedSize(500, 200);
    messageBox.show();
    emit widgetClosed();

}
void DataSelectionWidget::selectFolderWithMultDepthImagesSlot()
{
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    dialog.setOption(QFileDialog::ShowDirsOnly, false);
    QString depthImagesFolder = dialog.getExistingDirectory(this, "Choose Depth  Images Folder");
    QMessageBox messageBox;
    if (depthImagesFolder == "")
    {
        messageBox.critical(0, "Error", "Depth images folder  was not selected !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }
    QStringList nameFilter("*.tiff");
    QDir directory(depthImagesFolder);
    QStringList depthFilesInFolder = directory.entryList(nameFilter);
    if (depthFilesInFolder.size() == 0) {
        messageBox.critical(0, "Error", "Selected folder does not contains images !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }
    m_depthImagesFolderPath = depthImagesFolder;
    m_selectedDataType = 3;
    messageBox.information(0, "Info", "Selected folder with depth images !!!");
    messageBox.setFixedSize(500, 200);
    messageBox.show();
    emit widgetClosed();
   // this->hide();
   // emit widgetClosed();
   

}

void DataSelectionWidget::selectFolderWithMultipleDepthAndAggImagePairsSlot()
{
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    dialog.setOption(QFileDialog::ShowDirsOnly, false);
    QString depthAndAggrImagesFolder = dialog.getExistingDirectory(this, "Choose Depth And Aggr  Images Folder");
    QMessageBox messageBox;
    if (depthAndAggrImagesFolder == "")
    {
        messageBox.critical(0, "Error", "Images folder  was not selected !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }
    QStringList nameFilter("*.tiff");
    QDir directory(depthAndAggrImagesFolder);
    QStringList depthFilesInFolder = directory.entryList(nameFilter);
    if (depthFilesInFolder.size() == 0) {
        messageBox.critical(0, "Error", "Selected folder does not contains images !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }
    m_depthAndAggrImagesFolderPath = depthAndAggrImagesFolder;   
    m_selectedDataType = 4;
   
   // this->hide();
    emit widgetClosed();
}


bool DataSelectionWidget::isCurrentFolderExperiment(const QString& currentFolderPath)
{
    QMessageBox messageBox;
    if (currentFolderPath == "")
    {
        messageBox.critical(0, "Error", "Experiment folder  was not selected !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return false;
    }
    QDir const source(currentFolderPath);
    if (!source.exists()) {
        messageBox.critical(0, "Error", "Folder does not exists !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return false;
    }

    QStringList const folders = source.entryList(QDir::NoDot | QDir::NoDotDot | QDir::Dirs);

    QStringList const all = folders;
    QVector<QString> folderNamesVec;
    bool isFolderContainsRaven = false;
    for (QString const& name : all)
    {
        QString const fullPathName = currentFolderPath + QDir::separator() + name;
        if (QFileInfo(fullPathName).isDir())
        {
            folderNamesVec.push_back(fullPathName);
        }

    }
    for (int f = 0; f < folderNamesVec.size(); ++f) {
        if (QFileInfo(folderNamesVec[f]).fileName() == "Raven") {
            //if 
            isFolderContainsRaven = true;
            break;
        }
    }
    if (!isFolderContainsRaven) {
        messageBox.critical(0, "Error", "Selected folder does not contains  Raven folder !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return false;
    }
       
    QString ravenFolderPath = currentFolderPath + "/" + "Raven";
    QDir const sourceRaven(ravenFolderPath);
    QStringList const foldersRaven = sourceRaven.entryList(QDir::NoDot | QDir::NoDotDot | QDir::Dirs);

    QStringList const allR = foldersRaven;
    QVector<QString> folderNamesInsideRavenVec;
    bool isFolderContainsDepthAndAggr = false;
    for (QString const& name : allR)
    {
        QString const fullPathName = ravenFolderPath + QDir::separator() + name;
        if (QFileInfo(fullPathName).isDir())
        {
            if (name == "DepthImage" || name == "AggrImage") {
                folderNamesInsideRavenVec.push_back(name);
            }
        }

    }
    /*if (folderNamesInsideRavenVec.size() != 2) {
        messageBox.critical(0, "Error", "Raven folder does not contains two folders !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return false;
    }*/
    if (!(folderNamesInsideRavenVec.size() == 2 && ( (folderNamesInsideRavenVec[0] == "DepthImage" &&
                                                        folderNamesInsideRavenVec[1] == "AggrImage")  
                                                         || 
                                                        (folderNamesInsideRavenVec[1] == "DepthImage" &&
                                                          folderNamesInsideRavenVec[0] == "AggrImage")) )) {
        messageBox.critical(0, "Error", "Raven folder does not contains folder of depth and aggr  !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return false;


    }

    /*messageBox.information(0, "Info", "Selected experiment folder !!!");
    messageBox.setFixedSize(500, 200);
    messageBox.show();*/
    emit widgetClosed();

    return true;



}
void DataSelectionWidget::selectExperimentFolderSlot()
{

    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    dialog.setOption(QFileDialog::ShowDirsOnly, false);
    QString experimentFolder = dialog.getExistingDirectory(this, "Choose Experiment Folder");
    QMessageBox messageBox;
    if ( isCurrentFolderExperiment(experimentFolder)  ){
        

            messageBox.information(0, "Info", "Selected folder is experiment");
            messageBox.setFixedSize(500, 200);
            messageBox.show();
           // return;
    }
    

 
   /* if (experimentFolder == "")
    {
        messageBox.critical(0, "Error", "Experiment folder  was not selected !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }

    

   // experimentFolder
    QDir const source(experimentFolder);
    if (!source.exists())
        return;
        
   
    QStringList const folders = source.entryList(QDir::NoDot | QDir::NoDotDot | QDir::Dirs);

    QStringList const all = folders;
    QVector<QString> folderNamesVec;
    bool isFolderContainsRaven = false;
    for (QString const& name : all)
    {
        QString const fullPathName = experimentFolder + QDir::separator() + name;
        if (QFileInfo(fullPathName).isDir())
        {
            folderNamesVec.push_back(fullPathName);
        }
       
    }
    for (int f = 0; f < folderNamesVec.size(); ++f) {
        if (QFileInfo(folderNamesVec[f]).fileName() == "Raven") {
            //if 
            isFolderContainsRaven = true;
            break;
        }
    }
    if (!isFolderContainsRaven) {
        messageBox.critical(0, "Error", "Selected folder does not contains  Raven folder !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        return;
    }
    
    */

 

    m_experimentFolderPath = experimentFolder;
    m_selectedDataType = 5;
   // this->hide();
   // emit widgetClosed();
}

void DataSelectionWidget::applySelectedDataSlot()
{
    QMessageBox messageBox;
    switch (m_selectedDataType)
    {
    case 1:
        
        messageBox.information(0, "Apply", "Selected one depth image !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        break;
    case 2:
        messageBox.information(0, "Apply", "Selected depth and aggr image pair !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        break;
    case 3:
        messageBox.information(0, "Apply", "Selected folder with only depth images !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        break;

    case 4:
        messageBox.information(0, "Apply", "Selected folder with depth and aggr images !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        break;

    case 5:
        messageBox.information(0, "Apply", "Selected experiment folder !!!");
        messageBox.setFixedSize(500, 200);
        messageBox.show();
        break;

    default:
        break;
    }
    this->hide();
    emit widgetClosed();
}


void DataSelectionWidget::closeEvent(QCloseEvent* ev)
{

    QWidget::closeEvent(ev);
    blockSignals(false);
    ev->ignore();
    emit widgetClosed();
}

unsigned int DataSelectionWidget::getSelectedDataType()
{

    return m_selectedDataType;
}


QString DataSelectionWidget::getSimpleDepthPath() const
{
    return m_simpleDepthImagePath;
}
std::pair<QString, QString> DataSelectionWidget::getDepthAndAggrImagePath() const
{
    return m_depthAndAggrImagePath;

}
QString DataSelectionWidget::getDepthImagesFolderPath() const
{
    return m_depthImagesFolderPath;
}
QString DataSelectionWidget::getDepthAndAggrImagesFolderPath() const
{
    return m_depthAndAggrImagesFolderPath;

}
QString DataSelectionWidget::getExperimentFolderPath() const
{
    return m_experimentFolderPath;
}


DataSelectionWidget::~DataSelectionWidget()
{

}