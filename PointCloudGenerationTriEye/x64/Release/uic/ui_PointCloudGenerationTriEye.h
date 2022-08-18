/********************************************************************************
** Form generated from reading UI file 'PointCloudGenerationTriEye.ui'
**
** Created by: Qt User Interface Compiler version 5.9.9
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_POINTCLOUDGENERATIONTRIEYE_H
#define UI_POINTCLOUDGENERATIONTRIEYE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PointCloudGenerationTriEyeClass
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *PointCloudGenerationTriEyeClass)
    {
        if (PointCloudGenerationTriEyeClass->objectName().isEmpty())
            PointCloudGenerationTriEyeClass->setObjectName(QStringLiteral("PointCloudGenerationTriEyeClass"));
        PointCloudGenerationTriEyeClass->resize(600, 400);
        menuBar = new QMenuBar(PointCloudGenerationTriEyeClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        PointCloudGenerationTriEyeClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(PointCloudGenerationTriEyeClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        PointCloudGenerationTriEyeClass->addToolBar(mainToolBar);
        centralWidget = new QWidget(PointCloudGenerationTriEyeClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        PointCloudGenerationTriEyeClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(PointCloudGenerationTriEyeClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        PointCloudGenerationTriEyeClass->setStatusBar(statusBar);

        retranslateUi(PointCloudGenerationTriEyeClass);

        QMetaObject::connectSlotsByName(PointCloudGenerationTriEyeClass);
    } // setupUi

    void retranslateUi(QMainWindow *PointCloudGenerationTriEyeClass)
    {
        PointCloudGenerationTriEyeClass->setWindowTitle(QApplication::translate("PointCloudGenerationTriEyeClass", "PointCloudGenerationTriEye", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PointCloudGenerationTriEyeClass: public Ui_PointCloudGenerationTriEyeClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POINTCLOUDGENERATIONTRIEYE_H
