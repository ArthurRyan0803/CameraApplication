/********************************************************************************
** Form generated from reading UI file 'MainNavigationWindowhPfqtE.ui'
**
** Created by: Qt User Interface Compiler version 6.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef MAINNAVIGATIONWINDOWHPFQTE_H
#define MAINNAVIGATIONWINDOWHPFQTE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainNavigationWindowClass
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QGroupBox *gbCameraSelection;
    QFormLayout *formLayout;
    QLabel *label_1;
    QComboBox *cbCamCategory;
    QLabel *label_2;
    QComboBox *cbCamId;
    QGroupBox *gbCalibration;
    QVBoxLayout *verticalLayout_2;
    QPushButton *btnSingleCalib;
    QToolBar *mainToolBar;

    void setupUi(QMainWindow *MainNavigationWindowClass)
    {
        if (MainNavigationWindowClass->objectName().isEmpty())
            MainNavigationWindowClass->setObjectName(QString::fromUtf8("MainNavigationWindowClass"));
        MainNavigationWindowClass->resize(400, 189);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainNavigationWindowClass->sizePolicy().hasHeightForWidth());
        MainNavigationWindowClass->setSizePolicy(sizePolicy);
        MainNavigationWindowClass->setMinimumSize(QSize(400, 0));
        MainNavigationWindowClass->setMaximumSize(QSize(400, 16777215));
        centralWidget = new QWidget(MainNavigationWindowClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        gbCameraSelection = new QGroupBox(centralWidget);
        gbCameraSelection->setObjectName(QString::fromUtf8("gbCameraSelection"));
        formLayout = new QFormLayout(gbCameraSelection);
        formLayout->setSpacing(6);
        formLayout->setContentsMargins(11, 11, 11, 11);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        label_1 = new QLabel(gbCameraSelection);
        label_1->setObjectName(QString::fromUtf8("label_1"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_1);

        cbCamCategory = new QComboBox(gbCameraSelection);
        cbCamCategory->setObjectName(QString::fromUtf8("cbCamCategory"));

        formLayout->setWidget(0, QFormLayout::FieldRole, cbCamCategory);

        label_2 = new QLabel(gbCameraSelection);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_2);

        cbCamId = new QComboBox(gbCameraSelection);
        cbCamId->setObjectName(QString::fromUtf8("cbCamId"));

        formLayout->setWidget(1, QFormLayout::FieldRole, cbCamId);


        verticalLayout->addWidget(gbCameraSelection);

        gbCalibration = new QGroupBox(centralWidget);
        gbCalibration->setObjectName(QString::fromUtf8("gbCalibration"));
        verticalLayout_2 = new QVBoxLayout(gbCalibration);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        btnSingleCalib = new QPushButton(gbCalibration);
        btnSingleCalib->setObjectName(QString::fromUtf8("btnSingleCalib"));
        btnSingleCalib->setMinimumSize(QSize(0, 35));
        btnSingleCalib->setMaximumSize(QSize(16777215, 35));

        verticalLayout_2->addWidget(btnSingleCalib);


        verticalLayout->addWidget(gbCalibration);

        MainNavigationWindowClass->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainNavigationWindowClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainNavigationWindowClass->addToolBar(Qt::TopToolBarArea, mainToolBar);

        retranslateUi(MainNavigationWindowClass);

        QMetaObject::connectSlotsByName(MainNavigationWindowClass);
    } // setupUi

    void retranslateUi(QMainWindow *MainNavigationWindowClass)
    {
        MainNavigationWindowClass->setWindowTitle(QCoreApplication::translate("MainNavigationWindowClass", "Camera Calibration", nullptr));
        gbCameraSelection->setTitle(QCoreApplication::translate("MainNavigationWindowClass", "Camera Selection", nullptr));
        label_1->setText(QCoreApplication::translate("MainNavigationWindowClass", "Camera Category:", nullptr));
        label_2->setText(QCoreApplication::translate("MainNavigationWindowClass", "Camera ID:", nullptr));
        gbCalibration->setTitle(QCoreApplication::translate("MainNavigationWindowClass", "Calibration", nullptr));
        btnSingleCalib->setText(QCoreApplication::translate("MainNavigationWindowClass", "Single Camera Calibration", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainNavigationWindowClass: public Ui_MainNavigationWindowClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // MAINNAVIGATIONWINDOWHPFQTE_H
