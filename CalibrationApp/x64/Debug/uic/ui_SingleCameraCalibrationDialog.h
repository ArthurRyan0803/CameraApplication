/********************************************************************************
** Form generated from reading UI file 'SingleCameraCalibrationDialogQWgzFD.ui'
**
** Created by: Qt User Interface Compiler version 6.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef SINGLECAMERACALIBRATIONDIALOGQWGZFD_H
#define SINGLECAMERACALIBRATIONDIALOGQWGZFD_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SingleCameraCalibrationDialogClass
{
public:
    QGridLayout *gridLayout;
    QWidget *canvas;
    QGroupBox *groupBox_2;
    QFormLayout *formLayout;
    QLabel *label;
    QComboBox *cbCalibPattern;
    QCheckBox *cbDetectCalibBoard;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout;
    QPushButton *btnGrab;
    QPushButton *btnCapture;
    QSpacerItem *verticalSpacer;
    QPushButton *btnExit;

    void setupUi(QDialog *SingleCameraCalibrationDialogClass)
    {
        if (SingleCameraCalibrationDialogClass->objectName().isEmpty())
            SingleCameraCalibrationDialogClass->setObjectName(QString::fromUtf8("SingleCameraCalibrationDialogClass"));
        SingleCameraCalibrationDialogClass->resize(1352, 853);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(SingleCameraCalibrationDialogClass->sizePolicy().hasHeightForWidth());
        SingleCameraCalibrationDialogClass->setSizePolicy(sizePolicy);
        SingleCameraCalibrationDialogClass->setMinimumSize(QSize(0, 0));
        gridLayout = new QGridLayout(SingleCameraCalibrationDialogClass);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        canvas = new QWidget(SingleCameraCalibrationDialogClass);
        canvas->setObjectName(QString::fromUtf8("canvas"));

        gridLayout->addWidget(canvas, 0, 0, 2, 1);

        groupBox_2 = new QGroupBox(SingleCameraCalibrationDialogClass);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        formLayout = new QFormLayout(groupBox_2);
        formLayout->setSpacing(6);
        formLayout->setContentsMargins(11, 11, 11, 11);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        label = new QLabel(groupBox_2);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        cbCalibPattern = new QComboBox(groupBox_2);
        cbCalibPattern->setObjectName(QString::fromUtf8("cbCalibPattern"));

        formLayout->setWidget(0, QFormLayout::FieldRole, cbCalibPattern);

        cbDetectCalibBoard = new QCheckBox(groupBox_2);
        cbDetectCalibBoard->setObjectName(QString::fromUtf8("cbDetectCalibBoard"));
        cbDetectCalibBoard->setEnabled(false);

        formLayout->setWidget(1, QFormLayout::SpanningRole, cbDetectCalibBoard);


        gridLayout->addWidget(groupBox_2, 0, 1, 1, 1);

        groupBox = new QGroupBox(SingleCameraCalibrationDialogClass);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy1);
        verticalLayout = new QVBoxLayout(groupBox);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        btnGrab = new QPushButton(groupBox);
        btnGrab->setObjectName(QString::fromUtf8("btnGrab"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(btnGrab->sizePolicy().hasHeightForWidth());
        btnGrab->setSizePolicy(sizePolicy2);
        btnGrab->setMinimumSize(QSize(200, 30));
        btnGrab->setMaximumSize(QSize(16777215, 30));

        verticalLayout->addWidget(btnGrab);

        btnCapture = new QPushButton(groupBox);
        btnCapture->setObjectName(QString::fromUtf8("btnCapture"));
        sizePolicy2.setHeightForWidth(btnCapture->sizePolicy().hasHeightForWidth());
        btnCapture->setSizePolicy(sizePolicy2);
        btnCapture->setMinimumSize(QSize(200, 30));
        btnCapture->setMaximumSize(QSize(16777215, 30));

        verticalLayout->addWidget(btnCapture);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        btnExit = new QPushButton(groupBox);
        btnExit->setObjectName(QString::fromUtf8("btnExit"));
        sizePolicy2.setHeightForWidth(btnExit->sizePolicy().hasHeightForWidth());
        btnExit->setSizePolicy(sizePolicy2);
        btnExit->setMinimumSize(QSize(200, 30));
        btnExit->setMaximumSize(QSize(16777215, 30));

        verticalLayout->addWidget(btnExit);


        gridLayout->addWidget(groupBox, 1, 1, 1, 1);


        retranslateUi(SingleCameraCalibrationDialogClass);

        QMetaObject::connectSlotsByName(SingleCameraCalibrationDialogClass);
    } // setupUi

    void retranslateUi(QDialog *SingleCameraCalibrationDialogClass)
    {
        SingleCameraCalibrationDialogClass->setWindowTitle(QCoreApplication::translate("SingleCameraCalibrationDialogClass", "Single Camera Calibration", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("SingleCameraCalibrationDialogClass", "Options", nullptr));
        label->setText(QCoreApplication::translate("SingleCameraCalibrationDialogClass", "Calibration Pattern:", nullptr));
        cbDetectCalibBoard->setText(QCoreApplication::translate("SingleCameraCalibrationDialogClass", "Detect Calibration Board", nullptr));
        groupBox->setTitle(QCoreApplication::translate("SingleCameraCalibrationDialogClass", "Operation", nullptr));
        btnGrab->setText(QCoreApplication::translate("SingleCameraCalibrationDialogClass", "OneShot", nullptr));
        btnCapture->setText(QCoreApplication::translate("SingleCameraCalibrationDialogClass", "Start Cappture", nullptr));
        btnExit->setText(QCoreApplication::translate("SingleCameraCalibrationDialogClass", "Exit", nullptr));
    } // retranslateUi

};

namespace Ui {
    class SingleCameraCalibrationDialogClass: public Ui_SingleCameraCalibrationDialogClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // SINGLECAMERACALIBRATIONDIALOGQWGZFD_H
