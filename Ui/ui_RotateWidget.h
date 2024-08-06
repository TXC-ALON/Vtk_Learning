/********************************************************************************
** Form generated from reading UI file 'RotateWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ROTATEWIDGET_H
#define UI_ROTATEWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RotateWidget
{
public:
    QGridLayout *gridLayout_4;
    QFrame *line_2;
    QSpacerItem *verticalSpacer;
    QPushButton *pushButtonApply;
    QSpacerItem *horizontalSpacer;
    QFrame *frame;
    QGridLayout *gridLayout_3;
    QGroupBox *groupBoxMode;
    QHBoxLayout *horizontalLayout;
    QRadioButton *radioButtonManual;
    QRadioButton *radioButtonAuto;
    QFrame *line;
    QGroupBox *groupBoxManual;
    QGridLayout *gridLayout;
    QDoubleSpinBox *doubleSpinBoxX;
    QDoubleSpinBox *doubleSpinBoxZ;
    QLabel *labelY;
    QLabel *labelmm3;
    QLabel *labelmm2;
    QLabel *labelmm1;
    QDoubleSpinBox *doubleSpinBoxY;
    QLabel *labelX;
    QLabel *labelZ;
    QGroupBox *groupBoxAuto;
    QGridLayout *gridLayout_2;
    QDoubleSpinBox *doubleSpinBoxAngle;
    QLabel *labelCircle;
    QLabel *labelmm1_2;
    QLabel *labelmm2_2;
    QCheckBox *checkBoxAjust;
    QLabel *labelAngle;
    QCheckBox *checkBoxFlip;
    QDoubleSpinBox *doubleSpinBoxCircle;
    QCheckBox *checkBoxAutoAujst;
    QPushButton *pushButtonClose;
    QPushButton *pushButtonOK;

    void setupUi(QWidget *RotateWidget)
    {
        if (RotateWidget->objectName().isEmpty())
            RotateWidget->setObjectName(QString::fromUtf8("RotateWidget"));
        RotateWidget->resize(417, 534);
        gridLayout_4 = new QGridLayout(RotateWidget);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        gridLayout_4->setHorizontalSpacing(0);
        line_2 = new QFrame(RotateWidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        gridLayout_4->addWidget(line_2, 2, 0, 1, 4);

        verticalSpacer = new QSpacerItem(20, 173, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_4->addItem(verticalSpacer, 1, 0, 1, 1);

        pushButtonApply = new QPushButton(RotateWidget);
        pushButtonApply->setObjectName(QString::fromUtf8("pushButtonApply"));
        pushButtonApply->setMinimumSize(QSize(75, 40));

        gridLayout_4->addWidget(pushButtonApply, 3, 1, 1, 1);

        horizontalSpacer = new QSpacerItem(171, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_4->addItem(horizontalSpacer, 3, 0, 1, 1);

        frame = new QFrame(RotateWidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        gridLayout_3 = new QGridLayout(frame);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setHorizontalSpacing(6);
        groupBoxMode = new QGroupBox(frame);
        groupBoxMode->setObjectName(QString::fromUtf8("groupBoxMode"));
        horizontalLayout = new QHBoxLayout(groupBoxMode);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        radioButtonManual = new QRadioButton(groupBoxMode);
        radioButtonManual->setObjectName(QString::fromUtf8("radioButtonManual"));
        radioButtonManual->setMinimumSize(QSize(0, 25));
        radioButtonManual->setChecked(false);

        horizontalLayout->addWidget(radioButtonManual);

        radioButtonAuto = new QRadioButton(groupBoxMode);
        radioButtonAuto->setObjectName(QString::fromUtf8("radioButtonAuto"));
        radioButtonAuto->setMinimumSize(QSize(0, 25));

        horizontalLayout->addWidget(radioButtonAuto);


        gridLayout_3->addWidget(groupBoxMode, 0, 0, 1, 1);

        line = new QFrame(frame);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout_3->addWidget(line, 1, 0, 1, 2);

        groupBoxManual = new QGroupBox(frame);
        groupBoxManual->setObjectName(QString::fromUtf8("groupBoxManual"));
        gridLayout = new QGridLayout(groupBoxManual);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        doubleSpinBoxX = new QDoubleSpinBox(groupBoxManual);
        doubleSpinBoxX->setObjectName(QString::fromUtf8("doubleSpinBoxX"));
        doubleSpinBoxX->setMinimumSize(QSize(100, 25));
        doubleSpinBoxX->setDecimals(0);

        gridLayout->addWidget(doubleSpinBoxX, 0, 1, 1, 1);

        doubleSpinBoxZ = new QDoubleSpinBox(groupBoxManual);
        doubleSpinBoxZ->setObjectName(QString::fromUtf8("doubleSpinBoxZ"));
        doubleSpinBoxZ->setMinimumSize(QSize(100, 25));
        doubleSpinBoxZ->setDecimals(0);

        gridLayout->addWidget(doubleSpinBoxZ, 2, 1, 1, 1);

        labelY = new QLabel(groupBoxManual);
        labelY->setObjectName(QString::fromUtf8("labelY"));
        labelY->setMinimumSize(QSize(20, 25));

        gridLayout->addWidget(labelY, 1, 0, 1, 1);

        labelmm3 = new QLabel(groupBoxManual);
        labelmm3->setObjectName(QString::fromUtf8("labelmm3"));
        labelmm3->setMinimumSize(QSize(20, 25));

        gridLayout->addWidget(labelmm3, 2, 2, 1, 1);

        labelmm2 = new QLabel(groupBoxManual);
        labelmm2->setObjectName(QString::fromUtf8("labelmm2"));
        labelmm2->setMinimumSize(QSize(20, 25));

        gridLayout->addWidget(labelmm2, 1, 2, 1, 1);

        labelmm1 = new QLabel(groupBoxManual);
        labelmm1->setObjectName(QString::fromUtf8("labelmm1"));
        labelmm1->setMinimumSize(QSize(20, 25));

        gridLayout->addWidget(labelmm1, 0, 2, 1, 1);

        doubleSpinBoxY = new QDoubleSpinBox(groupBoxManual);
        doubleSpinBoxY->setObjectName(QString::fromUtf8("doubleSpinBoxY"));
        doubleSpinBoxY->setMinimumSize(QSize(100, 25));
        doubleSpinBoxY->setDecimals(0);

        gridLayout->addWidget(doubleSpinBoxY, 1, 1, 1, 1);

        labelX = new QLabel(groupBoxManual);
        labelX->setObjectName(QString::fromUtf8("labelX"));
        labelX->setMinimumSize(QSize(20, 25));

        gridLayout->addWidget(labelX, 0, 0, 1, 1);

        labelZ = new QLabel(groupBoxManual);
        labelZ->setObjectName(QString::fromUtf8("labelZ"));
        labelZ->setMinimumSize(QSize(20, 25));

        gridLayout->addWidget(labelZ, 2, 0, 1, 1);


        gridLayout_3->addWidget(groupBoxManual, 2, 0, 1, 1);

        groupBoxAuto = new QGroupBox(frame);
        groupBoxAuto->setObjectName(QString::fromUtf8("groupBoxAuto"));
        gridLayout_2 = new QGridLayout(groupBoxAuto);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        doubleSpinBoxAngle = new QDoubleSpinBox(groupBoxAuto);
        doubleSpinBoxAngle->setObjectName(QString::fromUtf8("doubleSpinBoxAngle"));
        doubleSpinBoxAngle->setMinimumSize(QSize(100, 25));
        doubleSpinBoxAngle->setDecimals(0);

        gridLayout_2->addWidget(doubleSpinBoxAngle, 2, 1, 1, 1);

        labelCircle = new QLabel(groupBoxAuto);
        labelCircle->setObjectName(QString::fromUtf8("labelCircle"));
        labelCircle->setMinimumSize(QSize(0, 25));

        gridLayout_2->addWidget(labelCircle, 3, 0, 1, 1);

        labelmm1_2 = new QLabel(groupBoxAuto);
        labelmm1_2->setObjectName(QString::fromUtf8("labelmm1_2"));
        labelmm1_2->setMinimumSize(QSize(20, 25));

        gridLayout_2->addWidget(labelmm1_2, 2, 2, 1, 1);

        labelmm2_2 = new QLabel(groupBoxAuto);
        labelmm2_2->setObjectName(QString::fromUtf8("labelmm2_2"));
        labelmm2_2->setMinimumSize(QSize(20, 25));

        gridLayout_2->addWidget(labelmm2_2, 3, 2, 1, 1);

        checkBoxAjust = new QCheckBox(groupBoxAuto);
        checkBoxAjust->setObjectName(QString::fromUtf8("checkBoxAjust"));
        checkBoxAjust->setChecked(true);

        gridLayout_2->addWidget(checkBoxAjust, 1, 0, 1, 2);

        labelAngle = new QLabel(groupBoxAuto);
        labelAngle->setObjectName(QString::fromUtf8("labelAngle"));
        labelAngle->setMinimumSize(QSize(0, 25));

        gridLayout_2->addWidget(labelAngle, 2, 0, 1, 1);

        checkBoxFlip = new QCheckBox(groupBoxAuto);
        checkBoxFlip->setObjectName(QString::fromUtf8("checkBoxFlip"));

        gridLayout_2->addWidget(checkBoxFlip, 0, 0, 1, 2);

        doubleSpinBoxCircle = new QDoubleSpinBox(groupBoxAuto);
        doubleSpinBoxCircle->setObjectName(QString::fromUtf8("doubleSpinBoxCircle"));
        doubleSpinBoxCircle->setMinimumSize(QSize(100, 25));
        doubleSpinBoxCircle->setDecimals(0);

        gridLayout_2->addWidget(doubleSpinBoxCircle, 3, 1, 1, 1);

        checkBoxAutoAujst = new QCheckBox(groupBoxAuto);
        checkBoxAutoAujst->setObjectName(QString::fromUtf8("checkBoxAutoAujst"));

        gridLayout_2->addWidget(checkBoxAutoAujst, 4, 0, 1, 2);


        gridLayout_3->addWidget(groupBoxAuto, 2, 1, 1, 1);


        gridLayout_4->addWidget(frame, 0, 0, 1, 4);

        pushButtonClose = new QPushButton(RotateWidget);
        pushButtonClose->setObjectName(QString::fromUtf8("pushButtonClose"));
        pushButtonClose->setMinimumSize(QSize(75, 40));

        gridLayout_4->addWidget(pushButtonClose, 3, 3, 1, 1);

        pushButtonOK = new QPushButton(RotateWidget);
        pushButtonOK->setObjectName(QString::fromUtf8("pushButtonOK"));
        pushButtonOK->setMinimumSize(QSize(75, 40));

        gridLayout_4->addWidget(pushButtonOK, 3, 2, 1, 1);


        retranslateUi(RotateWidget);

        QMetaObject::connectSlotsByName(RotateWidget);
    } // setupUi

    void retranslateUi(QWidget *RotateWidget)
    {
        RotateWidget->setWindowTitle(QCoreApplication::translate("RotateWidget", "\346\227\213\350\275\254", nullptr));
        pushButtonApply->setText(QCoreApplication::translate("RotateWidget", "\345\272\224\347\224\250", nullptr));
        groupBoxMode->setTitle(QCoreApplication::translate("RotateWidget", "\346\227\213\350\275\254\346\250\241\345\274\217", nullptr));
        radioButtonManual->setText(QCoreApplication::translate("RotateWidget", "\346\211\213\345\212\250\345\247\277\346\200\201", nullptr));
        radioButtonAuto->setText(QCoreApplication::translate("RotateWidget", "\350\207\252\345\212\250\345\247\277\346\200\201", nullptr));
        groupBoxManual->setTitle(QCoreApplication::translate("RotateWidget", "\346\227\213\350\275\254\350\247\222\345\272\246", nullptr));
        labelY->setText(QCoreApplication::translate("RotateWidget", "Y", nullptr));
        labelmm3->setText(QCoreApplication::translate("RotateWidget", "\302\260", nullptr));
        labelmm2->setText(QCoreApplication::translate("RotateWidget", "\302\260", nullptr));
        labelmm1->setText(QCoreApplication::translate("RotateWidget", "\302\260", nullptr));
        labelX->setText(QCoreApplication::translate("RotateWidget", "X", nullptr));
        labelZ->setText(QCoreApplication::translate("RotateWidget", "Z", nullptr));
        groupBoxAuto->setTitle(QCoreApplication::translate("RotateWidget", "\345\217\202\346\225\260\350\256\276\347\275\256", nullptr));
        labelCircle->setText(QCoreApplication::translate("RotateWidget", "\345\276\256\350\260\203\346\255\245\351\225\277", nullptr));
        labelmm1_2->setText(QCoreApplication::translate("RotateWidget", "\302\260", nullptr));
        labelmm2_2->setText(QCoreApplication::translate("RotateWidget", "\302\260", nullptr));
        checkBoxAjust->setText(QCoreApplication::translate("RotateWidget", "\345\247\277\346\200\201\345\276\256\350\260\203", nullptr));
        labelAngle->setText(QCoreApplication::translate("RotateWidget", "\350\247\222\345\272\246\351\230\210\345\200\274", nullptr));
        checkBoxFlip->setText(QCoreApplication::translate("RotateWidget", "\344\275\215\347\275\256\347\277\273\350\275\254", nullptr));
        checkBoxAutoAujst->setText(QCoreApplication::translate("RotateWidget", "\346\224\257\346\236\266\350\207\252\345\212\250\350\260\203\346\225\264(.ipd\346\226\207\344\273\266)", nullptr));
        pushButtonClose->setText(QCoreApplication::translate("RotateWidget", "\345\205\263\351\227\255", nullptr));
        pushButtonOK->setText(QCoreApplication::translate("RotateWidget", "\347\241\256\345\256\232", nullptr));
    } // retranslateUi

};

namespace Ui {
    class RotateWidget: public Ui_RotateWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROTATEWIDGET_H
