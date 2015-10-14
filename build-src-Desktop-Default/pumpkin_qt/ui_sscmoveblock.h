/********************************************************************************
** Form generated from reading UI file 'sscmoveblock.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SSCMOVEBLOCK_H
#define UI_SSCMOVEBLOCK_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpinBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SSCMoveBlocksDesign
{
public:
    QVBoxLayout *verticalLayout;
    QFrame *frame;
    QGridLayout *gridLayout;
    QLabel *blockLabel;
    QSlider *speedSlider;
    QSpinBox *speedSpin;
    QSlider *pulseSlider;
    QLabel *pulseLabel;
    QLabel *speedLabel;
    QSpinBox *pulseSpin;
    QPushButton *activeButton;

    void setupUi(QWidget *SSCMoveBlocksDesign)
    {
        if (SSCMoveBlocksDesign->objectName().isEmpty())
            SSCMoveBlocksDesign->setObjectName(QString::fromUtf8("SSCMoveBlocksDesign"));
        SSCMoveBlocksDesign->resize(187, 224);
        verticalLayout = new QVBoxLayout(SSCMoveBlocksDesign);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        frame = new QFrame(SSCMoveBlocksDesign);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setFrameShape(QFrame::Panel);
        frame->setFrameShadow(QFrame::Raised);
        gridLayout = new QGridLayout(frame);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        blockLabel = new QLabel(frame);
        blockLabel->setObjectName(QString::fromUtf8("blockLabel"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(blockLabel->sizePolicy().hasHeightForWidth());
        blockLabel->setSizePolicy(sizePolicy);
        blockLabel->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(blockLabel, 0, 0, 1, 2);

        speedSlider = new QSlider(frame);
        speedSlider->setObjectName(QString::fromUtf8("speedSlider"));
        speedSlider->setMaximum(65535);
        speedSlider->setPageStep(100);
        speedSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(speedSlider, 4, 0, 1, 2);

        speedSpin = new QSpinBox(frame);
        speedSpin->setObjectName(QString::fromUtf8("speedSpin"));
        speedSpin->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        speedSpin->setMaximum(65535);

        gridLayout->addWidget(speedSpin, 3, 1, 1, 1);

        pulseSlider = new QSlider(frame);
        pulseSlider->setObjectName(QString::fromUtf8("pulseSlider"));
        pulseSlider->setPageStep(100);
        pulseSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(pulseSlider, 2, 0, 1, 2);

        pulseLabel = new QLabel(frame);
        pulseLabel->setObjectName(QString::fromUtf8("pulseLabel"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(pulseLabel->sizePolicy().hasHeightForWidth());
        pulseLabel->setSizePolicy(sizePolicy1);
        pulseLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(pulseLabel, 1, 0, 1, 1);

        speedLabel = new QLabel(frame);
        speedLabel->setObjectName(QString::fromUtf8("speedLabel"));
        sizePolicy1.setHeightForWidth(speedLabel->sizePolicy().hasHeightForWidth());
        speedLabel->setSizePolicy(sizePolicy1);
        speedLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(speedLabel, 3, 0, 1, 1);

        pulseSpin = new QSpinBox(frame);
        pulseSpin->setObjectName(QString::fromUtf8("pulseSpin"));
        sizePolicy.setHeightForWidth(pulseSpin->sizePolicy().hasHeightForWidth());
        pulseSpin->setSizePolicy(sizePolicy);
        pulseSpin->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(pulseSpin, 1, 1, 1, 1);

        activeButton = new QPushButton(frame);
        activeButton->setObjectName(QString::fromUtf8("activeButton"));
        activeButton->setCheckable(true);

        gridLayout->addWidget(activeButton, 5, 0, 1, 2);


        verticalLayout->addWidget(frame);


        retranslateUi(SSCMoveBlocksDesign);
        QObject::connect(pulseSpin, SIGNAL(valueChanged(int)), pulseSlider, SLOT(setValue(int)));
        QObject::connect(pulseSlider, SIGNAL(valueChanged(int)), pulseSpin, SLOT(setValue(int)));
        QObject::connect(speedSpin, SIGNAL(valueChanged(int)), speedSlider, SLOT(setValue(int)));
        QObject::connect(speedSlider, SIGNAL(valueChanged(int)), speedSpin, SLOT(setValue(int)));

        QMetaObject::connectSlotsByName(SSCMoveBlocksDesign);
    } // setupUi

    void retranslateUi(QWidget *SSCMoveBlocksDesign)
    {
        SSCMoveBlocksDesign->setWindowTitle(QApplication::translate("SSCMoveBlocksDesign", "Form", 0, QApplication::UnicodeUTF8));
        blockLabel->setText(QApplication::translate("SSCMoveBlocksDesign", "Block Name", 0, QApplication::UnicodeUTF8));
        pulseLabel->setText(QApplication::translate("SSCMoveBlocksDesign", "Pulse: ", 0, QApplication::UnicodeUTF8));
        speedLabel->setText(QApplication::translate("SSCMoveBlocksDesign", "Speed: ", 0, QApplication::UnicodeUTF8));
        activeButton->setText(QApplication::translate("SSCMoveBlocksDesign", "Activate", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SSCMoveBlocksDesign: public Ui_SSCMoveBlocksDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SSCMOVEBLOCK_H
