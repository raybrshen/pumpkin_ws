/********************************************************************************
** Form generated from reading UI file 'sscmovecommand.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SSCMOVECOMMAND_H
#define UI_SSCMOVECOMMAND_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SSCMoveCommandDesign
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QTabWidget *moveTab;
    QFrame *frame;
    QHBoxLayout *horizontalLayout;
    QLabel *timeLabel;
    QSpinBox *timeSpin;
    QWidget *widget;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *sendButton;
    QPushButton *cancelButton;

    void setupUi(QDialog *SSCMoveCommandDesign)
    {
        if (SSCMoveCommandDesign->objectName().isEmpty())
            SSCMoveCommandDesign->setObjectName(QString::fromUtf8("SSCMoveCommandDesign"));
        SSCMoveCommandDesign->resize(259, 401);
        verticalLayout = new QVBoxLayout(SSCMoveCommandDesign);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label = new QLabel(SSCMoveCommandDesign);
        label->setObjectName(QString::fromUtf8("label"));
        QFont font;
        font.setFamily(QString::fromUtf8("Liberation Sans"));
        font.setPointSize(14);
        font.setBold(true);
        font.setWeight(75);
        font.setStyleStrategy(QFont::PreferAntialias);
        label->setFont(font);
        label->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(label);

        moveTab = new QTabWidget(SSCMoveCommandDesign);
        moveTab->setObjectName(QString::fromUtf8("moveTab"));

        verticalLayout->addWidget(moveTab);

        frame = new QFrame(SSCMoveCommandDesign);
        frame->setObjectName(QString::fromUtf8("frame"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        horizontalLayout = new QHBoxLayout(frame);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        timeLabel = new QLabel(frame);
        timeLabel->setObjectName(QString::fromUtf8("timeLabel"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(timeLabel->sizePolicy().hasHeightForWidth());
        timeLabel->setSizePolicy(sizePolicy1);

        horizontalLayout->addWidget(timeLabel);

        timeSpin = new QSpinBox(frame);
        timeSpin->setObjectName(QString::fromUtf8("timeSpin"));
        timeSpin->setMaximum(65535);

        horizontalLayout->addWidget(timeSpin);


        verticalLayout->addWidget(frame);

        widget = new QWidget(SSCMoveCommandDesign);
        widget->setObjectName(QString::fromUtf8("widget"));
        horizontalLayout_2 = new QHBoxLayout(widget);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        sendButton = new QPushButton(widget);
        sendButton->setObjectName(QString::fromUtf8("sendButton"));

        horizontalLayout_2->addWidget(sendButton);

        cancelButton = new QPushButton(widget);
        cancelButton->setObjectName(QString::fromUtf8("cancelButton"));

        horizontalLayout_2->addWidget(cancelButton);


        verticalLayout->addWidget(widget);


        retranslateUi(SSCMoveCommandDesign);

        QMetaObject::connectSlotsByName(SSCMoveCommandDesign);
    } // setupUi

    void retranslateUi(QDialog *SSCMoveCommandDesign)
    {
        SSCMoveCommandDesign->setWindowTitle(QApplication::translate("SSCMoveCommandDesign", "Dialog", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SSCMoveCommandDesign", "SSC Command Sender", 0, QApplication::UnicodeUTF8));
        timeLabel->setText(QApplication::translate("SSCMoveCommandDesign", "Time: ", 0, QApplication::UnicodeUTF8));
        sendButton->setText(QApplication::translate("SSCMoveCommandDesign", "Send Command", 0, QApplication::UnicodeUTF8));
        cancelButton->setText(QApplication::translate("SSCMoveCommandDesign", "Cancel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SSCMoveCommandDesign: public Ui_SSCMoveCommandDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SSCMOVECOMMAND_H
